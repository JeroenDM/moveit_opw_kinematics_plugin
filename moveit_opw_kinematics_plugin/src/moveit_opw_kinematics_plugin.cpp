// #include <class_loader/class_loader.hpp>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_state/conversions.h>

// abs
#include <cstdlib>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
// #include <eigen_conversions/eigen_msg.h> // TODO replace this with tf2
#include <tf2_eigen/tf2_eigen.hpp>

// OPW kinematics
// OPW kinematics
#include "opw_kinematics/opw_io.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"
#include "rcl_interfaces/srv/get_parameters.hpp"

// register OPWKinematics as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>

CLASS_LOADER_REGISTER_CLASS(moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin, kinematics::KinematicsBase
)

namespace moveit_opw_kinematics_plugin {
  static rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_opw_kinematics_plugin.opw_kinematics_plugin");

  rclcpp::Clock MoveItOPWKinematicsPlugin::steady_clock_{RCL_STEADY_TIME};

  MoveItOPWKinematicsPlugin::MoveItOPWKinematicsPlugin() : initialized_(false) {
  }

  bool MoveItOPWKinematicsPlugin::initialize(const rclcpp::Node::SharedPtr &node,
                                             const moveit::core::RobotModel &robot_model,
                                             const std::string &group_name, const std::string &base_frame,
                                             const std::vector<std::string> &tip_frames,
                                             double search_discretization) {
    node_ = node;
    group_name_ = group_name;
    storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

    RCLCPP_INFO_STREAM(LOGGER, "MoveItOPWKinematicsPlugin initializing");

    joint_model_group_ = robot_model_->getJointModelGroup(group_name);
    if (!joint_model_group_)
      return false;

    if (!joint_model_group_->isChain()) {
      RCLCPP_ERROR(LOGGER, "Group '%s' is not a chain", group_name.c_str());
      return false;
    }
    if (!joint_model_group_->isSingleDOFJoints()) {
      RCLCPP_ERROR(LOGGER, "Group '%s' includes joints that have more than 1 DOF", group_name.c_str());
      return false;
    }

    // Get the dimension of the planning group
    dimension_ = joint_model_group_->getVariableCount();
    RCLCPP_INFO_STREAM(LOGGER, "Dimension planning group '"
                       << group_name << "': " << dimension_
                       << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                       << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

    // Copy joint names
    for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i) {
      ik_group_info_.joint_names.push_back(joint_model_group_->getActiveJointModelNames()[i]);
    }

    // Make sure all the tip links are in the link_names vector
    for (std::size_t i = 0; i < tip_frames_.size(); ++i) {
      if (!joint_model_group_->hasLinkModel(tip_frames_[i])) {
        RCLCPP_ERROR(LOGGER, "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                     group_name.c_str());
        return false;
      }
      ik_group_info_.link_names.push_back(tip_frames_[i]);
    }

    // Set up the joint state groups that we need
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();

    // set geometric parameters for opw model
    if (!setOPWParameters()) {
      RCLCPP_ERROR_STREAM(LOGGER, "Could not load OPW parameters. Please make "
                          "sure they are loaded on the parameter server and are of the correct type(s).");
      return false;
    }

    // Resolve the OPW-frame -> tip transform before the self-test, which
    // validates FK including it.
    if (!computeTipOffset()) {
      return false;
    }

    // check geometric parameters for opw model
    if (!selfTest()) {
      RCLCPP_ERROR_STREAM(LOGGER, "The OPW parameters loaded from the parameter "
                          "server appear to be incorrect (self-test failed).");
      return false;
    }

    initialized_ = true;
    //        ROS_DEBUG_NAMED("opw", "OPW kinematics solver initialized");
    RCLCPP_DEBUG(LOGGER, "OPW kinematics solver initialized");
    return true;
  }

  bool MoveItOPWKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int> &redundant_joints) {
    if (num_possible_redundant_joints_ < 0) {
      RCLCPP_ERROR(LOGGER, "This group cannot have redundant joints");
      return false;
    }
    if (redundant_joints.size() > static_cast<std::size_t>(num_possible_redundant_joints_)) {
      RCLCPP_ERROR(LOGGER, "This group can only have %d redundant joints", num_possible_redundant_joints_);
      return false;
    }

    redundant_joint_indices_ = redundant_joints;

    return true;
  }


  bool MoveItOPWKinematicsPlugin::isRedundantJoint(unsigned int index) const {
    for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
      if (redundant_joint_indices_[j] == index)
        return true;
    return false;
  }

  int MoveItOPWKinematicsPlugin::getJointIndex(const std::string &name) const {
    for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); i++) {
      if (ik_group_info_.joint_names[i] == name)
        return i;
    }
    return -1;
  }

  bool MoveItOPWKinematicsPlugin::timedOut(const rclcpp::Time &start_time, double duration) const {
    return ((node_->now() - start_time).seconds() >= duration);
  }

  bool MoveItOPWKinematicsPlugin::computeTipOffset() {
    tip_offset_ = Eigen::Isometry3d::Identity();

    // Same lookup chain as the geometric parameters: move_group cache,
    // then local node parameters.
    std::string opw_tool_frame;
    bool found = lookupCachedParam("opw_tool_frame", opw_tool_frame, std::string(""));
    if (!found || opw_tool_frame.empty()) {
      found = lookupParam(node_, "opw_tool_frame", opw_tool_frame, std::string(""));
    }

    if (!found || opw_tool_frame.empty()) {
      RCLCPP_INFO(LOGGER, "opw_tool_frame not set; OPW model reaches tip frame '%s' directly",
                  tip_frames_[0].c_str());
      return true;
    }
    if (opw_tool_frame == tip_frames_[0]) {
      RCLCPP_INFO(LOGGER, "opw_tool_frame equals tip frame '%s'; no tip offset needed",
                  tip_frames_[0].c_str());
      return true;
    }

    if (!robot_model_->hasLinkModel(opw_tool_frame)) {
      RCLCPP_ERROR(LOGGER, "opw_tool_frame '%s' is not a link of the robot model", opw_tool_frame.c_str());
      return false;
    }

    // The transform must be constant (only fixed joints to the tip); check
    // it at two configurations to catch a frame that moves.
    auto offset_at = [&](const std::array<double, 6> &q) {
      robot_state_->setJointGroupPositions(joint_model_group_, q.data());
      return Eigen::Isometry3d(robot_state_->getGlobalLinkTransform(opw_tool_frame).inverse() *
                               robot_state_->getGlobalLinkTransform(tip_frames_[0]));
    };
    const Eigen::Isometry3d offset_a = offset_at({0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    const Eigen::Isometry3d offset_b = offset_at({0.3, -0.2, 0.1, 0.4, -0.5, 0.6});
    robot_state_->setToDefaultValues();

    if (!offset_a.isApprox(offset_b, 1e-9)) {
      RCLCPP_ERROR(LOGGER,
                   "opw_tool_frame '%s' is not rigidly attached to tip frame '%s' "
                   "(the transform between them changes with the joint state)",
                   opw_tool_frame.c_str(), tip_frames_[0].c_str());
      return false;
    }

    tip_offset_ = offset_a;
    const auto t = tip_offset_.translation();
    RCLCPP_INFO(LOGGER, "OPW tip offset '%s' -> '%s': [%.5f, %.5f, %.5f]",
                opw_tool_frame.c_str(), tip_frames_[0].c_str(), t.x(), t.y(), t.z());
    return true;
  }

  bool MoveItOPWKinematicsPlugin::selfTest() {
    // First, make sure the offsets are loaded
    RCLCPP_INFO(LOGGER, "OPW offsets: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                opw_parameters_.offsets[0], opw_parameters_.offsets[1],
                opw_parameters_.offsets[2], opw_parameters_.offsets[3],
                opw_parameters_.offsets[4], opw_parameters_.offsets[5]);

    // Test with angles that make sense for both OPW and MoveIt
    const std::array<double, 6> test_angles = {0.1, -0.1, 0.2, -0.3, 0.5, -0.8};

    RCLCPP_INFO(LOGGER, "Self-test input angles: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
                test_angles[0], test_angles[1], test_angles[2],
                test_angles[3], test_angles[4], test_angles[5]);

    // OPW FK extended to the group tip by the fixed tip offset.
    Eigen::Isometry3d fk_pose_opw = opw_kinematics::forward(opw_parameters_, test_angles) * tip_offset_;

    // Give MoveIt the angles that OPW is actually using internally
    robot_state_->setJointGroupPositions(joint_model_group_, test_angles.data());
    // root join
    RCLCPP_INFO_STREAM(LOGGER, "global link name: " << base_frame_ );
    // Get FK from MoveIt
    auto fk_pose_moveit = robot_state_->getGlobalLinkTransform(tip_frames_[0]);
    auto base = robot_state_->getGlobalLinkTransform(base_frame_);
    fk_pose_moveit = base.inverse() * fk_pose_moveit;
    fk_pose_opw = base.inverse() * fk_pose_opw;

    if (!comparePoses(fk_pose_opw, fk_pose_moveit)) {
      // Debug output
      RCLCPP_ERROR(LOGGER, "Self-test failed");
      auto opw_pos = fk_pose_opw.translation();
      auto moveit_pos = fk_pose_moveit.translation();
      RCLCPP_ERROR(LOGGER, "OPW Pose: [%.4f, %.4f, %.4f]",
                   opw_pos.x(), opw_pos.y(), opw_pos.z());
      RCLCPP_ERROR_STREAM(LOGGER, "OPW Matrix: " << std::endl << fk_pose_opw.linear());
      RCLCPP_ERROR(LOGGER, "MoveIt position: [%.4f, %.4f, %.4f]",
                   moveit_pos.x(), moveit_pos.y(), moveit_pos.z());
      RCLCPP_ERROR_STREAM(LOGGER, "Moveit Matrix: " << std::endl << fk_pose_moveit.linear());

      return false;
    }

    RCLCPP_INFO(LOGGER, "OPW self-test passed!");
    robot_state_->setToDefaultValues();
    return true;
  }

  bool MoveItOPWKinematicsPlugin::comparePoses(Eigen::Isometry3d &Ta, Eigen::Isometry3d &Tb) {
    // Default tolerance
    float tolerance = 2e-3;

    // Try to load tolerance from parameters with same prefix search strategy
    std::vector<std::string> tolerance_param_paths = {
      "opw_self_test_tolerance",
      "robot_description_kinematics.manipulator_opw.opw_self_test_tolerance",
      "manipulator_opw.opw_self_test_tolerance",
      group_name_ + ".opw_self_test_tolerance"
    };

    for (const auto &param_path: tolerance_param_paths) {
      double temp_tolerance;
      if (lookupParam(node_, param_path, temp_tolerance, 2e-3)) {
        tolerance = static_cast<float>(temp_tolerance);
        RCLCPP_INFO_ONCE(LOGGER, "Using OPW self-test tolerance: %f from parameter: %s",
                         tolerance, param_path.c_str());
        break;
      }
    }

    auto Ra = Ta.rotation();
    auto Rb = Tb.rotation();
    for (int i = 0; i < Ra.rows(); ++i) {
      for (int j = 0; j < Ra.cols(); ++j) {
        if (std::abs(Ra(i, j) - Rb(i, j)) > tolerance) {
          RCLCPP_ERROR(LOGGER, "Pose orientation error on element (%d, %d).", i, j);
          RCLCPP_ERROR(LOGGER, "opw: %f, moveit: %f.", Ra(i, j), Rb(i, j));
          return false;
        }
      }
    }

    auto pa = Ta.translation();
    auto pb = Tb.translation();
    for (int i = 0; i < 3; ++i) {
      if (std::abs(pa(i) - pb(i)) > tolerance) {
        RCLCPP_ERROR(LOGGER, "Pose position error on element (%d).", i);
        RCLCPP_ERROR(LOGGER, "opw: %f, moveit: %f.", pa(i), pb(i));
        return false;
      }
    }
    return true;
  }

  bool MoveItOPWKinematicsPlugin::getPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                const std::vector<double> &ik_seed_state,
                                                std::vector<double> &solution,
                                                moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                const kinematics::KinematicsQueryOptions &options) const {
    const IKCallbackFn solution_callback = nullptr;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                            consistency_limits, options);
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state, double timeout,
                                                   std::vector<double> &solution,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions &options) const {
    const IKCallbackFn solution_callback = nullptr;
    std::vector<double> consistency_limits;

    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                            consistency_limits,
                            options);
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state, double timeout,
                                                   const std::vector<double> &consistency_limits,
                                                   std::vector<double> &solution,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions &options) const {
    const IKCallbackFn solution_callback = 0;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                            consistency_limits,
                            options);
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state, double timeout,
                                                   std::vector<double> &solution,
                                                   const IKCallbackFn &solution_callback,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions &options) const {
    std::vector<double> consistency_limits;
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                            consistency_limits,
                            options);
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state, double timeout,
                                                   const std::vector<double> &consistency_limits,
                                                   std::vector<double> &solution,
                                                   const IKCallbackFn &solution_callback,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions &options) const {
    return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code,
                            consistency_limits,
                            options);
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::msg::Pose &ik_pose,
                                                   const std::vector<double> &ik_seed_state, double timeout,
                                                   std::vector<double> &solution,
                                                   const IKCallbackFn &solution_callback,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const std::vector<double> &consistency_limits,
                                                   const kinematics::KinematicsQueryOptions &options) const {
    // Convert single pose into a vector of one pose
    std::vector<geometry_msgs::msg::Pose> ik_poses;
    ik_poses.push_back(ik_pose);

    return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback,
                            error_code,
                            options);
  }

  void MoveItOPWKinematicsPlugin::expandIKSolutions(std::vector<std::vector<double> > &solutions) const {
    const std::vector<const moveit::core::JointModel *> &ajms = joint_model_group_->getActiveJointModels();
    for (size_t i = 0; i < ajms.size(); ++i) {
      const moveit::core::JointModel *jm = ajms[i];
      if (jm->getVariableBounds().size() > 0) {
        for (auto &bounds: jm->getVariableBounds()) {
          // todo: what to do about continuous joints
          if (!bounds.position_bounded_)
            continue;

          std::vector<std::vector<double> > additional_solutions;
          for (auto &sol: solutions) {
            std::vector<double> down_sol(sol);
            while (down_sol[i] - 2.0 * M_PI > bounds.min_position_) {
              down_sol[i] -= 2.0 * M_PI;
              additional_solutions.push_back(down_sol);
            }
            std::vector<double> up_sol(sol);
            while (up_sol[i] + 2.0 * M_PI < bounds.max_position_) {
              up_sol[i] += 2.0 * M_PI;
              additional_solutions.push_back(up_sol);
            }
          }
          RCLCPP_DEBUG_STREAM(LOGGER,
                              "Found " << additional_solutions.size() << " additional solutions for j="
                              << i + 1);
          solutions.insert(solutions.end(), additional_solutions.begin(), additional_solutions.end());
        }
      }
    }
  }

  bool MoveItOPWKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::msg::Pose> &ik_poses,
                                                   const std::vector<double> &ik_seed_state, double /*timeout*/,
                                                   const std::vector<double> & /*consistency_limits*/,
                                                   std::vector<double> &solution,
                                                   const IKCallbackFn &solution_callback,
                                                   moveit_msgs::msg::MoveItErrorCodes &error_code,
                                                   const kinematics::KinematicsQueryOptions & /*options*/) const {
    // Check if active
    if (!initialized_) {
      RCLCPP_ERROR(LOGGER, "kinematics not active");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // Check if seed state correct
    if (ik_seed_state.size() != dimension_) {
      RCLCPP_ERROR_STREAM(LOGGER,
                          "Seed state must have size " << dimension_ << " instead of size "
                          << ik_seed_state.size());
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // Check that we have the same number of poses as tips
    if (tip_frames_.size() != ik_poses.size()) {
      RCLCPP_ERROR_STREAM(LOGGER, "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                          << tip_frames_.size()
                          << ") in searchPositionIK");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    Eigen::Isometry3d pose;
    tf2::fromMsg(ik_poses[0], pose);
    std::vector<std::vector<double> > solutions;
    if (!getAllIK(pose, solutions)) {
      RCLCPP_DEBUG_STREAM(LOGGER, "Failed to find IK solution");
      error_code.val = error_code.NO_IK_SOLUTION;
      return false;
    }

    // for all solutions, check if solution +-360° is still inside limits
    // An opw solution might be outside the joint limits, while the extended one is inside (e.g. asymmetric limits)
    // therefore first extend solution space, then apply joint limits later
    expandIKSolutions(solutions);

    RCLCPP_DEBUG_STREAM(LOGGER, "Now have " << solutions.size() << " potential solutions");

    std::vector<LimitObeyingSol> limit_obeying_solutions;

    for (auto &sol: solutions) {
      robot_state_->setJointGroupPositions(joint_model_group_, sol);
      if (!robot_state_->satisfiesBounds(joint_model_group_)) {
        RCLCPP_DEBUG_STREAM(LOGGER, "Solution is outside bounds");
        continue;
      }
      limit_obeying_solutions.push_back({sol, distance(sol, ik_seed_state)});
    }

    if (limit_obeying_solutions.empty()) {
      RCLCPP_DEBUG(LOGGER, "None of the solutions is within joint limits");
      return false;
    }

    RCLCPP_DEBUG_STREAM(LOGGER, "Solutions within limits: " << limit_obeying_solutions.size());

    // sort solutions by distance to seed state
    std::sort(limit_obeying_solutions.begin(), limit_obeying_solutions.end());

    if (!solution_callback) {
      solution = limit_obeying_solutions.front().value;
      return true;
    }

    for (auto &sol: limit_obeying_solutions) {
      solution_callback(ik_poses[0], sol.value, error_code);
      if (error_code.val == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        solution = sol.value;
        RCLCPP_DEBUG_STREAM(LOGGER, "Solution passes callback");
        return true;
      }
    }

    RCLCPP_DEBUG_STREAM(LOGGER, "No solution fullfilled requirements of solution callback");
    return false;
  }

  bool MoveItOPWKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::msg::Pose> &ik_poses,
                                                const std::vector<double> & /*ik_seed_state*/,
                                                std::vector<std::vector<double> > &solutions,
                                                KinematicsResult & /*result*/,
                                                const kinematics::KinematicsQueryOptions & /*options*/) const {
    if (ik_poses.size() > 1 || ik_poses.size() == 0) {
      RCLCPP_ERROR_STREAM(LOGGER, "You can only get all solutions for a single pose.");
      return false;
    }
    Eigen::Isometry3d pose;
    tf2::fromMsg(ik_poses[0], pose);
    return getAllIK(pose, solutions);
  }

  bool MoveItOPWKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names,
                                                const std::vector<double> &joint_angles,
                                                std::vector<geometry_msgs::msg::Pose> &poses) const {
    if (!initialized_) {
      RCLCPP_ERROR(LOGGER, "kinematics not active");
      return false;
    }
    poses.resize(link_names.size());
    if (joint_angles.size() != dimension_) {
      RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", dimension_);
      return false;
    }

    // Check that we have the same number of poses as tips
    if (tip_frames_.size() != poses.size()) {
      RCLCPP_ERROR_STREAM(LOGGER, "Mismatched number of pose requests (" << poses.size() << ") to tip frames ("
                          << tip_frames_.size()
                          << ") in searchPositionFK");
      return false;
    }

    std::array<double, 6> joint_angles_array{};
    std::copy_n(joint_angles.begin(), 6, joint_angles_array.begin());
    poses[0] = tf2::toMsg(
        Eigen::Isometry3d(opw_kinematics::forward(opw_parameters_, joint_angles_array) * tip_offset_));
    return true;
  };

  const std::vector<std::string> &MoveItOPWKinematicsPlugin::getJointNames() const {
    return ik_group_info_.joint_names;
  }

  const std::vector<std::string> &MoveItOPWKinematicsPlugin::getLinkNames() const {
    return ik_group_info_.link_names;
  }

  const std::vector<std::string> &MoveItOPWKinematicsPlugin::getVariableNames() const {
    return joint_model_group_->getVariableNames();
  }

  rcl_interfaces::srv::GetParameters::Response::SharedPtr
  MoveItOPWKinematicsPlugin::getParamsFromNode(const std::string &node_name, const std::string &parameter_name) {
    auto client = node_->create_client<rcl_interfaces::srv::GetParameters>("/move_group/get_parameters");
    while (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(LOGGER, "Waiting for server to be up");
    }

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = std::vector<std::string>{parameter_name};

    auto future = client->async_send_request(request);

    try {
      auto response = future.get();
      return response;
    } catch (const std::exception &e) {
      RCLCPP_ERROR(LOGGER, "Service call failed");
      throw e;
    }
  }

  bool MoveItOPWKinematicsPlugin::setOPWParameters() {
    RCLCPP_INFO(LOGGER, "Getting kinematic parameters from parameter server.");

    // Try to cache parameters from move_group first (non-blocking)
    bool cached = cacheParametersFromMoveGroup();
    if (cached) {
      RCLCPP_INFO(LOGGER, "Using cached parameters from move_group");
    } else {
      RCLCPP_INFO(LOGGER, "Using local parameters or async lookup");
    }

    // Load geometric parameters individually
    std::map<std::string, double> geometric_parameters;
    std::vector<std::string> param_names = {"a1", "a2", "b", "c1", "c2", "c3", "c4"};

    for (const auto &name: param_names) {
      double value;
      std::string param_key = "opw_kinematics_geometric_parameters." + name;

      // Try cached first, then local, then async
      bool found = false;
      if (cached) {
        found = lookupCachedParam(param_key, value, 0.0);
      }
      if (!found) {
        found = lookupParam(node_, param_key, value, 0.0);
      }

      if (!found || (value == 0.0 && name != "b")) {
        RCLCPP_ERROR(LOGGER, "Failed to load geometric parameter: %s", param_key.c_str());
        return false;
      }

      geometric_parameters[name] = value;
      RCLCPP_INFO(LOGGER, "Loaded %s = %f", name.c_str(), value);
    }

    // Load joint offsets
    std::vector<double> joint_offsets(6);
    bool offsets_found = false;

    // Try cached first
    if (cached) {
      offsets_found = lookupCachedParam("opw_kinematics_joint_offsets", joint_offsets, std::vector<double>{});
      if (offsets_found && joint_offsets.size() == 6) {
        RCLCPP_INFO(LOGGER, "Successfully loaded joint_offsets from cache");
      } else {
        offsets_found = false;
      }
    }

    // Fallback to local/async lookup
    if (!offsets_found) {
      offsets_found = lookupParam(node_, "opw_kinematics_joint_offsets", joint_offsets, std::vector<double>{});
      if (offsets_found && joint_offsets.size() == 6) {
        RCLCPP_INFO(LOGGER, "Successfully loaded joint_offsets");
      } else {
        offsets_found = false;
      }
    }

    if (!offsets_found) {
      RCLCPP_ERROR(LOGGER, "Failed to load joint offsets for ik solver.");
      return false;
    }

    // Load joint sign corrections
    std::vector<int64_t> joint_sign_corrections(6);
    bool corrections_found = false;

    // Try cached first
    if (cached) {
      corrections_found = lookupCachedParam("opw_kinematics_joint_sign_corrections", joint_sign_corrections,
                                            std::vector<int64_t>{});
      if (corrections_found && joint_sign_corrections.size() == 6) {
        RCLCPP_INFO(LOGGER, "Successfully loaded joint_sign_corrections from cache");
      } else {
        corrections_found = false;
      }
    }

    // Fallback to local/async lookup
    if (!corrections_found) {
      corrections_found = lookupParam(node_, "opw_kinematics_joint_sign_corrections", joint_sign_corrections,
                                      std::vector<int64_t>{});
      if (corrections_found && joint_sign_corrections.size() == 6) {
        RCLCPP_INFO(LOGGER, "Successfully loaded joint_sign_corrections");
      } else {
        corrections_found = false;
      }
    }

    if (!corrections_found) {
      RCLCPP_ERROR(LOGGER, "Failed to load joint sign corrections for ik solver.");
      return false;
    }

    // Validate loaded parameters
    if (joint_offsets.size() != 6) {
      RCLCPP_ERROR(LOGGER, "Expected joint_offsets to contain 6 elements, but it has %zu.", joint_offsets.size());
      return false;
    }

    if (joint_sign_corrections.size() != 6) {
      RCLCPP_ERROR(LOGGER, "Expected joint_sign_corrections to contain 6 elements, but it has %zu.",
                   joint_sign_corrections.size());
      return false;
    }

    // Use the existing assignParameters function
    return assignParameters(geometric_parameters, joint_offsets, joint_sign_corrections);
  }

  bool MoveItOPWKinematicsPlugin::assignParameters(
    const std::map<std::string, double> &geom_params,
    const std::vector<double> &joint_offsets,
    const std::vector<int64_t> &joint_sign_corrections) {
    // Validate sizes
    if (joint_offsets.size() != 6) {
      RCLCPP_ERROR(LOGGER, "joint_offsets must have 6 elements, got %zu", joint_offsets.size());
      return false;
    }

    if (joint_sign_corrections.size() != 6) {
      RCLCPP_ERROR(LOGGER, "joint_sign_corrections must have 6 elements, got %zu", joint_sign_corrections.size());
      return false;
    }

    // Assign parameters
    opw_parameters_.a1 = geom_params.at("a1");
    opw_parameters_.a2 = geom_params.at("a2");
    opw_parameters_.b = geom_params.at("b");
    opw_parameters_.c1 = geom_params.at("c1");
    opw_parameters_.c2 = geom_params.at("c2");
    opw_parameters_.c3 = geom_params.at("c3");
    opw_parameters_.c4 = geom_params.at("c4");

    for (std::size_t i = 0; i < 6; ++i) {
      opw_parameters_.offsets[i] = joint_offsets[i];
      opw_parameters_.sign_corrections[i] = static_cast<signed char>(joint_sign_corrections[i]);
    }

    // Log success
    RCLCPP_INFO(LOGGER, "✓ Successfully loaded ALL OPW parameters");
    RCLCPP_INFO(LOGGER, "✓ Geometric: a1=%f, a2=%f, b=%f, c1=%f, c2=%f, c3=%f, c4=%f",
                opw_parameters_.a1, opw_parameters_.a2, opw_parameters_.b,
                opw_parameters_.c1, opw_parameters_.c2, opw_parameters_.c3, opw_parameters_.c4);
    RCLCPP_INFO(LOGGER, "✓ Offsets: [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]",
                opw_parameters_.offsets[0], opw_parameters_.offsets[1], opw_parameters_.offsets[2],
                opw_parameters_.offsets[3], opw_parameters_.offsets[4], opw_parameters_.offsets[5]);
    RCLCPP_INFO(LOGGER, "✓ Sign corrections: [%d, %d, %d, %d, %d, %d]",
                opw_parameters_.sign_corrections[0], opw_parameters_.sign_corrections[1],
                opw_parameters_.sign_corrections[2],
                opw_parameters_.sign_corrections[3], opw_parameters_.sign_corrections[4],
                opw_parameters_.sign_corrections[5]);

    return true;
  }

  // Add these as private member functions in your MoveItOPWKinematicsPlugin class

  // Updated non-blocking lookupParam function
  template<typename T>
  bool MoveItOPWKinematicsPlugin::lookupParam(const rclcpp::Node::SharedPtr &node, const std::string &param, T &val,
                                              const T &default_val) {
    // First, try local parameters immediately (fastest path)
    if (lookupParamLocal(node, param, val, default_val)) {
      return true;
    }

    // If local lookup fails, try async lookup from move_group with timeout
    return lookupParamFromMoveGroupAsync(node, param, val, default_val);
  }

  // Async lookup from move_group with very short timeout
  template<typename T>
  bool MoveItOPWKinematicsPlugin::lookupParamFromMoveGroupAsync(const rclcpp::Node::SharedPtr &node,
                                                                const std::string &param, T &val,
                                                                const T &default_val) {
    // Create a client to get parameters from move_group node
    auto client = node->create_client<rcl_interfaces::srv::GetParameters>("/move_group/get_parameters");

    // Very short wait - don't block the UI thread
    if (!client->wait_for_service(std::chrono::milliseconds(100))) {
      RCLCPP_DEBUG(LOGGER, "move_group parameter service not immediately available for param: %s", param.c_str());
      val = default_val;
      return false;
    }

    // Try different parameter name variations
    std::vector<std::string> param_variations = {
      group_name_ + "." + param,
      param,
      "robot_description_kinematics." + group_name_ + "." + param,
      "robot_description_kinematics." + param
    };

    for (const auto &param_name: param_variations) {
      auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
      request->names = {param_name};

      try {
        // Use async call with very short timeout
        auto future = client->async_send_request(request);

        // Wait for a very short time to avoid blocking
        auto future_status = future.wait_for(std::chrono::milliseconds(200));

        if (future_status == std::future_status::ready) {
          auto response = future.get();

          if (!response->values.empty() &&
              response->values[0].type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
            // Convert parameter value to the requested type
            if (convertParameterValue(response->values[0], val)) {
              RCLCPP_DEBUG(LOGGER, "Found parameter '%s' on move_group node", param_name.c_str());
              return true;
            }
          }
        } else {
          RCLCPP_DEBUG(LOGGER, "Timeout waiting for parameter '%s' from move_group", param_name.c_str());
          // Don't wait for other variations if one times out
          break;
        }
      } catch (const std::exception &e) {
        RCLCPP_DEBUG(LOGGER, "Failed to get parameter '%s' from move_group: %s", param_name.c_str(), e.what());
        // Continue trying other parameter variations
      }
    }

    // Parameter not found, use default
    val = default_val;
    return false;
  }

  // Fallback function for local parameter lookup (unchanged but optimized order)
  template<typename T>
  bool MoveItOPWKinematicsPlugin::lookupParamLocal(const rclcpp::Node::SharedPtr &node, const std::string &param,
                                                   T &val, const T &default_val) {
    // Try parameter variations in order of likelihood
    std::vector<std::string> param_variations = {
      param, // Try the direct parameter name first (most common)
      group_name_ + "." + param,
      "robot_description_kinematics." + group_name_ + "." + param,
      "robot_description_kinematics." + param
    };

    for (const auto &param_name: param_variations) {
      if (node->has_parameter(param_name)) {
        try {
          auto param_result = node->get_parameter(param_name);

          if constexpr (std::is_same_v<T, double>) {
            if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE) {
              val = param_result.as_double();
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s'", param_name.c_str());
              return true;
            } else if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              val = static_cast<double>(param_result.as_int());
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s' (converted from int)", param_name.c_str());
              return true;
            }
          } else if constexpr (std::is_same_v<T, std::vector<double> >) {
            if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY) {
              val = param_result.as_double_array();
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s'", param_name.c_str());
              return true;
            }
          } else if constexpr (std::is_same_v<T, std::vector<int64_t> >) {
            if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY) {
              val = param_result.as_integer_array();
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s'", param_name.c_str());
              return true;
            }
          } else if constexpr (std::is_same_v<T, int64_t>) {
            if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER) {
              val = param_result.as_int();
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s'", param_name.c_str());
              return true;
            }
          } else if constexpr (std::is_same_v<T, std::string>) {
            if (param_result.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
              val = param_result.as_string();
              RCLCPP_DEBUG(LOGGER, "Found local parameter '%s'", param_name.c_str());
              return true;
            }
          }
        } catch (const std::exception &e) {
          RCLCPP_WARN(LOGGER, "Error getting local parameter '%s': %s", param_name.c_str(), e.what());
        }
      }
    }

    val = default_val;
    return false;
  }

  template<typename T>
  bool MoveItOPWKinematicsPlugin::convertParameterValue(const rcl_interfaces::msg::ParameterValue &param_value,
                                                        T &val) {
    if constexpr (std::is_same_v<T, double>) {
      if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE) {
        val = param_value.double_value;
        return true;
      } else if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        val = static_cast<double>(param_value.integer_value);
        return true;
      }
    } else if constexpr (std::is_same_v<T, std::vector<double> >) {
      if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY) {
        val = param_value.double_array_value;
        return true;
      }
    } else if constexpr (std::is_same_v<T, std::vector<int64_t> >) {
      if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY) {
        val = param_value.integer_array_value;
        return true;
      }
    } else if constexpr (std::is_same_v<T, int64_t>) {
      if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
        val = param_value.integer_value;
        return true;
      }
    } else if constexpr (std::is_same_v<T, std::string>) {
      if (param_value.type == rcl_interfaces::msg::ParameterType::PARAMETER_STRING) {
        val = param_value.string_value;
        return true;
      }
    }
    return false;
  }


  // Alternative approach: Cache parameters during initialization
  bool MoveItOPWKinematicsPlugin::cacheParametersFromMoveGroup() {
    auto client = node_->create_client<rcl_interfaces::srv::GetParameters>("/move_group/get_parameters");

    // Don't block - if move_group isn't ready, we'll use local params
    if (!client->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_INFO(LOGGER, "move_group not available for parameter caching, will use local parameters");
      return false;
    }

    // Get all OPW parameters in one call
    std::vector<std::string> all_param_names = {
      "opw_kinematics_geometric_parameters.a1",
      "opw_kinematics_geometric_parameters.a2",
      "opw_kinematics_geometric_parameters.b",
      "opw_kinematics_geometric_parameters.c1",
      "opw_kinematics_geometric_parameters.c2",
      "opw_kinematics_geometric_parameters.c3",
      "opw_kinematics_geometric_parameters.c4",
      "opw_kinematics_joint_offsets",
      "opw_kinematics_joint_sign_corrections",
      "opw_tool_frame",
      // Also try with prefixes
      "robot_description_kinematics." + group_name_ + ".opw_tool_frame",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.a1",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.a2",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.b",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.c1",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.c2",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.c3",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_geometric_parameters.c4",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_joint_offsets",
      "robot_description_kinematics." + group_name_ + ".opw_kinematics_joint_sign_corrections"
    };

    auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
    request->names = all_param_names;

    try {
      auto future = client->async_send_request(request);
      auto future_status = future.wait_for(std::chrono::milliseconds(1000));

      if (future_status == std::future_status::ready) {
        auto response = future.get();

        // Cache the found parameters
        for (size_t i = 0; i < response->values.size() && i < all_param_names.size(); ++i) {
          if (response->values[i].type != rcl_interfaces::msg::ParameterType::PARAMETER_NOT_SET) {
            cached_parameters_[all_param_names[i]] = response->values[i];
            RCLCPP_DEBUG(LOGGER, "Cached parameter: %s", all_param_names[i].c_str());
          }
        }

        RCLCPP_INFO(LOGGER, "Successfully cached %zu parameters from move_group", cached_parameters_.size());
        return true;
      } else {
        // The caller falls back to locally declared parameters.
        RCLCPP_INFO(LOGGER, "Timeout caching parameters from move_group, will use local parameters");
        return false;
      }
    } catch (const std::exception &e) {
      RCLCPP_WARN(LOGGER, "Failed to cache parameters from move_group: %s", e.what());
      return false;
    }
  }

  // Use cached parameters if available
  template<typename T>
  bool MoveItOPWKinematicsPlugin::lookupCachedParam(const std::string &param, T &val, const T &default_val) {
    std::vector<std::string> param_variations = {
      param,
      group_name_ + "." + param,
      "robot_description_kinematics." + group_name_ + "." + param,
      "robot_description_kinematics." + param
    };

    for (const auto &param_name: param_variations) {
      auto it = cached_parameters_.find(param_name);
      if (it != cached_parameters_.end()) {
        if (convertParameterValue(it->second, val)) {
          RCLCPP_DEBUG(LOGGER, "Found cached parameter '%s'", param_name.c_str());
          return true;
        }
      }
    }

    val = default_val;
    return false;
  }

  double MoveItOPWKinematicsPlugin::distance(const std::vector<double> &a, const std::vector<double> &b) {
    double cost = 0.0;
    for (size_t i = 0; i < a.size(); ++i)
      cost += std::abs(b[i] - a[i]);
    return cost;
  }

  // Compute the index of the closest joint pose in 'candidates' from 'target'
  std::size_t MoveItOPWKinematicsPlugin::closestJointPose(const std::vector<double> &target,
                                                          const std::vector<std::vector<double> > &candidates) {
    size_t closest = 0; // index into candidates
    double lowest_cost = std::numeric_limits<double>::max();
    for (size_t i = 0; i < candidates.size(); ++i) {
      assert(target.size() == candidates[i].size());
      double c = distance(target, candidates[i]);
      if (c < lowest_cost) {
        closest = i;
        lowest_cost = c;
      }
    }
    return closest;
  }

  bool MoveItOPWKinematicsPlugin::getAllIK(const Eigen::Isometry3d &pose,
                                           std::vector<std::vector<double> > &joint_poses) const {
    joint_poses.clear();

    // Convert the requested tip pose to the equivalent OPW-frame pose before
    // solving, so every solution places the tip exactly on the request.
    auto base_transform = robot_state_->getGlobalLinkTransform(base_frame_);

    Eigen::Isometry3d tool_pose = base_transform * pose * tip_offset_.inverse();

    auto sols = opw_kinematics::inverse(opw_parameters_, tool_pose);

    // Check the output
    std::vector<double> tmp(6); // temporary storage for API reasons
    for (int i = 0; i < 8; i++) {
      //            double *sol = sols.data() + 6 * i;
      auto sol = sols[i];
      if (opw_kinematics::isValid(sol)) {
        opw_kinematics::harmonizeTowardZero(sol);

        // TODO: make this better...
        std::copy_n(sol.begin(), 6, tmp.data());
        joint_poses.push_back(tmp);
      }
    }

    return joint_poses.size() > 0;
  }

  bool MoveItOPWKinematicsPlugin::getIK(const Eigen::Isometry3d &pose, const std::vector<double> &seed_state,
                                        std::vector<double> &joint_pose) const {
    // Descartes Robot Model interface calls for 'closest' point to seed position
    std::vector<std::vector<double> > joint_poses;
    if (!getAllIK(pose, joint_poses))
      return false;
    // Find the closest joint pose; getAllIK() does isValid checks already
    joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
    return true;
  }
} // namespace moveit_opw_kinematics_plugin
