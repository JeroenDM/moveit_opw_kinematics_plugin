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

// OPW kinematics
// OPW kinematics
#include "opw_kinematics/opw_io.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"

// register OPWKinematics as a KinematicsBase implementation
#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_opw_kinematics_plugin
{
static rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_opw_kinematics_plugin");

using kinematics::KinematicsResult;

MoveItOPWKinematicsPlugin::MoveItOPWKinematicsPlugin() : active_(false)
{
}

bool MoveItOPWKinematicsPlugin::initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                                           const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  bool debug = false;

  RCLCPP_INFO_STREAM(LOGGER, "MoveItOPWKinematicsPlugin initializing");

  storeValues(robot_model, group_name, base_frame, tip_frames, search_discretization);

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (debug)
  {
    std::cout << std::endl
              << "Joint Model Variable Names: "
                 "------------------------------------------- "
              << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Get the dimension of the planning group
  dimension_ = joint_model_group_->getVariableCount();
  ROS_INFO_STREAM_NAMED("opw", "Dimension planning group '"
                                   << group_name << "': " << dimension_
                                   << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                                   << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getActiveJointModelNames()[i]);
  }

  if (debug)
  {
    RCLCPP_ERROR(LOGGER, "tip links available:");
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      RCLCPP_ERROR(LOGGER, "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                   group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // set geometric parameters for opw model
  if (!setOPWParameters())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Could not load OPW parameters. Please make "
                                "sure they are loaded on the parameter server and are of the correct type(s).");
    return false;
  }

  // check geometric parameters for opw model
  if (!selfTest())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "The OPW parameters loaded from the parameter "
                                "server appear to be incorrect (self-test failed).");
    return false;
  }

  active_ = true;
  ROS_DEBUG_NAMED("opw", "OPW kinematics solver initialized");
  return true;
}

bool MoveItOPWKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    RCLCPP_ERROR(LOGGER, "This group cannot have redundant joints");
    return false;
  }
  if (redundant_joints.size() > static_cast<std::size_t>(num_possible_redundant_joints_))
  {
    RCLCPP_ERROR(LOGGER "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }

  redundant_joint_indices_ = redundant_joints;

  return true;
}

bool MoveItOPWKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

int MoveItOPWKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); i++)
  {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool MoveItOPWKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool MoveItOPWKinematicsPlugin::selfTest()
{
  const std::vector<double> joint_angles = { 0.1, -0.1, 0.2, -0.3, 0.5, -0.8 };

  auto fk_pose_opw = opw_kinematics::forward(opw_parameters_, &joint_angles[0]);
  robot_state_->setJointGroupPositions(joint_model_group_, joint_angles);
  auto fk_pose_moveit = robot_state_->getGlobalLinkTransform(tip_frames_[0]);
  // group/robot might not be at origin, subtract base transform
  auto base = robot_state_->getGlobalLinkTransform(base_frame_);
  fk_pose_moveit = base.inverse() * fk_pose_moveit;

  if (!comparePoses(fk_pose_opw, fk_pose_moveit))
  {
    return false;
  }

  // reset robot state
  robot_state_->setToDefaultValues();
  return true;
}

bool MoveItOPWKinematicsPlugin::comparePoses(Eigen::Isometry3d& Ta, Eigen::Isometry3d& Tb)
{
  const float TOLERANCE = 1e-6;

  auto Ra = Ta.rotation();
  auto Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      if (std::abs(Ra(i, j) - Rb(i, j)) > TOLERANCE)
      {
        RCLCPP_ERROR(LOGGER, "Pose orientation error on element (%d, %d).", i, j);
        RCLCPP_ERROR(LOGGER, "opw: %f, moveit: %f.", Ra(i, j), Rb(i, j));
        return false;
      }
    }
  }

  auto pa = Ta.translation();
  auto pb = Tb.translation();
  for (int i = 0; i < 3; ++i)
  {
    if (std::abs(pa(i) - pb(i)) > TOLERANCE)
    {
      RCLCPP_ERROR(LOGGER, "Pose position error on element (%d).", i);
      RCLCPP_ERROR(LOGGER, "opw: %f, moveit: %f.", pa(i), pb(i));
      return false;
    }
  }
  return true;
}

bool MoveItOPWKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const std::vector<double>& consistency_limits,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

void MoveItOPWKinematicsPlugin::expandIKSolutions(std::vector<std::vector<double>>& solutions) const
{
  const std::vector<const robot_model::JointModel*>& ajms = joint_model_group_->getActiveJointModels();
  for (size_t i = 0; i < ajms.size(); ++i)
  {
    const robot_model::JointModel* jm = ajms[i];
    if (jm->getVariableBounds().size() > 0)
    {
      for (auto& bounds : jm->getVariableBounds())
      {
        // todo: what to do about continuous joints
        if (!bounds.position_bounded_)
          continue;

        std::vector<std::vector<double>> additional_solutions;
        for (auto& sol : solutions)
        {
          std::vector<double> down_sol(sol);
          while (down_sol[i] - 2.0 * M_PI > bounds.min_position_)
          {
            down_sol[i] -= 2.0 * M_PI;
            additional_solutions.push_back(down_sol);
          }
          std::vector<double> up_sol(sol);
          while (up_sol[i] + 2.0 * M_PI < bounds.max_position_)
          {
            up_sol[i] += 2.0 * M_PI;
            additional_solutions.push_back(up_sol);
          }
        }
        RCLCPP_DEBUG_STREAM(LOGGER, "Found " << additional_solutions.size() << " additional solutions for j=" << i + 1);
        solutions.insert(solutions.end(), additional_solutions.begin(), additional_solutions.end());
      }
    }
  }
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                 const std::vector<double>& ik_seed_state, double /*timeout*/,
                                                 const std::vector<double>& /*consistency_limits*/,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& /*options*/) const
{
  // Check if active
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    RCLCPP_ERROR_STREAM(LOGGER,
                        "Seed state must have size " << dimension_ << " instead of size " << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                       << tip_frames_.size()
                                                                       << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  std::vector<std::vector<double>> solutions;
  if (!getAllIK(pose, solutions))
  {
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

  for (auto& sol : solutions)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, sol);
    // robot_state_->update(); // not required for checking bounds
    if (!robot_state_->satisfiesBounds(joint_model_group_))
    {
      RCLCPP_DEBUG_STREAM(LOGGER, "Solution is outside bounds");
      continue;
    }
    limit_obeying_solutions.push_back({ sol, distance(sol, ik_seed_state) });
  }

  if (limit_obeying_solutions.empty())
  {
    ROS_DEBUG_NAMED("opw", "None of the solutions is within joint limits");
    return false;
  }

  RCLCPP_DEBUG_STREAM(LOGGER, "Solutions within limits: " << limit_obeying_solutions.size());

  // sort solutions by distance to seed state
  std::sort(limit_obeying_solutions.begin(), limit_obeying_solutions.end());

  if (!solution_callback)
  {
    solution = limit_obeying_solutions.front().value;
    return true;
  }

  for (auto& sol : limit_obeying_solutions)
  {
    solution_callback(ik_poses[0], sol.value, error_code);
    if (error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    {
      solution = sol.value;
      RCLCPP_DEBUG_STREAM(LOGGER, "Solution passes callback");
      return true;
    }
  }

  RCLCPP_DEBUG_STREAM(LOGGER, "No solution fullfilled requirements of solution callback");
  return false;
}

bool MoveItOPWKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                              const std::vector<double>& /*ik_seed_state*/,
                                              std::vector<std::vector<double>>& solutions, KinematicsResult& /*result*/,
                                              const kinematics::KinematicsQueryOptions& /*options*/) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "You can only get all solutions for a single pose.");
    return false;
  }
  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  return getAllIK(pose, solutions);
}

bool MoveItOPWKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                              const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::Pose>& poses) const
{
  if (!active_)
  {
    RCLCPP_ERROR(LOGGER, "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    RCLCPP_ERROR(LOGGER, "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != poses.size())
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Mismatched number of pose requests (" << poses.size() << ") to tip frames ("
                                                                       << tip_frames_.size()
                                                                       << ") in searchPositionFK");
    return false;
  }

  // forward function expect pointer to first element of array of joint values
  // that is why &joint_angles[0] is passed
  tf::poseEigenToMsg(opw_kinematics::forward(opw_parameters_, &joint_angles[0]), poses[0]);

  return true;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getJointNames() const
{
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getLinkNames() const
{
  return ik_group_info_.link_names;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getVariableNames() const
{
  return joint_model_group_->getVariableNames();
}

bool MoveItOPWKinematicsPlugin::setOPWParameters()
{
  RCLCPP_INFO_STREAM(LOGGER, "Getting kinematic parameters from parameter server.");

  ros::NodeHandle nh;

  std::map<std::string, double> geometric_parameters;
  if (!lookupParam("opw_kinematics_geometric_parameters", geometric_parameters, {}))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to load geometric parameters for ik solver.");
    return false;
  }

  std::vector<double> joint_offsets;
  if (!lookupParam("opw_kinematics_joint_offsets", joint_offsets, {}))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to load joint offsets for ik solver.");
    return false;
  }

  std::vector<int> joint_sign_corrections;
  if (!lookupParam("opw_kinematics_joint_sign_corrections", joint_sign_corrections, {}))
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Failed to load joint sign corrections for ik solver.");
    return false;
  }

  opw_parameters_.a1 = geometric_parameters["a1"];
  opw_parameters_.a2 = geometric_parameters["a2"];
  opw_parameters_.b = geometric_parameters["b"];
  opw_parameters_.c1 = geometric_parameters["c1"];
  opw_parameters_.c2 = geometric_parameters["c2"];
  opw_parameters_.c3 = geometric_parameters["c3"];
  opw_parameters_.c4 = geometric_parameters["c4"];

  if (joint_offsets.size() != 6)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Expected joint_offsets to contain 6 elements, but it has " << joint_offsets.size() << ".");
    return false;
  }

  if (joint_sign_corrections.size() != 6)
  {
    RCLCPP_ERROR_STREAM(LOGGER, "Expected joint_sign_corrections to contain 6 elements, but it has "
                     << joint_sign_corrections.size() << ".");
    return false;
  }

  for (std::size_t i = 0; i < joint_offsets.size(); ++i)
  {
    opw_parameters_.offsets[i] = joint_offsets[i];
    opw_parameters_.sign_corrections[i] = static_cast<signed char>(joint_sign_corrections[i]);
  }

  RCLCPP_INFO_STREAM(LOGGER,"Loaded parameters for ik solver:\n" << opw_parameters_);

  return true;
}

double MoveItOPWKinematicsPlugin::distance(const std::vector<double>& a, const std::vector<double>& b)
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
std::size_t MoveItOPWKinematicsPlugin::closestJointPose(const std::vector<double>& target,
                                                        const std::vector<std::vector<double>>& candidates)
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

bool MoveItOPWKinematicsPlugin::getAllIK(const Eigen::Isometry3d& pose,
                                         std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  // needed if we introduce a tip frame different from tool0
  // or a different base frame
  // Eigen::Isometry3d tool_pose = diff_base.inverse() * pose *
  // tip_frame.inverse();

  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(opw_parameters_, pose, sols.data());

  // Check the output
  std::vector<double> tmp(6);  // temporary storage for API reasons
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol);

      // TODO: make this better...
      std::copy(sol, sol + 6, tmp.data());
      // if (isValid(tmp))
      // {
      joint_poses.push_back(tmp);
      // }
    }
  }

  return joint_poses.size() > 0;
}

bool MoveItOPWKinematicsPlugin::getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
                                      std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

}  // namespace moveit_opw_kinematics_plugin
