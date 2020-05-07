#include <moveit_opw_kinematics_plugin/moveit_opw_railed_kinematics_plugin.h>

// URDF, SRDF
#include <srdfdom/model.h>
#include <urdf_model/model.h>
#include <moveit/rdf_loader/rdf_loader.h>

// Eigen
#include <eigen_conversions/eigen_msg.h>

static const std::string LOG_NAMESPACE = "opw_railed";

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;

MoveItOPWRailedKinematicsPlugin::MoveItOPWRailedKinematicsPlugin() : kinematics::KinematicsBase ()
{
}

bool MoveItOPWRailedKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                                 const std::string& base_frame,
                                                 const std::vector<std::string>& tip_frames,
                                                 double search_discretization)
{
  ROS_INFO_STREAM_NAMED(LOG_NAMESPACE, "MoveItOPWRailedKinematicsPlugin initializing");

  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  // Initialize MoveIt objects
  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED(LOG_NAMESPACE, "URDF and SRDF must be loaded for OPWMoveItRailedKinematicsPlugin kinematics "
                                   "solver to work.");
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));
  robot_state_.reset(new robot_state::RobotState(robot_model_));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getActiveJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getActiveJointModelNames()[i]);
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("opw", "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                      group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Get the OPW planning group (planning group for only the OPW joints, unlike the input group which includes a rail joint)
  const std::string opw_group_name_param = "opw_group";
  if (!lookupParam<std::string>(opw_group_name_param, opw_group_name_, ""))
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to get " << opw_group_name_param << "' parameter");
    return false;
  }
  auto opw_joint_model_group = robot_model_->getJointModelGroup(opw_group_name_);
  if (!opw_joint_model_group)
    return false;

  opw_base_frame_ = opw_joint_model_group->getActiveJointModels().front()->getParentLinkModel()->getName();

  // Get the difference between the input planning group and the specified OPW group, which represents the free joint(s)
  std::vector<std::string> active_joint_diff;
  std::set_difference(joint_model_group_->getActiveJointModelNames().begin(),
                      joint_model_group_->getActiveJointModelNames().end(),
                      opw_joint_model_group->getActiveJointModelNames().begin(),
                      opw_joint_model_group->getActiveJointModelNames().end(),
                      std::inserter(active_joint_diff, active_joint_diff.begin()));

  if (active_joint_diff.size() != 1)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "The MoveItOPWRailedKinematics plugin requires only 1 free joint (" << active_joint_diff.size() << " detected)");
    return false;
  }

  // Initialize the OPW plugin with the group that represents the OPW manipulator (the input joint group should have additional joints)
  if(!opw_plugin_.initialize(robot_description, opw_group_name_, opw_base_frame_, tip_frames, search_discretization))
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Failed to initialize MoveItOPWKinematicsSolver for railed solver");
    return false;
  }

  return true;
}

bool MoveItOPWRailedKinematicsPlugin::getFreeJointLimits(double& min, double& max) const
{
  try
  {
    auto bounds_vec = joint_model_group_->getActiveJointModelsBounds();
    if (bounds_vec.at(0)->at(0).position_bounded_)
    {
      max = bounds_vec.at(0)->at(0).max_position_;
      min = bounds_vec.at(0)->at(0).min_position_;
    }
    else
    {
      ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "Free joint is not position bounded");
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM_NAMED("opw_", ex.what());
    return false;
  }

  return true;
}

Eigen::Isometry3d MoveItOPWRailedKinematicsPlugin::getOPWBaseTransform(const double rail_position) const
{
  robot_state_->setToDefaultValues();
  std::vector<double> joint_vals;
  robot_state_->copyJointGroupPositions(group_name_, joint_vals);
  joint_vals[0] = rail_position;
  robot_state_->setJointGroupPositions(group_name_, joint_vals);

  // Lookup the transform to the OPW base link
  return Eigen::Isometry3d(robot_state_->getFrameTransform(opw_base_frame_).matrix());
}

geometry_msgs::Pose MoveItOPWRailedKinematicsPlugin::getUpdatedPose(const geometry_msgs::Pose& ik_pose,
                                                                    const double rail_position) const
{
  // Get the transform to the robot base
  Eigen::Isometry3d root_to_opw_base = getOPWBaseTransform(rail_position);

  // Get the transform to the kinematic base link
  Eigen::Isometry3d root_to_kin_base(robot_state_->getFrameTransform(base_frame_).matrix());

  // Convert the target pose
  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(ik_pose, pose);

  // Get the pose relative to the OPW base frame (it comes in relative to the kinematic base frame, i.e. rail base)
  Eigen::Isometry3d new_pose = (root_to_kin_base.inverse() * root_to_opw_base).inverse() * pose;

  geometry_msgs::Pose new_ik_pose;
  tf::poseEigenToMsg(new_pose, new_ik_pose);

  return new_ik_pose;
}

bool MoveItOPWRailedKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
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

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
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

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
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

bool MoveItOPWRailedKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                       const std::vector<double>& ik_seed_state, double timeout,
                                                       const std::vector<double>& consistency_limits,
                                                       std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                       moveit_msgs::MoveItErrorCodes& error_code,
                                                       const kinematics::KinematicsQueryOptions& options) const
{
  double min, max;
  if (!getFreeJointLimits(min, max))
    return false;

  std::size_t count = static_cast<std::size_t>(std::floor((max - min) / search_discretization_));
  std::vector<MoveItOPWKinematicsPlugin::LimitObeyingSol> sols;
  sols.reserve(count);

  std::vector<double> opw_seed_state(ik_seed_state.begin() + 1, ik_seed_state.end());
  for (std::size_t i = 0; i < count; ++i)
  {
    double rail_position = min + (i * search_discretization_);

    // Get the IK pose relative to the OPW base frame
    geometry_msgs::Pose new_ik_pose = getUpdatedPose(ik_poses.front(), rail_position);

    // Solve IK using the OPW plugin
    // Pass an "empty" callback as the solution callback for the OPW plugin; the solution_callback should be checked
    // with the full joint state, not just the OPW joints
    MoveItOPWKinematicsPlugin::LimitObeyingSol sol;
    if (opw_plugin_.searchPositionIK({ new_ik_pose }, opw_seed_state, timeout, consistency_limits, sol.value, 0,
                                     error_code, options))
    {
      sol.value.insert(sol.value.begin(), rail_position);

      // Check the full solution using the provided callback
      if (solution_callback)
      {
        moveit_msgs::MoveItErrorCodes tmp_error;
        solution_callback(ik_poses.front(), sol.value, tmp_error);
        if (tmp_error.val != moveit_msgs::MoveItErrorCodes::SUCCESS)
          continue;
      }

      // Order the solutions according to their distance from the seed state
      sol.dist_from_seed = MoveItOPWKinematicsPlugin::distance(sol.value, ik_seed_state);
      sols.push_back(sol);
    }
  }

  if (sols.empty())
  {
    ROS_WARN_STREAM_NAMED(LOG_NAMESPACE, "No IK solutions found");
    error_code.val = moveit_msgs::MoveItErrorCodes::NO_IK_SOLUTION;
    return false;
  }

  // Sort the solutions by lowest joint distance from the solution
  std::sort(sols.begin(), sols.end());
  solution = sols.front().value;

  return true;
}

bool MoveItOPWRailedKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                    const std::vector<double>& ik_seed_state,
                                                    std::vector<std::vector<double>>& solutions, KinematicsResult&,
                                                    const kinematics::KinematicsQueryOptions&) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED(LOG_NAMESPACE, "You can only get all solutions for a single pose.");
    return false;
  }

  double min, max;
  if (!getFreeJointLimits(min, max))
    return false;

  std::vector<MoveItOPWKinematicsPlugin::LimitObeyingSol> all_sols;

  std::size_t count = static_cast<std::size_t>(std::floor((max - min) / search_discretization_));
  for (std::size_t i = 0; i < count; ++i)
  {
    double rail_position = min + (i * search_discretization_);
    geometry_msgs::Pose new_ik_pose = getUpdatedPose(ik_poses.front(), rail_position);

    Eigen::Isometry3d new_pose;
    tf::poseMsgToEigen(new_ik_pose, new_pose);

    // Solve IK
    std::vector<std::vector<double>> sols;
    if (opw_plugin_.getAllIK(new_pose, sols))
    {
      for (std::vector<double>& sol : sols)
      {
        sol.insert(sol.begin(), rail_position);
        all_sols.push_back({ sol, MoveItOPWKinematicsPlugin::distance(sol, ik_seed_state) });
      }
    }
  }

  if (all_sols.empty())
  {
    ROS_WARN_STREAM_NAMED(LOG_NAMESPACE, "No solutions found");
    return false;
  }

  // Sort the solutions by lowest joint distance from the solution
  std::sort(all_sols.begin(), all_sols.end());

  solutions.clear();
  for (const MoveItOPWKinematicsPlugin::LimitObeyingSol sol : all_sols)
  {
    solutions.push_back(sol.value);
  }

  return true;
}

bool MoveItOPWRailedKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                                    const std::vector<double>& joint_angles,
                                                    std::vector<geometry_msgs::Pose>& poses) const
{
  std::vector<geometry_msgs::Pose> robot_poses;
  if (!opw_plugin_.getPositionFK(link_names, joint_angles, robot_poses))
  {
    return false;
  }

  Eigen::Isometry3d robot_base_to_tip;
  tf::poseMsgToEigen(robot_poses[0], robot_base_to_tip);
  Eigen::Isometry3d root_to_opw_base = getOPWBaseTransform(joint_angles[0]);
  Eigen::Isometry3d root_to_tip = root_to_opw_base * robot_base_to_tip;

  poses.resize(link_names.size());
  tf::poseEigenToMsg(root_to_tip, poses.front());

  return true;
}

}  // namespace moveit_opw_kinematics_plugin

// register as a KinematicsBase implementation
#include <class_loader/register_macro.hpp>
CLASS_LOADER_REGISTER_CLASS(moveit_opw_kinematics_plugin::MoveItOPWRailedKinematicsPlugin, kinematics::KinematicsBase)
