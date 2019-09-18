#include <class_loader/class_loader.hpp>
#include <moveit_opw_kinematics_plugin/moveit_opw_railed_kinematics_plugin.h>

// URDF, SRDF
#include <srdfdom/model.h>
#include <urdf_model/model.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_state/conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// OPW kinematics
#include "opw_kinematics/opw_io.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"

// register OPWKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_opw_kinematics_plugin::MoveItOPWRailedKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;

MoveItOPWRailedKinematicsPlugin::MoveItOPWRailedKinematicsPlugin() : MoveItOPWKinematicsPlugin()
{
}

bool MoveItOPWRailedKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                           const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  bool success = MoveItOPWKinematicsPlugin::initialize(robot_description, group_name, base_frame, tip_frames,
                                                       search_discretization);

  if (!success)
  {
    ROS_ERROR_STREAM("Failed to initialize");
    return false;
  }

  if (!lookupParam<std::string>("free_joint", free_joint_, ""))
  {
    ROS_ERROR_STREAM("Failed to get free_joint parameter");
    return false;
  }

  num_possible_redundant_joints_ = dimension_ - 6;
  if (num_possible_redundant_joints_ != 1)
  {
    ROS_ERROR_STREAM("Requires only 1 free joint (" << num_possible_redundant_joints_ - 6 << " provided)");
    return false;
  }

  setRedundantJoints({ static_cast<unsigned>(getJointIndex(free_joint_)) });

  return success;
}

bool MoveItOPWRailedKinematicsPlugin::getFreeJointLimits(double& min, double& max) const
{
  std::size_t idx;
  try
  {
    auto bounds_vec = joint_model_group_->getActiveJointModelsBounds();
    idx = getJointIndex(free_joint_);
    if (bounds_vec.at(idx)->at(0).position_bounded_)
    {
      max = bounds_vec.at(idx)->at(0).max_position_;
      min = bounds_vec.at(idx)->at(0).min_position_;
    }
    else
    {
      ROS_ERROR_STREAM("Free joint is not position bounded");
      return false;
    }
  }
  catch (const std::exception& ex)
  {
    ROS_ERROR_STREAM(ex.what());
    return false;
  }

  return true;
}

Eigen::Isometry3d MoveItOPWRailedKinematicsPlugin::getRobotBaseTransform(const double rail_position) const
{
  robot_state_->setToDefaultValues();
  std::vector<double> joint_vals;
  robot_state_->copyJointGroupPositions(group_name_, joint_vals);
  joint_vals[getJointIndex(free_joint_)] = rail_position;
  robot_state_->setJointGroupPositions(group_name_, joint_vals);

  // Lookup the transform to the child link of the free joint (i.e. robot base)
  std::string robot_base = joint_model_group_->getJointModel(free_joint_)->getChildLinkModel()->getName();

  return Eigen::Isometry3d(robot_state_->getFrameTransform(robot_base).matrix());
}

geometry_msgs::Pose MoveItOPWRailedKinematicsPlugin::getUpdatedPose(const geometry_msgs::Pose& ik_pose,
                                                                    const double rail_position) const
{
  // Get the transform to the robot base
  Eigen::Isometry3d root_to_robot_base = getRobotBaseTransform(rail_position);

  // Get the transform to the kinematic base link
  Eigen::Isometry3d root_to_kin_base(robot_state_->getFrameTransform(base_frame_).matrix());

  // Convert the target pose
  Eigen::Isometry3d pose;
  tf::poseMsgToEigen(ik_pose, pose);

  // Get the pose relative to the OPW base frame (it comes in relative to the kinematic base frame, i.e. rail base)
  Eigen::Isometry3d new_pose = (root_to_kin_base.inverse() * root_to_robot_base).inverse() * pose;

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
                                                       const kinematics::KinematicsQueryOptions& options,
                                                       const moveit::core::RobotState* context_state) const
{
  double min, max;
  if (!getFreeJointLimits(min, max))
    return false;

  std::vector<LimitObeyingSol> sols;

  std::size_t count = static_cast<std::size_t>(std::floor((max - min) / search_discretization_));
  for (std::size_t i = 0; i < count; ++i)
  {
    double rail_position = min + (i * search_discretization_);

    // Get the IK pose relative to the OPW base frame
    geometry_msgs::Pose new_ik_pose = getUpdatedPose(ik_poses.front(), rail_position);

    // Solve IK
    std::vector<double> sol;
    if (MoveItOPWKinematicsPlugin::searchPositionIK({ new_ik_pose }, ik_seed_state, timeout, consistency_limits, sol, solution_callback,
                                                    error_code, options, context_state))
    {
      sol.insert(sol.begin() + getJointIndex(free_joint_), rail_position);
      sols.push_back({ sol, distance(sol, ik_seed_state) });
    }
  }

  if (sols.empty())
  {
    ROS_WARN_STREAM("No solutions found");
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
    ROS_ERROR_STREAM_NAMED("opw", "You can only get all solutions for a single pose.");
    return false;
  }

  double min, max;
  if (!getFreeJointLimits(min, max))
    return false;

  std::vector<LimitObeyingSol> all_sols;

  std::size_t count = static_cast<std::size_t>(std::floor((max - min) / search_discretization_));
  for (std::size_t i = 0; i < count; ++i)
  {
    double rail_position = min + (i * search_discretization_);
    geometry_msgs::Pose new_ik_pose = getUpdatedPose(ik_poses.front(), rail_position);

    Eigen::Isometry3d new_pose;
    tf::poseMsgToEigen(new_ik_pose, new_pose);

    // Solve IK
    std::vector<std::vector<double>> sols;
    if (MoveItOPWKinematicsPlugin::getAllIK(new_pose, sols))
    {
      for (std::vector<double>& sol : sols)
      {
        sol.insert(sol.begin() + getJointIndex(free_joint_), rail_position);
        all_sols.push_back({ sol, distance(sol, ik_seed_state) });
      }
    }
  }

  if (all_sols.empty())
  {
    ROS_WARN_STREAM("No solutions found");
    return false;
  }

  // Sort the solutions by lowest joint distance from the solution
  std::sort(all_sols.begin(), all_sols.end());

  solutions.clear();
  for (const LimitObeyingSol sol : all_sols)
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
  if (!MoveItOPWKinematicsPlugin::getPositionFK(link_names, joint_angles, robot_poses))
  {
    return false;
  }

  Eigen::Isometry3d robot_base_to_tip;
  tf::poseMsgToEigen(robot_poses[0], robot_base_to_tip);
  Eigen::Isometry3d root_to_robot_base = getRobotBaseTransform(joint_angles[getJointIndex(free_joint_)]);
  Eigen::Isometry3d root_to_tip = root_to_robot_base * robot_base_to_tip;

  poses.resize(link_names.size());
  tf::poseEigenToMsg(root_to_tip, poses.front());

  return true;
}

}  // namespace moveit_opw_kinematics_plugin
