#include "moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h"

#include <eigen_conversions/eigen_msg.h>
#include <class_loader/class_loader.hpp>
#include <map>
#include <ros/ros.h>
#include <vector>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

// OPW kinematics
#include "opw_kinematics/opw_io.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"


namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;
bool MoveItOPWKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose,
                                              const std::vector<double> &ik_seed_state,
                                              std::vector<double> &solution,
                                              moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_pose, pose);
  return getIK(pose, ik_seed_state, solution);
}

// NOte, I don't know if this is the functions I should use for returning all the solutions
bool MoveItOPWKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                                              const std::vector<double> &ik_seed_state,
                                              std::vector<std::vector<double>> &solutions,
                                              KinematicsResult &result,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM("Only a single ik pose supported at the moment.");
    return false;
  }
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  return getAllIK(pose, solutions);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 const std::vector<double> &consistency_limits, std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 const std::vector<double> &consistency_limits, std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return getPositionIK(ik_pose, ik_seed_state, solution, error_code);
}

bool MoveItOPWKinematicsPlugin::getPositionFK(const std::vector<std::string> &link_names, const std::vector<double> &joint_angles,
                                              std::vector<geometry_msgs::Pose> &poses) const
{
  if (joint_angles.size() == 6)
  {
    poses.resize(1);
    tf::poseEigenToMsg(opw_kinematics::forward(opw_parameters_, &joint_angles[0]), poses[0]);
    return true;
  }
  else
  {
    ROS_DEBUG("The input joint_angles for getPositionFk have the wrong size.");
    return false;
  }
}

bool MoveItOPWKinematicsPlugin::initialize(const std::string &robot_description,
                                           const std::string &group_name,
                                           const std::string &base_frame,
                                           const std::string &tip_frame,
                                           double search_discretization)
{
  //setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
  group_name_ = group_name;
  setOPWParameters();
  // robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
  // robot_model_ = robot_model_loader.getModel();
  return true;
}

const std::vector<std::string> &MoveItOPWKinematicsPlugin::getJointNames() const
{
  return robot_model_->getJointModelNames();
}

const std::vector<std::string> &MoveItOPWKinematicsPlugin::getLinkNames() const
{
  return robot_model_->getLinkModelNames();
}

MoveItOPWKinematicsPlugin::MoveItOPWKinematicsPlugin(robot_model::RobotModelPtr model)
{
  ROS_INFO_STREAM("Robot name : " << model->getName());
  ROS_INFO_STREAM("Planning group: " << model->getJointModelGroupNames()[0]);

  robot_model_ = model;
  const std::vector<std::string> link_names = getLinkNames();
  //setValues("not_used", model->getJointModelGroupNames()[0], link_names[0], link_names[7], 0.1);
  group_name_ = model->getJointModelGroupNames()[0];

  if (!setOPWParameters())
    ROS_ERROR("OPW parameters could not be initialized.");
}

bool MoveItOPWKinematicsPlugin::setOPWParameters()
{
  ROS_INFO_STREAM("Getting kinematic parameters from parameter server.");

  // Using the full parameter name at the moment because the leading slash
  // in front of robot_description_kinematics is missing
  // in lookupParam function, but I'm not sure if this is a bug or I do not understand the interface.
  std::string prefix = "/robot_description_kinematics/" + group_name_ + "/";
  ros::NodeHandle nh;

  std::map<std::string, double> geometric_parameters, dummy;
  if (!lookupParam(prefix + "kinematics_solver_geometric_parameters", geometric_parameters, dummy))
  {
    ROS_ERROR_STREAM("Failed to load geometric parameters for ik solver.");
    return false;
  }

  std::vector<double> joint_offsets, dummy2;
  if (!lookupParam(prefix + "kinematics_solver_joint_offsets", joint_offsets, dummy2))
  {
    ROS_ERROR_STREAM("Failed to load joint offsets for ik solver.");
    return false;
  }

  opw_parameters_.a1 = geometric_parameters["a1"];
  opw_parameters_.a2 = geometric_parameters["a2"];
  opw_parameters_.b = geometric_parameters["b"];
  opw_parameters_.c1 = geometric_parameters["c1"];
  opw_parameters_.c2 = geometric_parameters["c2"];
  opw_parameters_.c3 = geometric_parameters["c3"];
  opw_parameters_.c4 = geometric_parameters["c4"];
  for (std::size_t i = 0; i < joint_offsets.size(); ++i)
  {
    opw_parameters_.offsets[i] = joint_offsets[i];
  }

  ROS_INFO_STREAM("Loaded parameters for ik solver:\ngeometric: " << opw_parameters_);

  return true;
}

bool MoveItOPWKinematicsPlugin::getAllIK(const Eigen::Affine3d &pose,
                                         std::vector<std::vector<double>> &joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  Eigen::Affine3d tool_pose = pose;

  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(opw_parameters_, tool_pose, sols.data());

  // Check the output
  std::vector<double> tmp(6); // temporary storage for API reasons
  for (int i = 0; i < 8; i++)
  {
    double *sol = sols.data() + 6 * i;
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

bool MoveItOPWKinematicsPlugin::getIK(const Eigen::Affine3d &pose,
                                      const std::vector<double> &seed_state,
                                      std::vector<double> &joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

double MoveItOPWKinematicsPlugin::distance(const std::vector<double> &a, const std::vector<double> &b) const
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
std::size_t MoveItOPWKinematicsPlugin::closestJointPose( const std::vector<double> &target, const std::vector<std::vector<double>> &candidates) const
{
  size_t closest = 0; // index into candidates
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

} // namespace moveit_opw_kinematics_plugin

// register MoveItOPWKinematicsPlugin as a KinematicsBase implementation
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin, kinematics::KinematicsBase);