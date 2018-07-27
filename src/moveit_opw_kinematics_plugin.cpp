#include "moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h"

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// OPW kinematics
#include "opw_kinematics/opw_kinematics.h"

namespace moveit_opw_kinematics_plugin
{
bool MoveItOPWKinematicsPlugin::getPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state,
                                              std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                              const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 std::vector<double> &solution, moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 const std::vector<double> &consistency_limits, std::vector<double> &solution,
                                                 moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(
    const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
    std::vector<double> &solution, const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
    const kinematics::KinematicsQueryOptions &options) const
{
  return false;
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose &ik_pose, const std::vector<double> &ik_seed_state, double timeout,
                                                 const std::vector<double> &consistency_limits, std::vector<double> &solution,
                                                 const IKCallbackFn &solution_callback, moveit_msgs::MoveItErrorCodes &error_code,
                                                 const kinematics::KinematicsQueryOptions &options) const
{
  return false;
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

bool MoveItOPWKinematicsPlugin::initialize(const std::string &robot_description, const std::string &group_name,
                                           const std::string &base_frame, const std::string &tip_frame,
                                           double search_discretization)
{
  return false;
}

const std::vector<std::string> &MoveItOPWKinematicsPlugin::getJointNames() const
{
  return robot_model_->getJointModelNames();
}

const std::vector<std::string> &MoveItOPWKinematicsPlugin::getLinkNames() const
{
  return robot_model_->getLinkModelNames();
}

// bool MoveItOPWKinematicsPlugin::initialize(const std::string &robot_description,
//                                            const std::string &group_name,
//                                            const std::string &base_frame,
//                                            const std::string &tip_frame,
//                                            double search_discretization)
// {
//   setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
//   robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
//   robot_model_ = robot_model_loader.getModel();
//   return true;
// }

MoveItOPWKinematicsPlugin::MoveItOPWKinematicsPlugin(robot_model::RobotModelPtr model)
{
  ROS_INFO_STREAM("Robot name : " << model->getName());
  ROS_INFO_STREAM("Planning group: " << model->getJointModelGroupNames()[0]);

  robot_model_ = model;
  const std::vector<std::string> link_names = getLinkNames();
  setValues("not_used", model->getJointModelGroupNames()[0], link_names[0], link_names[7], 0.1);

  if (!setOPWParameters())
    ROS_ERROR("OPW parameters could not be initialized.");
}

bool MoveItOPWKinematicsPlugin::setOPWParameters()
{
  // these parameters should be extracted from the robot model in a systematic way
  opw_parameters_.a1 = 0.025;
  opw_parameters_.a2 = -0.035;
  opw_parameters_.b = 0.000;
  opw_parameters_.c1 = 0.400;
  opw_parameters_.c2 = 0.315;
  opw_parameters_.c3 = 0.365;
  opw_parameters_.c4 = 0.080;
  opw_parameters_.offsets[1] = -M_PI / 2.0;

  return true;
}

} // namespace moveit_opw_kinematics_plugin