#include "moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h"

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

// OPW kinematics
#include "opw_kinematics/opw_kinematics.h"

namespace moveit_opw_kinematics_plugin
{
bool MoveItOPWKinematicsPlugin::initialize(const std::string &robot_description,
                                           const std::string &group_name,
                                           const std::string &base_frame,
                                           const std::string &tip_frame,
                                           double search_discretization)
{
  setValues(robot_description, group_name, base_frame, tip_frame, search_discretization);
  robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
  robot_model::RobotModelPtr kinematic_model_ = robot_model_loader.getModel();
}

bool MoveItOPWKinematicsPlugin::setOPWParameters(opw_kinematics::Parameters<double> &parameters)
{
  return false;
}
} // namespace moveit_opw_kinematics_plugin