#ifndef MOVEIT_OPW_KINEMATICS_PLUGIN_H
#define MOVEIT_OPW_KINEMATICS_PLUGIN_H

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// OPW kinematics
#include "opw_kinematics/opw_kinematics.h"

namespace moveit_opw_kinematics_plugin
{
class MoveItOPWKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  /**
       * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
       * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
       * For example, the name of the ROS parameter that contains the robot description;
       * @param group_name The group for which this solver is being configured
       * @param base_frame The base frame in which all input poses are expected.
       * This may (or may not) be the root frame of the chain that the solver operates on
       * @param tip_frame The tip of the chain
       * @param search_discretization The discretization of the search when the solver steps through the redundancy
       * @return True if initialization was successful, false otherwise
       */
  virtual bool initialize(const std::string &robot_description,
                          const std::string &group_name,
                          const std::string &base_frame,
                          const std::string &tip_frame,
                          double search_discretization);

private:
  /**
       * @brief get the opw parameters out of the robot model.
       * The specific dimensions needed to solve the inverse kinematics are extracted from the robot model.
       * @param parameters Output parameters.
       * @return True if succeeded in finding the right dimensions in the robot model.
       */
  bool setOPWParameters(opw_kinematics::Parameters<double> &parameters);
};
} // namespace moveit_opw_kinematics_plugin

#endif /* MOVEIT_OPW_KINEMATICS_PLUGIN_H */