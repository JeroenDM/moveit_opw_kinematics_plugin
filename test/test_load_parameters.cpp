#include <ros/ros.h>
#include <map>

#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_load_parameters");

  using Plugin = moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin;

  Plugin plugin;
  plugin.initialize("robot_description", "manipulator", "base", "tip", 0.0);
  
  ros::shutdown();

  return 0;
}