#include <ros/ros.h>
#include <map>
#include <vector>
#include <string>
#include <iostream>

#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_load_parameters");

  //ros::NodeHandle nh;
  //std::string des;
  //nh.getParam("robot_description", des);
  //std::cout << des << std::endl;

  using Plugin = moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin;

  Plugin plugin;
  plugin.initialize("robot_description", "manipulator", "base", "tip", 0.0);

  std::vector<std::string> res = plugin.getLinkNames();
  std::cout << res[0] << std::endl;
  
  ros::shutdown();

  return 0;
}