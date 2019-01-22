#include <gtest/gtest.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

#include "test_utils.h"

class TestKukaSpecific : public testing::Test
{
protected:
  void SetUp() override
  {
    plugin_.initialize("robot_description", "manipulator", "base_link", "tool0", 0.1);
  };
  void TearDown() override{};

protected:
  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin_;
};

/** \Brief check forward kinematics for robot home position
 * 
 * Calculate by hand position and oriention of tool0 when all joint angles are zero
 * px = a1 + c2 + c3 + c4
 * py = 0
 * pz = c1 + a2
 */
TEST_F(TestKukaSpecific, positionFKAllZero)
{
  using Eigen::AngleAxisd;
  using Eigen::Translation3d;
  using Eigen::Vector3d;

  std::vector<std::string> link_names;
  std::vector<double> joint_angles = { 0, 0, 0, 0, 0, 0 };
  std::vector<geometry_msgs::Pose> poses;

  plugin_.getPositionFK(plugin_.getLinkNames(), joint_angles, poses);

  Eigen::Affine3d pose_actual, pose_desired;
  tf::poseMsgToEigen(poses[0], pose_actual);

  
  pose_desired = Translation3d(0.785, 0, 0.435) * AngleAxisd(M_PI_2, Vector3d::UnitY());

  moveit_opw_kinematics_plugin::testing::comparePoses(pose_actual, pose_desired);
}