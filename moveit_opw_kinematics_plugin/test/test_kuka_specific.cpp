#include <gtest/gtest.h>
#include <vector>
#include <string>

#include <ros/ros.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

#include "test_utils.h"

class TestKukaSpecific : public testing::Test
{
protected:
  void SetUp() override
  {
    rdf_loader::RDFLoader rdf_loader("robot_description");
    const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
    const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

    robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));
    plugin_.initialize(*robot_model_.get(), "manipulator", "base_link", { "tool0" }, 0.1);
  }
  void TearDown() override
  {
  }

protected:
  robot_model::RobotModelPtr robot_model_;
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

  Eigen::Isometry3d pose_actual, pose_desired;
  tf::poseMsgToEigen(poses[0], pose_actual);

  pose_desired = Translation3d(0.785, 0, 0.435) * AngleAxisd(M_PI_2, Vector3d::UnitY());

  moveit_opw_kinematics_plugin::testing::comparePoses(pose_actual, pose_desired);
}

/** \Brief check forward kinematics for a known position
 *
 * WARNING: Ugly add hoc test
 * This test is meant to catch errors in the specified joint_signed_corrections
 * of joint 1, 4 and 6.
 * We really need more general tests to check for consistency with the URDF.
 */
TEST_F(TestKukaSpecific, positionFKCheckSigns)
{
  using Eigen::AngleAxisd;
  using Eigen::Translation3d;
  using Eigen::Vector3d;

  const double J1 = M_PI_2, J4 = 0.3, J6 = 0.2;

  std::vector<std::string> link_names;
  std::vector<double> joint_angles = { J1, 0, 0, J4, 0, J6 };
  std::vector<geometry_msgs::Pose> poses;

  plugin_.getPositionFK(plugin_.getLinkNames(), joint_angles, poses);

  Eigen::Isometry3d pose_actual, pose_desired;
  tf::poseMsgToEigen(poses[0], pose_actual);

  // rotation for the joint offset of joint 2
  pose_desired = Translation3d(0, 0, 0) * AngleAxisd(M_PI_2, Vector3d::UnitY());

  // rotate around GLOBAL z to add rotation caused by joint 1
  pose_desired = AngleAxisd(-J1, Vector3d::UnitZ()) * pose_desired;

  // rotate two times around LOCAL z-axis for joints J4 and J6
  pose_desired = pose_desired * AngleAxisd(-J4, Vector3d::UnitZ()) * AngleAxisd(-J6, Vector3d::UnitZ());

  // put the frame at the expected position, non-zero y-value
  // instead of x caused by the rotation of joint 1
  pose_desired = Translation3d(0, -0.785, 0.435) * pose_desired;

  moveit_opw_kinematics_plugin::testing::comparePoses(pose_actual, pose_desired);
}
