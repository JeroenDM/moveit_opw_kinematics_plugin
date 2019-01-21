#include <gtest/gtest.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

#include "test_utils.h"

// Names to read values from the parameter server
const std::string GROUP_PARAM = "group";
const std::string TIP_LINK_PARAM = "tip_link";
const std::string ROOT_LINK_PARAM = "root_link";

// Robot description almost always called "robot_description" and therefore hardcoded below
const std::string ROBOT_DESCRIPTION = "robot_description";

/** \Brief Test ik and fk only using the plugin
 *
 * As the ik and fk are both calculated using the opw_kinematics packages,
 * this tests only check if passing values from the plugin to opw_kinematics works,
 * and if the ik and fk calculations inside opw_kinematics are consistent for this robot.
 */
class TestPluginFanuc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle pnh("~");
    if (pnh.getParam(GROUP_PARAM, group_name_) && pnh.getParam(ROOT_LINK_PARAM, root_link_) &&
        pnh.getParam(TIP_LINK_PARAM, tip_link_))
    {
      // the last parameter specifies "search_discretization", which is not used by the opw plugin
      plugin_.initialize(ROBOT_DESCRIPTION, group_name_, root_link_, tip_link_, 0.1);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load parameters necessary to load plugin.");
    }
  }
  void TearDown() override
  {
  }

  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::string robot_description_name_;
};

TEST_F(TestPluginFanuc, InitOk)
{
  ASSERT_EQ(plugin_.getGroupName(), group_name_);
}

TEST_F(TestPluginFanuc, CompareIKAndFK)
{
  std::vector<std::string> link_names;
  const std::vector<double> joint_angles = { 0, 0.1, 0.2, 0.3, 0.4, 0.5 };
  std::vector<geometry_msgs::Pose> poses_out;

  // find reachable pose
  plugin_.getPositionFK(plugin_.getLinkNames(), joint_angles, poses_out);

  // calculate all ik solutions for this pose
  const std::vector<geometry_msgs::Pose> poses_in = { poses_out[0] };
  std::vector<std::vector<double> > solutions;
  kinematics::KinematicsResult result;
  bool res = plugin_.getPositionIK(poses_in, joint_angles, solutions, result);
  EXPECT_TRUE(res);

  // check if fk for all this solutions gives the same pose
  Eigen::Affine3d actual, desired;
  tf::poseMsgToEigen(poses_out[0], desired);
  for (auto js : solutions)
  {
    plugin_.getPositionFK(plugin_.getLinkNames(), js, poses_out);
    tf::poseMsgToEigen(poses_out[0], actual);
    moveit_opw_kinematics_plugin::testing::comparePoses(actual, desired);
  }
}

/** \Brief Check ik of opw_kinematics against fk of MoveIt! kinematics.
 *
 *  In this test fixure the aim is to calculate inverse kinematics using opw_kinematics
 *  inside the plugin and check this using robot_state.getGlobalLinkTransform("tip_link").
 */
class TestKinematicsFanuc : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle pnh("~");
    if (pnh.getParam(GROUP_PARAM, group_name_) && pnh.getParam(ROOT_LINK_PARAM, root_link_) &&
        pnh.getParam(TIP_LINK_PARAM, tip_link_))
    {
      robot_model_loader::RobotModelLoader robot_model_loader(ROBOT_DESCRIPTION);
      robot_model_ = robot_model_loader.getModel();
      robot_state_.reset(new robot_state::RobotState(robot_model_));
      robot_state_->setToDefaultValues();
      joint_model_group_ = robot_model_->getJointModelGroup(group_name_);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load parameters necessary to load plugin.");
    }
  }
  void TearDown() override
  {
  }

  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::string robot_description_name_;

  robot_model::RobotModelPtr robot_model_;
  robot_state::RobotStatePtr robot_state_;
  robot_model::JointModelGroup* joint_model_group_;
};

TEST_F(TestKinematicsFanuc, InitOk)
{
  ASSERT_EQ(robot_model_->getJointModelGroupNames()[0], group_name_);
}

/** \Brief This checks ik against fk for a single joint pose
 * */
TEST_F(TestKinematicsFanuc, TestAllIkSingleJointPose)
{
  const std::vector<double> joint_angles = { 0, 0.1, 0.2, 0.3, 0.4, 0.5 };
  robot_state_->setJointGroupPositions(joint_model_group_, joint_angles);

  // find reachable pose
  auto fk_pose = robot_state_->getGlobalLinkTransform(tip_link_);

  // type conversions
  geometry_msgs::Pose fk_pose_msgs;
  tf::poseEigenToMsg(fk_pose, fk_pose_msgs);
  const std::vector<geometry_msgs::Pose> fk_poses = { fk_pose_msgs };

  // calculate all ik solutions for the pose in fk_poses
  std::vector<std::vector<double> > solutions;
  kinematics::KinematicsResult result;
  kinematics::KinematicsQueryOptions options;
  const auto solver = joint_model_group_->getSolverInstance();

  bool success = solver->getPositionIK(fk_poses, joint_angles, solutions, result, options);
  ASSERT_TRUE(success);

  std::size_t num_solutions = solutions.size();
  ASSERT_GT(num_solutions, 0);

  // check if fk for all this solutions gives the same pose
  Eigen::Affine3d actual_pose;
  for (auto js : solutions)
  {
    robot_state_->setJointGroupPositions(joint_model_group_, js);
    actual_pose = robot_state_->getGlobalLinkTransform(tip_link_);
    moveit_opw_kinematics_plugin::testing::comparePoses(actual_pose, fk_pose);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_opw_kinematics_test_fanuc");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}