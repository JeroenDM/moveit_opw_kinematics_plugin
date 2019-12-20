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
class TestPlugin : public ::testing::Test
{
protected:
  void SetUp() override
  {
    ros::NodeHandle nh;
    if (nh.getParam(GROUP_PARAM, group_name_) && nh.getParam(ROOT_LINK_PARAM, root_link_) &&
        nh.getParam(TIP_LINK_PARAM, tip_link_))
    {
      rdf_loader::RDFLoader rdf_loader(ROBOT_DESCRIPTION);
      const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
      const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

      if (!urdf_model || !srdf)
      {
        ROS_ERROR_NAMED("opw", "URDF and SRDF must be loaded for OPW kinematics "
                               "tests to work.");
        return;
      }

      robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

      // the last parameter specifies "search_discretization", which is not used by the opw plugin
      plugin_.initialize(*robot_model_.get(), group_name_, root_link_, { tip_link_ }, 0.1);
    }
    else
    {
      ROS_ERROR_STREAM("Failed to load parameters necessary to load plugin.");
    }
  }
  void TearDown() override
  {
  }

  robot_model::RobotModelPtr robot_model_;
  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin_;
  std::string root_link_;
  std::string tip_link_;
  std::string group_name_;
  std::string robot_description_name_;
};

TEST_F(TestPlugin, InitOk)
{
  ASSERT_EQ(plugin_.getGroupName(), group_name_);
}

TEST_F(TestPlugin, CompareIKAndFK)
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
  Eigen::Isometry3d actual, desired;
  tf::poseMsgToEigen(poses_out[0], desired);
  for (auto js : solutions)
  {
    plugin_.getPositionFK(plugin_.getLinkNames(), js, poses_out);
    tf::poseMsgToEigen(poses_out[0], actual);
    moveit_opw_kinematics_plugin::testing::comparePoses(actual, desired);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_opw_kinematics_test_fanuc");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
