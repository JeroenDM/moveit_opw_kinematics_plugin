#include <gtest/gtest.h>
#include <ros/ros.h>

#include <eigen_conversions/eigen_msg.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

// Names to read values from the parameter server
const std::string GROUP_PARAM = "group";
const std::string TIP_LINK_PARAM = "tip_link";
const std::string ROOT_LINK_PARAM = "root_link";

// Robot description almost always called "robot_description" and therefore hardcoded below
const std::string ROBOT_DESCRIPTION = "robot_description";

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
        ROS_ERROR_NAMED("opw", "URDF and SRDF must be loaded for SRV kinematics "
                               "solver to work.");  // TODO: is this true?
        return;
      }

      robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));
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

/** \Brief Initialize plugin when the kinematic parameters loaded are incorrect.
 *
 * Therefore initialize should return false. Notice we do not check explicitly
 * why it fails, but we assumed that it fails because the faulty parameters
 * where loaded by the rostest launch file.
 */
TEST_F(TestPlugin, InitFaulty)
{
  // the last parameter specifies "search_discretization", which is not used by the opw plugin
  bool res = plugin_.initialize(*robot_model_.get(), group_name_, root_link_, { tip_link_ }, 0.1);
  EXPECT_FALSE(res);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "moveit_opw_kinematics_test_fanuc");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
