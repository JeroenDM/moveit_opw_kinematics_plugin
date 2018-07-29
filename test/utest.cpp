#include "moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h"

#include <Eigen/Dense>
#include <fstream>
#include <gtest/gtest.h>
#include <map>
#include <string>

#include <ros/ros.h>
#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/MoveItErrorCodes.h>

const double TOLERANCE = 1e-6; // absolute tolerance for EXPECT_NEAR checks

template <typename T>
using Transform = Eigen::Transform<T, 3, Eigen::Affine>;

/** @brief Compare every element of two eigen affine3 poses.
 */
template <typename T>
void comparePoses(const Transform<T> & Ta, const Transform<T> & Tb)
{
  using Matrix = Eigen::Matrix<T, 3, 3>;
  using Vector = Eigen::Matrix<T, 3, 1>;

  Matrix Ra = Ta.rotation(), Rb = Tb.rotation();
  for (int i = 0; i < Ra.rows(); ++i)
  {
    for (int j = 0; j < Ra.cols(); ++j)
    {
      EXPECT_NEAR(Ra(i, j), Rb(i, j), TOLERANCE);
    }
  }

  Vector pa = Ta.translation(), pb = Tb.translation();
  EXPECT_NEAR(pa[0], pb[0], TOLERANCE);
  EXPECT_NEAR(pa[1], pb[1], TOLERANCE);
  EXPECT_NEAR(pa[2], pb[2], TOLERANCE);
}

const std::string urdf_path = "src/moveit_opw_kinematics_plugin/test/kuka_kr6r700sixx.urdf";
const std::string srdf_path = "src/moveit_opw_kinematics_plugin/test/kuka_kr6r700sixx.srdf";

class LoadRobot : public testing::Test
{
protected:
  void SetUp() override
  {

    srdf_model_.reset(new srdf::Model());
    std::string xml_string;
    std::fstream xml_file(urdf_path.c_str(), std::fstream::in);
    if (xml_file.is_open())
    {
      while (xml_file.good())
      {
        std::string line;
        std::getline(xml_file, line);
        xml_string += (line + "\n");
      }
      xml_file.close();
      urdf_model_ = urdf::parseURDF(xml_string);
    }
    srdf_model_->initFile(*urdf_model_, srdf_path);
    robot_model_.reset(new moveit::core::RobotModel(urdf_model_, srdf_model_));
  };

  void TearDown() override
  {
  }

protected:
  urdf::ModelInterfaceSharedPtr urdf_model_;
  srdf::ModelSharedPtr srdf_model_;
  moveit::core::RobotModelPtr robot_model_;
};

TEST_F(LoadRobot, InitOK)
{
  ASSERT_EQ(urdf_model_->getName(), "kuka_kr6r700sixx");
  ASSERT_EQ(srdf_model_->getName(), "kuka_kr6r700sixx");
}

TEST_F(LoadRobot, initialize)
{
  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin;
  plugin.initialize(robot_model_);
}

TEST_F(LoadRobot, positionFK)
{
  using Eigen::AngleAxisd;
  using Eigen::Translation3d;
  using Eigen::Vector3d;

  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin;
  plugin.initialize(robot_model_);

  std::vector<std::string> link_names;
  std::vector<double> joint_angles = {0, 0, 0, 0, 0, 0};
  std::vector<geometry_msgs::Pose> poses;

  plugin.getPositionFK(link_names, joint_angles, poses);

  Eigen::Affine3d pose_actual, pose_desired;
  tf::poseMsgToEigen(poses[0], pose_actual);

  // position and orientation taking into account the offset of the second joint
  // px = a1 + c2 + c3 + c4
  // py = 0
  // pz = c1 + a2
  pose_desired = Translation3d(0.785, 0, 0.435) * AngleAxisd(M_PI_2, Vector3d::UnitY());

  comparePoses(pose_actual, pose_desired);

}

TEST_F(LoadRobot, singleSolutionIK)
{
  moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin;
  plugin.initialize(robot_model_);

  std::vector<std::string> link_names;
  const std::vector<double> joint_angles = {0, 0.1, 0.2, 0.3, 0.4, 0.5};
  std::vector<geometry_msgs::Pose> poses;

  plugin.getPositionFK(link_names, joint_angles, poses);
  const geometry_msgs::Pose pose_in = poses[0];

  // Eigen::Affine3d pose_actual, pose_desired;
  // tf::poseMsgToEigen(poses[0], pose_actual);

  std::vector<double> solution;
  moveit_msgs::MoveItErrorCodes error_code;
  bool res = plugin.getPositionIK(pose_in, joint_angles, solution, error_code);
  EXPECT_TRUE(res);

  for (int i = 0; i < solution.size(); ++i)
  {
    EXPECT_NEAR(solution[i], joint_angles[i], TOLERANCE);
  }
}

// TEST_F(LoadRobot, allSolutionsIK)
// {
//   moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin plugin(robot_model_);

//   std::vector<std::string> link_names;
//   const std::vector<double> joint_angles = {0, 0.1, 0.2, 0.3, 0.4, 0.5};
//   const std::vector<geometry_msgs::Pose> poses;

//   plugin.getPositionFK(link_names, joint_angles, poses);
//   const geometry_msgs::Pose pose_in = poses[0];

//   // Eigen::Affine3d pose_actual, pose_desired;
//   // tf::poseMsgToEigen(poses[0], pose_actual);

//   std::vector<std::vector<double>> solutions;
//   kinematics::KinematicsResult result;
//   bool res = plugin.getPositionIK(poses, joint_angles, solutions, result);
//   EXPECT_TRUE(res);

//   for (int i = 0; i < solutions[0].size(); ++i)
//   {
//     EXPECT_NEAR(solutions[0][i], joint_angles[i], TOLERANCE);
//   }
// }

int main(int argc, char **argv)
{
    ros::init(argc, argv, "utest");
    testing::InitGoogleTest(&argc, argv);
    bool res = RUN_ALL_TESTS();
    ros::shutdown();
    return res;
}


