#include "moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h"

#include <fstream>
#include <gtest/gtest.h>
#include <string>

#include <moveit/robot_model/robot_model.h>
#include <urdf_parser/urdf_parser.h>

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


int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}


