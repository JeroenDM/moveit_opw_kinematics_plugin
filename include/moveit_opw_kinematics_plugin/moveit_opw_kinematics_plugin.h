#ifndef MOVEIT_OPW_KINEMATICS_PLUGIN_
#define MOVEIT_OPW_KINEMATICS_PLUGIN_

// ROS
#include <ros/ros.h>

// System
#include <memory>

// ROS msgs
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/KinematicSolverInfo.h>
#include <moveit_msgs/MoveItErrorCodes.h>

// MoveIt!
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// OPW kinematics
#include "opw_kinematics/opw_kinematics.h"

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;
/**
 * @brief Specific implementation of kinematics using ROS service calls to communicate with
   external IK solvers. This version can be used with any robot. Supports non-chain kinematic groups
 */
class MoveItOPWKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  // struct for storing and sorting solutions
  struct LimitObeyingSol
  {
    std::vector<double> value;
    double dist_from_seed;

    bool operator<(const LimitObeyingSol& a) const
    {
      return dist_from_seed < a.dist_from_seed;
    }
  };

  /**
   *  @brief Default constructor
   */
  MoveItOPWKinematicsPlugin();

  virtual bool
  getPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state,
                std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool searchPositionIK(
      const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
      std::vector<double>& solution, const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
      const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool getPositionFK(const std::vector<std::string>& link_names, const std::vector<double>& joint_angles,
                             std::vector<geometry_msgs::Pose>& poses) const override;

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_name, const std::string& tip_frame, double search_discretization) override
  {
    std::vector<std::string> tip_frames;
    tip_frames.push_back(tip_frame);
    return initialize(robot_description, group_name, base_name, tip_frames, search_discretization);
  }

  virtual bool initialize(const std::string& robot_description, const std::string& group_name,
                          const std::string& base_name, const std::vector<std::string>& tip_frames,
                          double search_discretization) override;

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const std::vector<std::string>& getJointNames() const override;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const std::vector<std::string>& getLinkNames() const override;

  /**
   * @brief  Return all the variable names in the order they are represented internally
   */
  const std::vector<std::string>& getVariableNames() const;

  virtual bool
  getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                std::vector<std::vector<double>>& solutions, KinematicsResult& result,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

protected:
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback,
                   moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                   double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const;

  virtual bool setRedundantJoints(const std::vector<unsigned int>& redundant_joint_indices) override;

private:
  bool timedOut(const ros::WallTime& start_time, double duration) const;

  int getJointIndex(const std::string& name) const;

  bool isRedundantJoint(unsigned int index) const;

  bool setOPWParameters();

  static double distance(const std::vector<double>& a, const std::vector<double>& b);
  static std::size_t closestJointPose(const std::vector<double>& target,
                               const std::vector<std::vector<double>>& candidates);
  bool getAllIK(const Eigen::Affine3d& pose, std::vector<std::vector<double>>& joint_poses) const;
  bool getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state, std::vector<double>& joint_pose) const;

  /**
   * @brief check forward and inverse kinematics consistency
   *
   * A basic tests to check if the geometric parameters loaded at initialization are correct.
   */
  bool selfTest();

  /**
   * @brief check if two poses are the same within an absolute tolerance of 1e-6
   */
  bool comparePoses(Eigen::Isometry3d& Ta, Eigen::Affine3d& Tb);

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  unsigned int dimension_; /** Dimension of the group */

  robot_model::RobotModelPtr robot_model_;
  robot_model::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

  opw_kinematics::Parameters<double> opw_parameters_;
};
}  // namespace moveit_opw_kinematics_plugin

#endif
