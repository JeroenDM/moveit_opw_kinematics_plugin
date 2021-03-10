#ifndef MOVEIT_OPW_KINEMATICS_PLUGIN_
#define MOVEIT_OPW_KINEMATICS_PLUGIN_

// ROS
#include <rclcpp/rclcpp.hpp>

// System
#include <memory>

// ROS msgs
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit_msgs/srv/get_position_fk.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <moveit_msgs/msg/kinematic_solver_info.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>

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

  virtual bool initialize(const moveit::core::RobotModel& robot_model, const std::string& group_name,
                          const std::string& base_frame, const std::vector<std::string>& tip_frames,
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
  bool getAllIK(const Eigen::Isometry3d& pose, std::vector<std::vector<double>>& joint_poses) const;
  bool getIK(const Eigen::Isometry3d& pose, const std::vector<double>& seed_state,
             std::vector<double>& joint_pose) const;

  /**
   * @brief append IK solutions by adding +-2pi
   *
   * For all solutions, check if solution +-360° is still inside limits
   * An opw solution might be outside the joint limits, while the extended one is inside (e.g. asymmetric limits)
   * therefore this just extends the solution space, need to apply joint limits separately
   */
  void expandIKSolutions(std::vector<std::vector<double>>& solutions) const;

  /**
   * @brief check forward and inverse kinematics consistency
   *
   * A basic tests to check if the geometric parameters loaded at initialization are correct.
   */
  bool selfTest();

  /**
   * @brief check if two poses are the same within an absolute tolerance of 1e-6
   */
  bool comparePoses(Eigen::Isometry3d& Ta, Eigen::Isometry3d& Tb);

  bool active_; /** Internal variable that indicates whether solvers are configured and ready */

  moveit_msgs::KinematicSolverInfo ik_group_info_; /** Stores information for the inverse kinematics solver */

  unsigned int dimension_; /** Dimension of the group */

  const robot_model::JointModelGroup* joint_model_group_;

  robot_state::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

  opw_kinematics::Parameters<double> opw_parameters_;
};
}  // namespace moveit_opw_kinematics_plugin

#endif
