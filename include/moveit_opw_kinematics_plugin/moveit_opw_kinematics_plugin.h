#ifndef MOVEIT_OPW_KINEMATICS_PLUGIN_H
#define MOVEIT_OPW_KINEMATICS_PLUGIN_H

// ROS
#include <ros/ros.h>

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
 * @brief Specific implementation of kinematics using the ros package opw_kinematics for industrial robots.
 */
class MoveItOPWKinematicsPlugin : public kinematics::KinematicsBase
{
public:
  static const double DEFAULT_SEARCH_DISCRETIZATION; /* = 0.1 */
  static const double DEFAULT_TIMEOUT;               /* = 1.0 */

  MoveItOPWKinematicsPlugin() = default;
  //MoveItOPWKinematicsPlugin(robot_model::RobotModelPtr model);

  /**
   * @brief Given a desired pose of the end-effector, compute the joint angles to reach it
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  getPositionIK(const geometry_msgs::Pose &ik_pose,
                const std::vector<double> &ik_seed_state,
                std::vector<double> &solution,
                moveit_msgs::MoveItErrorCodes &error_code,
                const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
 * @brief Given a desired pose of the end-effector, compute the set joint angles solutions that are able to reach it.
 *
 * This is a default implementation that returns only one solution and so its result is equivalent to calling
 * 'getPositionIK(...)' with a zero initialized seed.
 *
 * @param ik_poses  The desired pose of each tip link
 * @param ik_seed_state an initial guess solution for the inverse kinematics
 * @param solutions A vector of vectors where each entry is a valid joint solution
 * @param result A struct that reports the results of the query
 * @param options An option struct which contains the type of redundancy discretization used. This default
 *                implementation only supports the KinematicSearches::NO_DISCRETIZATION method; requesting any
 *                other will result in failure.
 * @return True if a valid set of solutions was found, false otherwise.
 */
  virtual bool getPositionIK(const std::vector<geometry_msgs::Pose> &ik_poses,
                             const std::vector<double> &ik_seed_state,
                             std::vector<std::vector<double>> &solutions,
                             KinematicsResult &result,
                             const kinematics::KinematicsQueryOptions &options) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state,
                   double timeout,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state,
                   double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool searchPositionIK(
      const geometry_msgs::Pose &ik_pose,
      const std::vector<double> &ik_seed_state,
      double timeout,
      std::vector<double> &solution,
      const IKCallbackFn &solution_callback,
      moveit_msgs::MoveItErrorCodes &error_code,
      const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a desired pose of the end-effector, search for the joint angles required to reach it.
   * This particular method is intended for "searching" for a solutions by stepping through the redundancy
   * (or other numerical routines).
   * @param ik_pose the desired pose of the link
   * @param ik_seed_state an initial guess solution for the inverse kinematics
   * @param timeout The amount of time (in seconds) available to the solver
   * @param consistency_limits the distance that any joint in the solution can be from the corresponding joints in the
   * current seed state
   * @param solution the solution vector
   * @param solution_callback A callback solution for the IK solution
   * @param error_code an error code that encodes the reason for failure or success
   * @param lock_redundant_joints if setRedundantJoints() was previously called, keep the values of the joints marked as
   * redundant the same as in the seed
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool
  searchPositionIK(const geometry_msgs::Pose &ik_pose,
                   const std::vector<double> &ik_seed_state,
                   double timeout,
                   const std::vector<double> &consistency_limits,
                   std::vector<double> &solution,
                   const IKCallbackFn &solution_callback,
                   moveit_msgs::MoveItErrorCodes &error_code,
                   const kinematics::KinematicsQueryOptions &options = kinematics::KinematicsQueryOptions()) const;

  /**
   * @brief Given a set of joint angles and a set of links, compute their pose
   * @param link_names A set of links for which FK needs to be computed
   * @param joint_angles The state for which FK is being computed
   * @param poses The resultant set of poses (in the frame returned by getBaseFrame())
   * @return True if a valid solution was found, false otherwise
   */
  virtual bool getPositionFK(const std::vector<std::string> &link_names,
                             const std::vector<double> &joint_angles,
                             std::vector<geometry_msgs::Pose> &poses) const;

  /**
   * @brief  Initialization function for the kinematics, for use with kinematic chain IK solvers
   * @param robot_description This parameter can be used as an identifier for the robot kinematics it is computed for;
   * For example, the name of the ROS parameter that contains the robot description;
   * @param group_name The group for which this solver is being configured
   * @param base_frame The base frame in which all input poses are expected.
   * This may (or may not) be the root frame of the chain that the solver operates on
   * @param tip_frame The tip of the chain
   * @param search_discretization The discretization of the search when the solver steps through the redundancy
   * @return True if initialization was successful, false otherwise
   */
  virtual bool initialize(const std::string &robot_description,
                          const std::string &group_name,
                          const std::string &base_frame,
                          const std::string &tip_frame,
                          double search_discretization);
  
  bool initialize(robot_model::RobotModelPtr model);

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  virtual const std::vector<std::string> &getJointNames() const;

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  virtual const std::vector<std::string> &getLinkNames() const;

  /**
   * @brief  Virtual destructor for the interface
   */
  virtual ~MoveItOPWKinematicsPlugin()
  {
  }

private:
  /** @brief Get parameters needed for the ik solver from parameter server.
   * The parameters must be specified in the kinematics.yaml file.
   * @return True if all parameters are succesfully loaded
   */
  bool setOPWParameters();

  // use fucntions from descartes_owp_model for inverse kinematics calculations
  bool getAllIK(const Eigen::Affine3d &pose, std::vector<std::vector<double>> &joint_poses) const;
  bool getIK(const Eigen::Affine3d &pose, const std::vector<double> &seed_state, std::vector<double> &joint_pose) const;

  /** @brief Compute the 'joint distance' between two poses.
   */
  double distance(const std::vector<double> &a, const std::vector<double> &b) const;

  /** @brief Compute the index of the closest joint pose in 'candidates' from 'target'.
   */
 std::size_t closestJointPose(const std::vector<double> &target, const std::vector<std::vector<double>> &candidates) const;

  std::string robot_description_;
  std::string group_name_;
  std::string base_frame_;
  std::vector<std::string> tip_frames_;

  std::vector<std::string> joint_names_;
  std::vector<std::string> link_names_;

  double default_timeout_;
  std::vector<unsigned int> redundant_joint_indices_;
  std::map<int, double> redundant_joint_discretization_;

  robot_model::RobotModelPtr robot_model_;
  robot_model::JointModelGroup *joint_model_group_;
  robot_state::RobotStatePtr robot_state_;

  int num_possible_redundant_joints_;

  opw_kinematics::Parameters<double> opw_parameters_;
};

} // namespace moveit_opw_kinematics_plugin

#endif /* MOVEIT_OPW_KINEMATICS_PLUGIN_H */