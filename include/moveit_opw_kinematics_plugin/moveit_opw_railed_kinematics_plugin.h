#ifndef MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_
#define MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_

#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;
/**
 * @brief Specific implementation of kinematics using ROS service calls to communicate with
   external IK solvers. This version can be used with any robot. Supports non-chain kinematic groups
 */
class MoveItOPWRailedKinematicsPlugin : public MoveItOPWKinematicsPlugin
{
public:
  /**
   *  @brief Default constructor
   */
  MoveItOPWRailedKinematicsPlugin();

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

  virtual bool
  getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                std::vector<std::vector<double>>& solutions, KinematicsResult& result,
                const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

protected:
  virtual bool
  searchPositionIK(const geometry_msgs::Pose& ik_pose, const std::vector<double>& ik_seed_state, double timeout,
                   std::vector<double>& solution, const IKCallbackFn& solution_callback,
                   moveit_msgs::MoveItErrorCodes& error_code, const std::vector<double>& consistency_limits,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions()) const override;

  virtual bool
  searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses, const std::vector<double>& ik_seed_state,
                   double timeout, const std::vector<double>& consistency_limits, std::vector<double>& solution,
                   const IKCallbackFn& solution_callback, moveit_msgs::MoveItErrorCodes& error_code,
                   const kinematics::KinematicsQueryOptions& options = kinematics::KinematicsQueryOptions(),
                   const moveit::core::RobotState* context_state = NULL) const override;

  bool getFreeJointLimits(double& min, double& max) const;

  Eigen::Isometry3d getRobotBaseTransform(const double rail_position) const;

  geometry_msgs::Pose getUpdatedPose(const geometry_msgs::Pose& pose,
                                     const double rail_position) const;

  std::string free_joint_;
};
}  // namespace moveit_opw_kinematics_plugin

#endif // MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_
