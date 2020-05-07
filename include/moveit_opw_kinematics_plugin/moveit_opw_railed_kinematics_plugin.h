#ifndef MOVEIT_OPW_KINEMATICS_PLUGIN_MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_
#define MOVEIT_OPW_KINEMATICS_PLUGIN_MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_

#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;

/**
 * @brief Kinematics plugin implementation leveraging the MoveItOPWKinematics plugin for robots mounted on a rail
 */
class MoveItOPWRailedKinematicsPlugin : public kinematics::KinematicsBase
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

  inline virtual bool initialize(const std::string& robot_description, const std::string& group_name,
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

  /**
   * @brief  Return all the joint names in the order they are used internally
   */
  const inline std::vector<std::string>& getJointNames() const override
  {
    return ik_group_info_.joint_names;
  }

  /**
   * @brief  Return all the link names in the order they are represented internally
   */
  const inline std::vector<std::string>& getLinkNames() const override
  {
    return ik_group_info_.link_names;
  }

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

  bool getFreeJointLimits(double& min, double& max) const;

  /**
   * @brief Calculates the transform to the OPW robot base frame at the input rail joint position
   * @param rail_position - joint positoin of the rail
   * @return
   */
  Eigen::Isometry3d getOPWBaseTransform(const double rail_position) const;

  /**
   * @brief Calculates the target IK pose in the OPW robot base frame at the input rail joint position
   * @param pose - target IK pose
   * @param rail_position - joint position of the rail
   * @return
   */
  geometry_msgs::Pose getUpdatedPose(const geometry_msgs::Pose& pose,
                                     const double rail_position) const;

  std::string opw_group_name_;                            /** @brief Name of the planning group that represents the OPW robot */
  std::string opw_base_frame_;                            /** @brief Name of the OPW robot base frame */
  moveit_msgs::KinematicSolverInfo ik_group_info_;        /** @brief Solver information */
  robot_model::RobotModelConstPtr robot_model_;           /** @brief MoveIt robot model */
  robot_state::RobotStatePtr robot_state_;                /** @brief MoveIt robot state */
  const robot_model::JointModelGroup* joint_model_group_; /** @brief MoveIt joint model group */
  MoveItOPWKinematicsPlugin opw_plugin_;                  /** @brief OPW kinematics plugin used for robot kinematics */
};
}  // namespace moveit_opw_kinematics_plugin

#endif // MOVEIT_OPW_KINEMATICS_PLUGIN_MOVEIT_OPW_RAILED_KINEMATICS_PLUGIN_
