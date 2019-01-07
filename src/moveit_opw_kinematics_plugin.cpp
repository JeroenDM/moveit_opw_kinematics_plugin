#include <class_loader/class_loader.hpp>
#include <moveit_opw_kinematics_plugin/moveit_opw_kinematics_plugin.h>

// URDF, SRDF
#include <srdfdom/model.h>
#include <urdf_model/model.h>

#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/rdf_loader/rdf_loader.h>
#include <moveit/robot_state/conversions.h>

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

// OPW kinematics
#include "opw_kinematics/opw_io.h"
#include "opw_kinematics/opw_kinematics.h"
#include "opw_kinematics/opw_utilities.h"

// register OPWKinematics as a KinematicsBase implementation
CLASS_LOADER_REGISTER_CLASS(moveit_opw_kinematics_plugin::MoveItOPWKinematicsPlugin, kinematics::KinematicsBase)

namespace moveit_opw_kinematics_plugin
{
using kinematics::KinematicsResult;

MoveItOPWKinematicsPlugin::MoveItOPWKinematicsPlugin() : active_(false)
{
}

bool MoveItOPWKinematicsPlugin::initialize(const std::string& robot_description, const std::string& group_name,
                                           const std::string& base_frame, const std::vector<std::string>& tip_frames,
                                           double search_discretization)
{
  bool debug = false;

  ROS_INFO_STREAM_NAMED("opw", "MoveItOPWKinematicsPlugin initializing");

  setValues(robot_description, group_name, base_frame, tip_frames, search_discretization);

  rdf_loader::RDFLoader rdf_loader(robot_description_);
  const srdf::ModelSharedPtr& srdf = rdf_loader.getSRDF();
  const urdf::ModelInterfaceSharedPtr& urdf_model = rdf_loader.getURDF();

  if (!urdf_model || !srdf)
  {
    ROS_ERROR_NAMED("opw", "URDF and SRDF must be loaded for SRV kinematics "
                           "solver to work.");  // TODO: is this true?
    return false;
  }

  robot_model_.reset(new robot_model::RobotModel(urdf_model, srdf));

  joint_model_group_ = robot_model_->getJointModelGroup(group_name);
  if (!joint_model_group_)
    return false;

  if (debug)
  {
    std::cout << std::endl
              << "Joint Model Variable Names: "
                 "------------------------------------------- "
              << std::endl;
    const std::vector<std::string> jm_names = joint_model_group_->getVariableNames();
    std::copy(jm_names.begin(), jm_names.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
    std::cout << std::endl;
  }

  // Get the dimension of the planning group
  dimension_ = joint_model_group_->getVariableCount();
  ROS_INFO_STREAM_NAMED("opw", "Dimension planning group '"
                                   << group_name << "': " << dimension_
                                   << ". Active Joints Models: " << joint_model_group_->getActiveJointModels().size()
                                   << ". Mimic Joint Models: " << joint_model_group_->getMimicJointModels().size());

  // Copy joint names
  for (std::size_t i = 0; i < joint_model_group_->getJointModels().size(); ++i)
  {
    ik_group_info_.joint_names.push_back(joint_model_group_->getJointModelNames()[i]);
  }

  if (debug)
  {
    ROS_ERROR_STREAM_NAMED("opw", "tip links available:");
    std::copy(tip_frames_.begin(), tip_frames_.end(), std::ostream_iterator<std::string>(std::cout, "\n"));
  }

  // Make sure all the tip links are in the link_names vector
  for (std::size_t i = 0; i < tip_frames_.size(); ++i)
  {
    if (!joint_model_group_->hasLinkModel(tip_frames_[i]))
    {
      ROS_ERROR_NAMED("opw", "Could not find tip name '%s' in joint group '%s'", tip_frames_[i].c_str(),
                      group_name.c_str());
      return false;
    }
    ik_group_info_.link_names.push_back(tip_frames_[i]);
  }

  // Choose what ROS service to send IK requests to
  ROS_DEBUG_STREAM_NAMED("opw", "Looking for ROS service name on rosparam server with param: "
                                    << "/kinematics_solver_service_name");
  std::string ik_service_name;
  lookupParam("kinematics_solver_service_name", ik_service_name, std::string("solve_ik"));

  // Setup the joint state groups that we need
  robot_state_.reset(new robot_state::RobotState(robot_model_));
  robot_state_->setToDefaultValues();

  // set geometric parameters for opw model
  if (!setOPWParameters())
  {
    ROS_ERROR_STREAM_NAMED("opw", "Could not load opw parameters. Check kinematics.yaml.");
    return false;
  }

  active_ = true;
  ROS_DEBUG_NAMED("opw", "ROS service-based kinematics solver initialized");
  return true;
}

bool MoveItOPWKinematicsPlugin::setRedundantJoints(const std::vector<unsigned int>& redundant_joints)
{
  if (num_possible_redundant_joints_ < 0)
  {
    ROS_ERROR_NAMED("srv", "This group cannot have redundant joints");
    return false;
  }
  if (redundant_joints.size() > num_possible_redundant_joints_)
  {
    ROS_ERROR_NAMED("srv", "This group can only have %d redundant joints", num_possible_redundant_joints_);
    return false;
  }

  return true;
}

bool MoveItOPWKinematicsPlugin::isRedundantJoint(unsigned int index) const
{
  for (std::size_t j = 0; j < redundant_joint_indices_.size(); ++j)
    if (redundant_joint_indices_[j] == index)
      return true;
  return false;
}

int MoveItOPWKinematicsPlugin::getJointIndex(const std::string& name) const
{
  for (unsigned int i = 0; i < ik_group_info_.joint_names.size(); i++)
  {
    if (ik_group_info_.joint_names[i] == name)
      return i;
  }
  return -1;
}

bool MoveItOPWKinematicsPlugin::timedOut(const ros::WallTime& start_time, double duration) const
{
  return ((ros::WallTime::now() - start_time).toSec() >= duration);
}

bool MoveItOPWKinematicsPlugin::getPositionIK(const geometry_msgs::Pose& ik_pose,
                                              const std::vector<double>& ik_seed_state, std::vector<double>& solution,
                                              moveit_msgs::MoveItErrorCodes& error_code,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, default_timeout_, solution, solution_callback, error_code,
                          consistency_limits, options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  std::vector<double> consistency_limits;

  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  const IKCallbackFn solution_callback = 0;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  std::vector<double> consistency_limits;
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  return searchPositionIK(ik_pose, ik_seed_state, timeout, solution, solution_callback, error_code, consistency_limits,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const geometry_msgs::Pose& ik_pose,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const std::vector<double>& consistency_limits,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Convert single pose into a vector of one pose
  std::vector<geometry_msgs::Pose> ik_poses;
  ik_poses.push_back(ik_pose);

  return searchPositionIK(ik_poses, ik_seed_state, timeout, consistency_limits, solution, solution_callback, error_code,
                          options);
}

bool MoveItOPWKinematicsPlugin::searchPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                                 const std::vector<double>& ik_seed_state, double timeout,
                                                 const std::vector<double>& consistency_limits,
                                                 std::vector<double>& solution, const IKCallbackFn& solution_callback,
                                                 moveit_msgs::MoveItErrorCodes& error_code,
                                                 const kinematics::KinematicsQueryOptions& options) const
{
  // Check if active
  if (!active_)
  {
    ROS_ERROR_NAMED("opw", "kinematics not active");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check if seed state correct
  if (ik_seed_state.size() != dimension_)
  {
    ROS_ERROR_STREAM_NAMED("opw", "Seed state must have size " << dimension_ << " instead of size "
                                                               << ik_seed_state.size());
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != ik_poses.size())
  {
    ROS_ERROR_STREAM_NAMED("opw", "Mismatched number of pose requests (" << ik_poses.size() << ") to tip frames ("
                                                                         << tip_frames_.size()
                                                                         << ") in searchPositionIK");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  if (!getIK(pose, ik_seed_state, solution))
  {
    ROS_ERROR_STREAM_NAMED("opw", "Failed to find IK solution");
    error_code.val = error_code.NO_IK_SOLUTION;
    return false;
  }

  return true;
}

bool MoveItOPWKinematicsPlugin::getPositionIK(const std::vector<geometry_msgs::Pose>& ik_poses,
                                              const std::vector<double>& ik_seed_state,
                                              std::vector<std::vector<double>>& solutions, KinematicsResult& result,
                                              const kinematics::KinematicsQueryOptions& options) const
{
  if (ik_poses.size() > 1 || ik_poses.size() == 0)
  {
    ROS_ERROR_STREAM_NAMED("opw", "You can only get all solutions for a single pose.");
    return false;
  }
  Eigen::Affine3d pose;
  tf::poseMsgToEigen(ik_poses[0], pose);
  return getAllIK(pose, solutions);
}

bool MoveItOPWKinematicsPlugin::getPositionFK(const std::vector<std::string>& link_names,
                                              const std::vector<double>& joint_angles,
                                              std::vector<geometry_msgs::Pose>& poses) const
{
  ros::WallTime n1 = ros::WallTime::now();
  if (!active_)
  {
    ROS_ERROR_NAMED("opw", "kinematics not active");
    return false;
  }
  poses.resize(link_names.size());
  if (joint_angles.size() != dimension_)
  {
    ROS_ERROR_NAMED("opw", "Joint angles vector must have size: %d", dimension_);
    return false;
  }

  // Check that we have the same number of poses as tips
  if (tip_frames_.size() != poses.size())
  {
    ROS_ERROR_STREAM_NAMED("opw", "Mismatched number of pose requests (" << poses.size() << ") to tip frames ("
                                                                         << tip_frames_.size()
                                                                         << ") in searchPositionFK");
    return false;
  }

  // forward function expect pointer to first element of array of joint values
  // that is why &joint_angles[0] is passed
  tf::poseEigenToMsg(opw_kinematics::forward(opw_parameters_, &joint_angles[0]), poses[0]);

  return true;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getJointNames() const
{
  return ik_group_info_.joint_names;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getLinkNames() const
{
  return ik_group_info_.link_names;
}

const std::vector<std::string>& MoveItOPWKinematicsPlugin::getVariableNames() const
{
  return joint_model_group_->getVariableNames();
}

bool MoveItOPWKinematicsPlugin::setOPWParameters()
{
  ROS_INFO_STREAM("Getting kinematic parameters from parameter server.");

  // Using the full parameter name at the moment because the leading slash
  // in front of robot_description_kinematics is missing
  // in lookupParam function, but I'm not sure if this is a bug or I do not
  // understand the interface.
  std::string prefix = "/robot_description_kinematics/" + group_name_ + "/";
  ros::NodeHandle nh;

  std::map<std::string, double> geometric_parameters, dummy;
  if (!lookupParam(prefix + "kinematics_solver_geometric_parameters", geometric_parameters, dummy))
  {
    ROS_ERROR_STREAM("Failed to load geometric parameters for ik solver.");
    return false;
  }

  std::vector<double> joint_offsets, dummy2;
  if (!lookupParam(prefix + "kinematics_solver_joint_offsets", joint_offsets, dummy2))
  {
    ROS_ERROR_STREAM("Failed to load joint offsets for ik solver.");
    return false;
  }

  std::vector<int> joint_sign_corrections, dummy3;
  if (!lookupParam(prefix + "kinematics_solver_joint_sign_corrections", joint_sign_corrections, dummy3))
  {
    ROS_ERROR_STREAM("Failed to load joint sign corrections for ik solver.");
    return false;
  }

  opw_parameters_.a1 = geometric_parameters["a1"];
  opw_parameters_.a2 = geometric_parameters["a2"];
  opw_parameters_.b = geometric_parameters["b"];
  opw_parameters_.c1 = geometric_parameters["c1"];
  opw_parameters_.c2 = geometric_parameters["c2"];
  opw_parameters_.c3 = geometric_parameters["c3"];
  opw_parameters_.c4 = geometric_parameters["c4"];
  for (std::size_t i = 0; i < joint_offsets.size(); ++i)
  {
    opw_parameters_.offsets[i] = joint_offsets[i];
    opw_parameters_.sign_corrections[i] = joint_sign_corrections[i];
  }

  ROS_INFO_STREAM("Loaded parameters for ik solver:\n" << opw_parameters_);

  return true;
}

double MoveItOPWKinematicsPlugin::distance(const std::vector<double>& a, const std::vector<double>& b) const
{
  double cost = 0.0;
  for (size_t i = 0; i < a.size(); ++i)
    cost += std::abs(b[i] - a[i]);
  return cost;
}

// Compute the index of the closest joint pose in 'candidates' from 'target'
std::size_t MoveItOPWKinematicsPlugin::closestJointPose(const std::vector<double>& target,
                                                        const std::vector<std::vector<double>>& candidates) const
{
  size_t closest = 0;  // index into candidates
  double lowest_cost = std::numeric_limits<double>::max();
  for (size_t i = 0; i < candidates.size(); ++i)
  {
    assert(target.size() == candidates[i].size());
    double c = distance(target, candidates[i]);
    if (c < lowest_cost)
    {
      closest = i;
      lowest_cost = c;
    }
  }
  return closest;
}

bool MoveItOPWKinematicsPlugin::getAllIK(const Eigen::Affine3d& pose,
                                         std::vector<std::vector<double>>& joint_poses) const
{
  joint_poses.clear();

  // Transform input pose
  // needed if we introduce a tip frame different from tool0
  // or a different base frame
  // Eigen::Affine3d tool_pose = diff_base.inverse() * pose *
  // tip_frame.inverse();

  // convert Eigen::Affine3d to Eigen::Isometry3d for opw_kinematics
  Eigen::Isometry3d pose_isometry;
  pose_isometry = pose.matrix();

  std::array<double, 6 * 8> sols;
  opw_kinematics::inverse(opw_parameters_, pose_isometry, sols.data());

  // Check the output
  std::vector<double> tmp(6);  // temporary storage for API reasons
  for (int i = 0; i < 8; i++)
  {
    double* sol = sols.data() + 6 * i;
    if (opw_kinematics::isValid(sol))
    {
      opw_kinematics::harmonizeTowardZero(sol);

      // TODO: make this better...
      std::copy(sol, sol + 6, tmp.data());
      // if (isValid(tmp))
      // {
      joint_poses.push_back(tmp);
      // }
    }
  }

  return joint_poses.size() > 0;
}

bool MoveItOPWKinematicsPlugin::getIK(const Eigen::Affine3d& pose, const std::vector<double>& seed_state,
                                      std::vector<double>& joint_pose) const
{
  // Descartes Robot Model interface calls for 'closest' point to seed position
  std::vector<std::vector<double>> joint_poses;
  if (!getAllIK(pose, joint_poses))
    return false;
  // Find closest joint pose; getAllIK() does isValid checks already
  joint_pose = joint_poses[closestJointPose(seed_state, joint_poses)];
  return true;
}

}  // namespace moveit_opw_kinematics_plugin
