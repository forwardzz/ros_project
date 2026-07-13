#ifndef INSPECTION_ROBOT_DWA_CONTROLLER__DWA_CONTROLLER_HPP_
#define INSPECTION_ROBOT_DWA_CONTROLLER__DWA_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav2_core/controller.hpp"
#include "nav2_core/goal_checker.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace inspection_robot_dwa_controller
{

class DWAController : public nav2_core::Controller
{
public:
  DWAController() = default;
  ~DWAController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override;

  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void reset() override;

private:
  struct Trajectory
  {
    double vx{0.0};
    double wz{0.0};
    double total_cost{0.0};
    double path_cost{0.0};
    double goal_cost{0.0};
    double goal_front_cost{0.0};
    double alignment_cost{0.0};
    double obstacle_cost{0.0};
    double twirling_cost{0.0};
    double oscillation_cost{0.0};
    double linear_velocity_change_cost{0.0};
    double angular_velocity_change_cost{0.0};
    std::vector<geometry_msgs::msg::Pose2D> poses;
  };

  void readParameters();
  void declareParameter(const std::string & name, const rclcpp::ParameterValue & value);
  double getDouble(const std::string & name);
  int getInt(const std::string & name);
  bool getBool(const std::string & name);

  nav_msgs::msg::Path transformGlobalPlan(const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::PoseStamped transformPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame) const;
  geometry_msgs::msg::PoseStamped transformPlanPose(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & target_frame) const;
  geometry_msgs::msg::PoseStamped poseWithFallbackFrame(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & fallback_frame) const;
  std::string getPlanFrame() const;

  std::vector<double> sampleRange(double min_value, double max_value, int samples) const;
  std::vector<double> sampleLinearVelocities(double current_vx) const;
  std::vector<double> sampleAngularVelocities(double current_wz) const;

  bool makeTrajectory(
    const geometry_msgs::msg::PoseStamped & pose,
    double vx,
    double wz,
    Trajectory & trajectory) const;
  bool scoreTrajectory(
    Trajectory & trajectory,
    const nav_msgs::msg::Path & local_plan,
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::PoseStamped & goal_pose) const;

  bool isCollisionFree(
    const geometry_msgs::msg::Pose2D & pose,
    double & cost) const;
  double distanceToPath(double x, double y, const nav_msgs::msg::Path & path) const;
  double distanceToPose(double x, double y, const geometry_msgs::msg::PoseStamped & pose) const;
  double yawFromPose(const geometry_msgs::msg::PoseStamped & pose) const;
  double pathHeadingError(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const nav_msgs::msg::Path & local_plan) const;
  double normalizeCostmapCost(double cost) const;
  double getGoalPositionTolerance(nav2_core::GoalChecker * goal_checker) const;
  double getGoalYawTolerance(nav2_core::GoalChecker * goal_checker) const;
  bool shouldRotateToGoal(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    const geometry_msgs::msg::PoseStamped & goal_pose,
    nav2_core::GoalChecker * goal_checker,
    geometry_msgs::msg::TwistStamped & command);

  void publishPlan(
    const nav_msgs::msg::Path & path,
    const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr & publisher) const;
  nav_msgs::msg::Path trajectoryToPath(
    const Trajectory & trajectory,
    const std::string & frame_id,
    const rclcpp::Time & stamp) const;

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Logger logger_{rclcpp::get_logger("inspection_robot_dwa_controller")};
  rclcpp::Clock::SharedPtr clock_;
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav2_costmap_2d::Costmap2D * costmap_{nullptr};
  std::unique_ptr<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>
  collision_checker_;

  nav_msgs::msg::Path global_plan_;
  bool configured_{false};
  bool active_{false};
  bool speed_limited_{false};
  double speed_limit_{0.0};
  double last_cmd_vx_{0.0};
  double last_cmd_wz_{0.0};
  bool rotating_to_path_{false};
  geometry_msgs::msg::Pose2D oscillation_reset_pose_;
  bool have_oscillation_reset_pose_{false};

  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr global_plan_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr local_plan_pub_;

  double controller_frequency_{10.0};
  double sim_time_{1.3};
  double sim_granularity_{0.05};
  double angular_sim_granularity_{0.05};
  bool use_dwa_{true};
  double transform_tolerance_{0.5};
  double prune_distance_{1.0};
  double max_robot_pose_search_dist_{2.0};
  double forward_plan_distance_{2.0};
  double min_vel_x_{0.0};
  double max_vel_x_{0.18};
  double max_vel_theta_{0.65};
  double min_speed_xy_{0.0};
  double acc_lim_x_{0.35};
  double decel_lim_x_{-0.45};
  double acc_lim_theta_{0.70};
  double decel_lim_theta_{-0.90};
  int vx_samples_{6};
  int vtheta_samples_{15};
  int max_local_plan_poses_{60};
  bool debug_timing_{false};
  double timing_warn_ms_{80.0};
  double path_distance_bias_{12.0};
  double goal_distance_bias_{8.0};
  double goal_front_bias_{4.0};
  double alignment_bias_{6.0};
  double occdist_scale_{0.08};
  double twirling_scale_{0.2};
  double oscillation_scale_{8.0};
  double prefer_forward_scale_{8.0};
  double linear_velocity_change_scale_{2.0};
  double angular_velocity_change_scale_{0.5};
  double forward_point_distance_{0.25};
  double path_heading_lookahead_{0.35};
  double rotate_to_path_engage_angle_{0.60};
  double rotate_to_path_disengage_angle_{0.25};
  double obstacle_cost_threshold_{254.0};
  bool allow_unknown_{false};
  double oscillation_reset_dist_{0.08};
  double oscillation_reset_angle_{0.25};
  double xy_goal_tolerance_{0.10};
  double yaw_goal_tolerance_{0.25};
  bool rotate_to_goal_heading_{false};
  double rotate_to_goal_angular_vel_{0.35};
  double trans_stopped_velocity_{0.03};
  double theta_stopped_velocity_{0.05};
  double min_approach_vel_x_{0.03};
  double min_dwa_window_vel_x_{0.03};
  double approach_slowdown_distance_{0.35};
};

}  // namespace inspection_robot_dwa_controller

#endif  // INSPECTION_SIM_DWA_CONTROLLER__DWA_CONTROLLER_HPP_
