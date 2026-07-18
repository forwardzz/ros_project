#include "inspection_robot_dwa_controller/dwa_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "angles/angles.h"
#include "nav2_core/controller_exceptions.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2/time.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace inspection_robot_dwa_controller
{
namespace
{

double clamp(double value, double lower, double upper)
{
  return std::max(lower, std::min(upper, value));
}

double hypot2(double x, double y)
{
  return std::hypot(x, y);
}

geometry_msgs::msg::Quaternion yawToQuaternion(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  return tf2::toMsg(q);
}

}  // namespace

void DWAController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  if (!node_) {
    throw nav2_core::ControllerException("DWAController failed to lock parent node");
  }

  plugin_name_ = std::move(name);
  logger_ = node_->get_logger();
  clock_ = node_->get_clock();
  tf_ = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);
  costmap_ = costmap_ros_->getCostmap();
  collision_checker_ =
    std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(
    costmap_);

  readParameters();

  global_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/global_plan", rclcpp::SystemDefaultsQoS());
  local_plan_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    plugin_name_ + "/local_plan", rclcpp::SystemDefaultsQoS());

  configured_ = true;
  RCLCPP_INFO(
    logger_,
    "Configured Nav2 DWA controller '%s' with vx=[%.2f, %.2f], wz=%.2f, samples=%dx%d",
    plugin_name_.c_str(), min_vel_x_, max_vel_x_, max_vel_theta_, vx_samples_, vtheta_samples_);
}

void DWAController::cleanup()
{
  global_plan_pub_.reset();
  local_plan_pub_.reset();
  collision_checker_.reset();
  global_plan_ = nav_msgs::msg::Path();
  rotating_to_path_ = false;
  configured_ = false;
  active_ = false;
}

void DWAController::activate()
{
  if (global_plan_pub_) {
    global_plan_pub_->on_activate();
  }
  if (local_plan_pub_) {
    local_plan_pub_->on_activate();
  }
  active_ = true;
}

void DWAController::deactivate()
{
  active_ = false;
  if (global_plan_pub_) {
    global_plan_pub_->on_deactivate();
  }
  if (local_plan_pub_) {
    local_plan_pub_->on_deactivate();
  }
}

void DWAController::setPlan(const nav_msgs::msg::Path & path)
{
  if (path.poses.empty()) {
    throw nav2_core::InvalidPath("DWAController received an empty global plan");
  }
  global_plan_ = path;
  reset();
}

geometry_msgs::msg::TwistStamped DWAController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  nav2_core::GoalChecker * goal_checker)
{
  const auto compute_start = std::chrono::steady_clock::now();
  auto warn_if_slow = [this, &compute_start](
      size_t local_plan_poses,
      size_t vx_sample_count,
      size_t wz_sample_count,
      size_t trajectory_count,
      size_t valid_trajectory_count,
      size_t max_trajectory_poses)
    {
      if (!debug_timing_) {
        return;
      }

      const auto elapsed = std::chrono::steady_clock::now() - compute_start;
      const double elapsed_ms =
        std::chrono::duration<double, std::milli>(elapsed).count();
      if (elapsed_ms <= timing_warn_ms_) {
        return;
      }

      RCLCPP_WARN(
        logger_,
        "DWA compute took %.1f ms: local_plan_poses=%zu, samples=%zux%zu, "
        "trajectories=%zu, valid=%zu, max_trajectory_poses=%zu",
        elapsed_ms, local_plan_poses, vx_sample_count, wz_sample_count,
        trajectory_count, valid_trajectory_count, max_trajectory_poses);
    };

  if (!configured_) {
    throw nav2_core::ControllerException("DWAController is not configured");
  }
  if (global_plan_.poses.empty()) {
    throw nav2_core::InvalidPath("DWAController cannot compute without a global plan");
  }

  costmap_ = costmap_ros_->getCostmap();
  collision_checker_->setCostmap(costmap_);

  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  geometry_msgs::msg::PoseStamped robot_pose = transformPose(pose, costmap_frame);
  nav_msgs::msg::Path local_plan = transformGlobalPlan(robot_pose);
  if (local_plan.poses.empty()) {
    throw nav2_core::InvalidPath("DWAController transformed plan is empty");
  }

  geometry_msgs::msg::PoseStamped goal_pose =
    transformPlanPose(poseWithFallbackFrame(global_plan_.poses.back(), getPlanFrame()), costmap_frame);

  const double goal_distance = distanceToPose(
    robot_pose.pose.position.x, robot_pose.pose.position.y, goal_pose);
  const double xy_goal_tolerance = getGoalPositionTolerance(goal_checker);

  publishPlan(local_plan, global_plan_pub_);

  geometry_msgs::msg::TwistStamped command;
  command.header.stamp = clock_->now();
  command.header.frame_id = costmap_ros_->getBaseFrameID();

  if (shouldRotateToGoal(robot_pose, velocity, goal_pose, goal_checker, command)) {
    nav_msgs::msg::Path empty_local;
    empty_local.header.frame_id = costmap_frame;
    empty_local.header.stamp = command.header.stamp;
    publishPlan(empty_local, local_plan_pub_);
    last_cmd_vx_ = command.twist.linear.x;
    last_cmd_wz_ = command.twist.angular.z;
    warn_if_slow(local_plan.poses.size(), 0, 0, 0, 0, 0);
    return command;
  }

  if (!have_oscillation_reset_pose_) {
    oscillation_reset_pose_.x = robot_pose.pose.position.x;
    oscillation_reset_pose_.y = robot_pose.pose.position.y;
    oscillation_reset_pose_.theta = yawFromPose(robot_pose);
    have_oscillation_reset_pose_ = true;
  } else {
    const double reset_dist = hypot2(
      robot_pose.pose.position.x - oscillation_reset_pose_.x,
      robot_pose.pose.position.y - oscillation_reset_pose_.y);
    const double reset_angle = std::abs(
      angles::shortest_angular_distance(oscillation_reset_pose_.theta, yawFromPose(robot_pose)));
    if (reset_dist >= oscillation_reset_dist_ || reset_angle >= oscillation_reset_angle_) {
      oscillation_reset_pose_.x = robot_pose.pose.position.x;
      oscillation_reset_pose_.y = robot_pose.pose.position.y;
      oscillation_reset_pose_.theta = yawFromPose(robot_pose);
    }
  }

  const double path_heading_error = pathHeadingError(robot_pose, local_plan);
  if (rotating_to_path_) {
    if (std::abs(path_heading_error) <= rotate_to_path_disengage_angle_) {
      rotating_to_path_ = false;
    }
  } else if (std::abs(path_heading_error) >= rotate_to_path_engage_angle_) {
    rotating_to_path_ = true;
  }

  std::vector<double> vx_samples = sampleLinearVelocities(velocity.linear.x);
  std::vector<double> wz_samples = sampleAngularVelocities(velocity.angular.z);

  if (goal_distance > xy_goal_tolerance) {
    const double effective_max_vel_x = speed_limited_ ?
      std::min(max_vel_x_, std::max(0.0, speed_limit_)) : max_vel_x_;
    const double effective_min_approach =
      std::min(min_approach_vel_x_, effective_max_vel_x);

    double effective_approach_max = effective_max_vel_x;
    if (
      approach_slowdown_distance_ > xy_goal_tolerance &&
      goal_distance < approach_slowdown_distance_)
    {
      const double ratio = clamp(
        (goal_distance - xy_goal_tolerance) /
        (approach_slowdown_distance_ - xy_goal_tolerance),
        0.0, 1.0);
      effective_approach_max = std::max(
        effective_min_approach, effective_max_vel_x * ratio);
    }

    vx_samples.erase(std::remove_if(vx_samples.begin(), vx_samples.end(),
        [effective_min_approach, effective_approach_max](double v) {
          return v > 1e-6 &&
                 (v < effective_min_approach || v > effective_approach_max);
        }), vx_samples.end());

    const bool has_forward = std::any_of(
      vx_samples.begin(), vx_samples.end(), [](double v) {
        return v > 1e-6;
      });
    if (!has_forward && effective_approach_max > 0.0) {
      const double dwa_floor = std::min(min_dwa_window_vel_x_, effective_max_vel_x);
      const double candidate = std::min(
        effective_approach_max,
        std::max(effective_min_approach, dwa_floor));
      vx_samples.push_back(candidate);
      std::sort(vx_samples.begin(), vx_samples.end());
    }
  }

  bool found = false;
  Trajectory best;
  best.total_cost = std::numeric_limits<double>::infinity();
  size_t trajectory_count = 0;
  size_t valid_trajectory_count = 0;
  size_t max_trajectory_poses = 0;

  auto consider_trajectory = [&](double vx, double wz) {
    Trajectory trajectory;
    if (!makeTrajectory(robot_pose, vx, wz, trajectory)) {
      return;
    }
    ++trajectory_count;
    max_trajectory_poses = std::max(max_trajectory_poses, trajectory.poses.size());
    if (!scoreTrajectory(trajectory, local_plan, robot_pose, goal_pose)) {
      return;
    }
    ++valid_trajectory_count;
    if (
      !found || trajectory.total_cost < best.total_cost ||
      (std::abs(trajectory.total_cost - best.total_cost) < 1e-6 && trajectory.vx > best.vx))
    {
      best = trajectory;
      found = true;
    }
  };

  if (!rotating_to_path_) {
    for (const double vx : vx_samples) {
      if (vx <= 1e-6) {
        continue;
      }
      for (const double wz : wz_samples) {
        consider_trajectory(vx, wz);
      }
    }
  }

  if (rotating_to_path_ || !found) {
    for (const double wz : wz_samples) {
      if (std::abs(wz) < theta_stopped_velocity_) {
        continue;
      }
      consider_trajectory(0.0, wz);
    }
  }

  if (!found) {
    nav_msgs::msg::Path empty_local;
    empty_local.header.frame_id = costmap_frame;
    empty_local.header.stamp = command.header.stamp;
    publishPlan(empty_local, local_plan_pub_);
    warn_if_slow(
      local_plan.poses.size(), vx_samples.size(), wz_samples.size(),
      trajectory_count, valid_trajectory_count, max_trajectory_poses);
    throw nav2_core::NoValidControl("DWAController failed to find a collision-free trajectory");
  }

  command.twist.linear.x = best.vx;
  command.twist.angular.z = best.wz;
  last_cmd_vx_ = best.vx;
  last_cmd_wz_ = best.wz;

  publishPlan(trajectoryToPath(best, costmap_frame, command.header.stamp), local_plan_pub_);
  warn_if_slow(
    local_plan.poses.size(), vx_samples.size(), wz_samples.size(),
    trajectory_count, valid_trajectory_count, max_trajectory_poses);
  return command;
}

void DWAController::setSpeedLimit(const double & speed_limit, const bool & percentage)
{
  if (speed_limit <= 0.0) {
    speed_limited_ = false;
    speed_limit_ = 0.0;
    return;
  }

  speed_limited_ = true;
  speed_limit_ = percentage ? max_vel_x_ * speed_limit / 100.0 : speed_limit;
}

void DWAController::reset()
{
  last_cmd_vx_ = 0.0;
  last_cmd_wz_ = 0.0;
  rotating_to_path_ = false;
  have_oscillation_reset_pose_ = false;
}

void DWAController::readParameters()
{
  declareParameter("sim_time", rclcpp::ParameterValue(1.3));
  declareParameter("sim_granularity", rclcpp::ParameterValue(0.05));
  declareParameter("angular_sim_granularity", rclcpp::ParameterValue(0.05));
  declareParameter("use_dwa", rclcpp::ParameterValue(true));
  declareParameter("transform_tolerance", rclcpp::ParameterValue(0.5));
  declareParameter("prune_distance", rclcpp::ParameterValue(1.0));
  declareParameter("max_robot_pose_search_dist", rclcpp::ParameterValue(2.0));
  declareParameter("forward_plan_distance", rclcpp::ParameterValue(2.0));
  declareParameter("min_vel_x", rclcpp::ParameterValue(0.0));
  declareParameter("max_vel_x", rclcpp::ParameterValue(0.18));
  declareParameter("max_vel_theta", rclcpp::ParameterValue(0.65));
  declareParameter("min_speed_xy", rclcpp::ParameterValue(0.0));
  declareParameter("acc_lim_x", rclcpp::ParameterValue(0.35));
  declareParameter("decel_lim_x", rclcpp::ParameterValue(-0.45));
  declareParameter("acc_lim_theta", rclcpp::ParameterValue(0.70));
  declareParameter("decel_lim_theta", rclcpp::ParameterValue(-0.90));
  declareParameter("vx_samples", rclcpp::ParameterValue(6));
  declareParameter("vtheta_samples", rclcpp::ParameterValue(15));
  declareParameter("max_local_plan_poses", rclcpp::ParameterValue(60));
  declareParameter("debug_timing", rclcpp::ParameterValue(false));
  declareParameter("timing_warn_ms", rclcpp::ParameterValue(80.0));
  declareParameter("path_distance_bias", rclcpp::ParameterValue(12.0));
  declareParameter("goal_distance_bias", rclcpp::ParameterValue(8.0));
  declareParameter("goal_front_bias", rclcpp::ParameterValue(4.0));
  declareParameter("alignment_bias", rclcpp::ParameterValue(6.0));
  declareParameter("occdist_scale", rclcpp::ParameterValue(0.08));
  declareParameter("twirling_scale", rclcpp::ParameterValue(0.2));
  declareParameter("oscillation_scale", rclcpp::ParameterValue(8.0));
  declareParameter("prefer_forward_scale", rclcpp::ParameterValue(8.0));
  declareParameter("linear_velocity_change_scale", rclcpp::ParameterValue(2.0));
  declareParameter("angular_velocity_change_scale", rclcpp::ParameterValue(0.5));
  declareParameter("forward_point_distance", rclcpp::ParameterValue(0.25));
  declareParameter("path_heading_lookahead", rclcpp::ParameterValue(0.35));
  declareParameter("rotate_to_path_engage_angle", rclcpp::ParameterValue(0.60));
  declareParameter("rotate_to_path_disengage_angle", rclcpp::ParameterValue(0.25));
  declareParameter("obstacle_cost_threshold", rclcpp::ParameterValue(254.0));
  declareParameter("allow_unknown", rclcpp::ParameterValue(false));
  declareParameter("oscillation_reset_dist", rclcpp::ParameterValue(0.08));
  declareParameter("oscillation_reset_angle", rclcpp::ParameterValue(0.25));
  declareParameter("xy_goal_tolerance", rclcpp::ParameterValue(0.10));
  declareParameter("yaw_goal_tolerance", rclcpp::ParameterValue(0.25));
  declareParameter("rotate_to_goal_heading", rclcpp::ParameterValue(false));
  declareParameter("rotate_to_goal_angular_vel", rclcpp::ParameterValue(0.35));
  declareParameter("trans_stopped_velocity", rclcpp::ParameterValue(0.03));
  declareParameter("theta_stopped_velocity", rclcpp::ParameterValue(0.05));
  declareParameter("min_approach_vel_x", rclcpp::ParameterValue(0.03));
  declareParameter("min_dwa_window_vel_x", rclcpp::ParameterValue(0.03));
  declareParameter("approach_slowdown_distance", rclcpp::ParameterValue(0.35));

  if (node_->has_parameter("controller_frequency")) {
    node_->get_parameter("controller_frequency", controller_frequency_);
  }

  sim_time_ = getDouble("sim_time");
  sim_granularity_ = getDouble("sim_granularity");
  angular_sim_granularity_ = getDouble("angular_sim_granularity");
  use_dwa_ = getBool("use_dwa");
  transform_tolerance_ = getDouble("transform_tolerance");
  prune_distance_ = getDouble("prune_distance");
  max_robot_pose_search_dist_ = getDouble("max_robot_pose_search_dist");
  forward_plan_distance_ = getDouble("forward_plan_distance");
  min_vel_x_ = getDouble("min_vel_x");
  max_vel_x_ = getDouble("max_vel_x");
  max_vel_theta_ = getDouble("max_vel_theta");
  min_speed_xy_ = getDouble("min_speed_xy");
  acc_lim_x_ = getDouble("acc_lim_x");
  decel_lim_x_ = getDouble("decel_lim_x");
  acc_lim_theta_ = getDouble("acc_lim_theta");
  decel_lim_theta_ = getDouble("decel_lim_theta");
  vx_samples_ = std::max(1, getInt("vx_samples"));
  vtheta_samples_ = std::max(1, getInt("vtheta_samples"));
  max_local_plan_poses_ = std::max(2, getInt("max_local_plan_poses"));
  debug_timing_ = getBool("debug_timing");
  timing_warn_ms_ = getDouble("timing_warn_ms");
  path_distance_bias_ = getDouble("path_distance_bias");
  goal_distance_bias_ = getDouble("goal_distance_bias");
  goal_front_bias_ = getDouble("goal_front_bias");
  alignment_bias_ = getDouble("alignment_bias");
  occdist_scale_ = getDouble("occdist_scale");
  twirling_scale_ = getDouble("twirling_scale");
  oscillation_scale_ = getDouble("oscillation_scale");
  prefer_forward_scale_ = getDouble("prefer_forward_scale");
  linear_velocity_change_scale_ = getDouble("linear_velocity_change_scale");
  angular_velocity_change_scale_ = getDouble("angular_velocity_change_scale");
  forward_point_distance_ = getDouble("forward_point_distance");
  path_heading_lookahead_ = getDouble("path_heading_lookahead");
  rotate_to_path_engage_angle_ = getDouble("rotate_to_path_engage_angle");
  rotate_to_path_disengage_angle_ = getDouble("rotate_to_path_disengage_angle");
  obstacle_cost_threshold_ = getDouble("obstacle_cost_threshold");
  allow_unknown_ = getBool("allow_unknown");
  oscillation_reset_dist_ = getDouble("oscillation_reset_dist");
  oscillation_reset_angle_ = getDouble("oscillation_reset_angle");
  xy_goal_tolerance_ = getDouble("xy_goal_tolerance");
  yaw_goal_tolerance_ = getDouble("yaw_goal_tolerance");
  rotate_to_goal_heading_ = getBool("rotate_to_goal_heading");
  rotate_to_goal_angular_vel_ = getDouble("rotate_to_goal_angular_vel");
  trans_stopped_velocity_ = getDouble("trans_stopped_velocity");
  theta_stopped_velocity_ = getDouble("theta_stopped_velocity");
  min_approach_vel_x_ = getDouble("min_approach_vel_x");
  min_dwa_window_vel_x_ = getDouble("min_dwa_window_vel_x");
  approach_slowdown_distance_ = getDouble("approach_slowdown_distance");

  sim_time_ = std::max(0.1, sim_time_);
  sim_granularity_ = std::max(0.01, sim_granularity_);
  angular_sim_granularity_ = std::max(0.01, angular_sim_granularity_);
  controller_frequency_ = std::max(1.0, controller_frequency_);
  max_robot_pose_search_dist_ = std::max(0.1, max_robot_pose_search_dist_);
  path_heading_lookahead_ = std::max(0.05, path_heading_lookahead_);
  rotate_to_path_engage_angle_ = std::max(0.01, rotate_to_path_engage_angle_);
  rotate_to_path_disengage_angle_ = clamp(
    rotate_to_path_disengage_angle_, 0.0, rotate_to_path_engage_angle_);
  linear_velocity_change_scale_ = std::max(0.0, linear_velocity_change_scale_);
  angular_velocity_change_scale_ = std::max(0.0, angular_velocity_change_scale_);
  timing_warn_ms_ = std::max(1.0, timing_warn_ms_);
}

void DWAController::declareParameter(
  const std::string & name,
  const rclcpp::ParameterValue & value)
{
  nav2_util::declare_parameter_if_not_declared(node_, plugin_name_ + "." + name, value);
}

double DWAController::getDouble(const std::string & name)
{
  return node_->get_parameter(plugin_name_ + "." + name).as_double();
}

int DWAController::getInt(const std::string & name)
{
  return static_cast<int>(node_->get_parameter(plugin_name_ + "." + name).as_int());
}

bool DWAController::getBool(const std::string & name)
{
  return node_->get_parameter(plugin_name_ + "." + name).as_bool();
}

nav_msgs::msg::Path DWAController::transformGlobalPlan(
  const geometry_msgs::msg::PoseStamped & pose)
{
  const std::string costmap_frame = costmap_ros_->getGlobalFrameID();
  const std::string plan_frame = getPlanFrame();
  geometry_msgs::msg::PoseStamped robot_in_plan_frame;
  try {
    robot_in_plan_frame = transformPose(pose, plan_frame);
  } catch (const tf2::TransformException & ex) {
    throw nav2_core::ControllerTFError(
            std::string("DWAController failed to transform robot pose to plan frame: ") +
            ex.what());
  }

  const double robot_x = robot_in_plan_frame.pose.position.x;
  const double robot_y = robot_in_plan_frame.pose.position.y;

  size_t nearest_index = 0;
  double nearest_dist = std::numeric_limits<double>::infinity();
  double search_distance = 0.0;
  double previous_plan_x = 0.0;
  double previous_plan_y = 0.0;
  for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped plan_pose =
      poseWithFallbackFrame(global_plan_.poses[i], plan_frame);
    if (plan_pose.header.frame_id != plan_frame) {
      try {
        plan_pose = transformPlanPose(plan_pose, plan_frame);
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::ControllerTFError(
                std::string("DWAController failed to transform global plan to plan frame: ") +
                ex.what());
      }
    }

    const double plan_x = plan_pose.pose.position.x;
    const double plan_y = plan_pose.pose.position.y;
    if (i > 0) {
      search_distance += hypot2(plan_x - previous_plan_x, plan_y - previous_plan_y);
      if (search_distance > max_robot_pose_search_dist_) {
        break;
      }
    }
    previous_plan_x = plan_x;
    previous_plan_y = plan_y;

    const double dist = hypot2(
      plan_x - robot_x,
      plan_y - robot_y);
    if (dist < nearest_dist) {
      nearest_dist = dist;
      nearest_index = i;
    }
  }

  if (nearest_index > 1 && nearest_dist < prune_distance_) {
    global_plan_.poses.erase(global_plan_.poses.begin(), global_plan_.poses.begin() + nearest_index);
    nearest_index = 0;
  }

  nav_msgs::msg::Path local_plan;
  local_plan.header.frame_id = costmap_frame;
  local_plan.header.stamp = clock_->now();

  const double max_plan_distance = std::max(0.2, forward_plan_distance_);
  for (size_t i = nearest_index; i < global_plan_.poses.size(); ++i) {
    geometry_msgs::msg::PoseStamped plan_pose =
      poseWithFallbackFrame(global_plan_.poses[i], plan_frame);
    if (plan_pose.header.frame_id != plan_frame) {
      try {
        plan_pose = transformPlanPose(plan_pose, plan_frame);
      } catch (const tf2::TransformException & ex) {
        throw nav2_core::ControllerTFError(
                std::string("DWAController failed to transform global plan to plan frame: ") +
                ex.what());
      }
    }

    const double dist = hypot2(
      plan_pose.pose.position.x - robot_x,
      plan_pose.pose.position.y - robot_y);
    if (dist > max_plan_distance && !local_plan.poses.empty()) {
      break;
    }

    geometry_msgs::msg::PoseStamped costmap_pose;
    try {
      costmap_pose = transformPlanPose(plan_pose, costmap_frame);
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(
              std::string("DWAController failed to transform local plan to costmap frame: ") +
              ex.what());
    }

    unsigned int mx = 0;
    unsigned int my = 0;
    const bool in_costmap = costmap_->worldToMap(
      costmap_pose.pose.position.x, costmap_pose.pose.position.y, mx, my);
    if (!in_costmap && !local_plan.poses.empty()) {
      break;
    }
    local_plan.poses.push_back(costmap_pose);

    if (local_plan.poses.size() >= static_cast<size_t>(max_local_plan_poses_)) {
      break;
    }
  }

  if (local_plan.poses.empty()) {
    try {
      const auto nearest_pose =
        poseWithFallbackFrame(global_plan_.poses[nearest_index], plan_frame);
      local_plan.poses.push_back(transformPlanPose(nearest_pose, costmap_frame));
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(
              std::string("DWAController failed to transform nearest plan pose: ") +
              ex.what());
    }
  }
  if (local_plan.poses.size() == 1 && global_plan_.poses.size() > nearest_index + 1) {
    try {
      const auto next_pose =
        poseWithFallbackFrame(global_plan_.poses[nearest_index + 1], plan_frame);
      local_plan.poses.push_back(transformPlanPose(next_pose, costmap_frame));
    } catch (const tf2::TransformException & ex) {
      throw nav2_core::ControllerTFError(
              std::string("DWAController failed to transform next plan pose: ") +
              ex.what());
    }
  }

  return local_plan;
}

geometry_msgs::msg::PoseStamped DWAController::transformPlanPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame) const
{
  auto latest_pose = pose;
  latest_pose.header.stamp.sec = 0;
  latest_pose.header.stamp.nanosec = 0;
  return transformPose(latest_pose, target_frame);
}

geometry_msgs::msg::PoseStamped DWAController::poseWithFallbackFrame(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & fallback_frame) const
{
  auto framed_pose = pose;
  if (framed_pose.header.frame_id.empty()) {
    framed_pose.header.frame_id = fallback_frame;
  }
  return framed_pose;
}

std::string DWAController::getPlanFrame() const
{
  if (!global_plan_.header.frame_id.empty()) {
    return global_plan_.header.frame_id;
  }
  for (const auto & pose : global_plan_.poses) {
    if (!pose.header.frame_id.empty()) {
      return pose.header.frame_id;
    }
  }
  return costmap_ros_->getGlobalFrameID();
}

geometry_msgs::msg::PoseStamped DWAController::transformPose(
  const geometry_msgs::msg::PoseStamped & pose,
  const std::string & target_frame) const
{
  if (pose.header.frame_id == target_frame) {
    return pose;
  }

  try {
    return tf_->transform(
      pose, target_frame, tf2::durationFromSec(transform_tolerance_));
  } catch (const tf2::ExtrapolationException &) {
    auto latest_pose = pose;
    latest_pose.header.stamp.sec = 0;
    latest_pose.header.stamp.nanosec = 0;
    return tf_->transform(
      latest_pose, target_frame, tf2::durationFromSec(transform_tolerance_));
  }
}

std::vector<double> DWAController::sampleRange(
  double min_value,
  double max_value,
  int samples) const
{
  if (samples <= 1 || std::abs(max_value - min_value) < 1e-9) {
    return {clamp(0.0, min_value, max_value)};
  }

  std::vector<double> values;
  values.reserve(static_cast<size_t>(samples) + 1);
  const double step = (max_value - min_value) / static_cast<double>(samples - 1);
  for (int i = 0; i < samples; ++i) {
    values.push_back(min_value + step * static_cast<double>(i));
  }
  if (min_value <= 0.0 && max_value >= 0.0) {
    values.push_back(0.0);
  }
  std::sort(values.begin(), values.end());
  values.erase(std::unique(values.begin(), values.end(), [](double a, double b) {
      return std::abs(a - b) < 1e-6;
    }), values.end());
  return values;
}

std::vector<double> DWAController::sampleLinearVelocities(double current_vx) const
{
  double effective_max_vel_x = max_vel_x_;
  if (speed_limited_) {
    effective_max_vel_x = std::min(effective_max_vel_x, std::max(0.0, speed_limit_));
  }

  double lower = min_vel_x_;
  double upper = effective_max_vel_x;
  if (use_dwa_) {
    const double period = 1.0 / controller_frequency_;
    lower = std::max(lower, current_vx + decel_lim_x_ * period);
    upper = std::min(upper, current_vx + acc_lim_x_ * period);
  }
  if (lower > upper) {
    lower = upper;
  }

  auto samples = sampleRange(lower, upper, vx_samples_);
  if (min_speed_xy_ > 0.0) {
    samples.erase(std::remove_if(samples.begin(), samples.end(), [this](double value) {
        return std::abs(value) > 1e-6 && std::abs(value) < min_speed_xy_;
      }), samples.end());
    if (samples.empty()) {
      samples.push_back(0.0);
    }
  }
  return samples;
}

std::vector<double> DWAController::sampleAngularVelocities(double current_wz) const
{
  double lower = -max_vel_theta_;
  double upper = max_vel_theta_;
  if (use_dwa_) {
    const double period = 1.0 / controller_frequency_;
    const double decel = std::abs(decel_lim_theta_);
    lower = std::max(lower, current_wz - decel * period);
    upper = std::min(upper, current_wz + acc_lim_theta_ * period);
  }
  if (lower > upper) {
    lower = upper;
  }
  return sampleRange(lower, upper, vtheta_samples_);
}

bool DWAController::makeTrajectory(
  const geometry_msgs::msg::PoseStamped & pose,
  double vx,
  double wz,
  Trajectory & trajectory) const
{
  trajectory.vx = vx;
  trajectory.wz = wz;

  geometry_msgs::msg::Pose2D current;
  current.x = pose.pose.position.x;
  current.y = pose.pose.position.y;
  current.theta = yawFromPose(pose);
  trajectory.poses.push_back(current);

  const double linear_step_time = sim_granularity_ / std::max(std::abs(vx), 0.01);
  const double angular_step_time = angular_sim_granularity_ / std::max(std::abs(wz), 0.01);
  const double dt = clamp(std::min({linear_step_time, angular_step_time, 0.10}), 0.02, 0.10);
  const int steps = std::max(1, static_cast<int>(std::ceil(sim_time_ / dt)));
  const double step_dt = sim_time_ / static_cast<double>(steps);

  for (int i = 0; i < steps; ++i) {
    current.x += vx * std::cos(current.theta) * step_dt;
    current.y += vx * std::sin(current.theta) * step_dt;
    current.theta = angles::normalize_angle(current.theta + wz * step_dt);
    trajectory.poses.push_back(current);
  }

  return !trajectory.poses.empty();
}

bool DWAController::scoreTrajectory(
  Trajectory & trajectory,
  const nav_msgs::msg::Path & local_plan,
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const geometry_msgs::msg::PoseStamped & goal_pose) const
{
  double max_obstacle_cost = 0.0;
  for (const auto & pose : trajectory.poses) {
    double cost = 0.0;
    if (!isCollisionFree(pose, cost)) {
      return false;
    }
    max_obstacle_cost = std::max(max_obstacle_cost, normalizeCostmapCost(cost));
  }

  const auto & final_pose = trajectory.poses.back();
  const double nose_x = final_pose.x + forward_point_distance_ * std::cos(final_pose.theta);
  const double nose_y = final_pose.y + forward_point_distance_ * std::sin(final_pose.theta);

  trajectory.path_cost = distanceToPath(final_pose.x, final_pose.y, local_plan);
  trajectory.goal_cost = distanceToPose(final_pose.x, final_pose.y, goal_pose);
  const double robot_goal_dist = distanceToPose(
    robot_pose.pose.position.x, robot_pose.pose.position.y, goal_pose);

  trajectory.goal_front_cost =
    robot_goal_dist > forward_point_distance_
        ? distanceToPose(nose_x, nose_y, goal_pose)
        : 0.0;
  trajectory.alignment_cost =
    robot_goal_dist > forward_point_distance_ ? distanceToPath(nose_x, nose_y, local_plan) : 0.0;

  trajectory.obstacle_cost = max_obstacle_cost;
  trajectory.twirling_cost = std::abs(trajectory.wz);

  const bool angular_flip =
    std::abs(last_cmd_wz_) > theta_stopped_velocity_ &&
    std::abs(trajectory.wz) > theta_stopped_velocity_ &&
    last_cmd_wz_ * trajectory.wz < 0.0;
  const bool linear_flip =
    std::abs(last_cmd_vx_) > trans_stopped_velocity_ &&
    std::abs(trajectory.vx) > trans_stopped_velocity_ &&
    last_cmd_vx_ * trajectory.vx < 0.0;
  trajectory.oscillation_cost = (angular_flip || linear_flip) ? 1.0 : 0.0;
  trajectory.linear_velocity_change_cost = std::abs(trajectory.vx - last_cmd_vx_);
  trajectory.angular_velocity_change_cost = std::abs(trajectory.wz - last_cmd_wz_);

  const double prefer_forward_cost = trajectory.vx < 0.0 ? std::abs(trajectory.vx) : 0.0;

  trajectory.total_cost =
    path_distance_bias_ * trajectory.path_cost +
    goal_distance_bias_ * trajectory.goal_cost +
    goal_front_bias_ * trajectory.goal_front_cost +
    alignment_bias_ * trajectory.alignment_cost +
    occdist_scale_ * trajectory.obstacle_cost +
    twirling_scale_ * trajectory.twirling_cost +
    oscillation_scale_ * trajectory.oscillation_cost +
    linear_velocity_change_scale_ * trajectory.linear_velocity_change_cost +
    angular_velocity_change_scale_ * trajectory.angular_velocity_change_cost +
    prefer_forward_scale_ * prefer_forward_cost;

  return std::isfinite(trajectory.total_cost);
}

bool DWAController::isCollisionFree(
  const geometry_msgs::msg::Pose2D & pose,
  double & cost) const
{
  cost = collision_checker_->footprintCostAtPose(
    pose.x, pose.y, pose.theta, costmap_ros_->getRobotFootprint());

  if (cost < 0.0) {
    return false;
  }
  if (!allow_unknown_ && cost >= nav2_costmap_2d::NO_INFORMATION) {
    return false;
  }
  return cost < obstacle_cost_threshold_;
}

double DWAController::distanceToPath(
  double x,
  double y,
  const nav_msgs::msg::Path & path) const
{
  double best = std::numeric_limits<double>::infinity();
  for (const auto & pose : path.poses) {
    best = std::min(best, hypot2(x - pose.pose.position.x, y - pose.pose.position.y));
  }
  return best;
}

double DWAController::distanceToPose(
  double x,
  double y,
  const geometry_msgs::msg::PoseStamped & pose) const
{
  return hypot2(x - pose.pose.position.x, y - pose.pose.position.y);
}

double DWAController::yawFromPose(const geometry_msgs::msg::PoseStamped & pose) const
{
  return tf2::getYaw(pose.pose.orientation);
}

double DWAController::pathHeadingError(
  const geometry_msgs::msg::PoseStamped & robot_pose,
  const nav_msgs::msg::Path & local_plan) const
{
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;
  const geometry_msgs::msg::PoseStamped * target = &local_plan.poses.back();
  for (const auto & plan_pose : local_plan.poses) {
    if (hypot2(
        plan_pose.pose.position.x - robot_x,
        plan_pose.pose.position.y - robot_y) >= path_heading_lookahead_)
    {
      target = &plan_pose;
      break;
    }
  }

  const double dx = target->pose.position.x - robot_x;
  const double dy = target->pose.position.y - robot_y;
  if (hypot2(dx, dy) < 1e-6) {
    return 0.0;
  }
  return angles::shortest_angular_distance(yawFromPose(robot_pose), std::atan2(dy, dx));
}

double DWAController::normalizeCostmapCost(double cost) const
{
  if (cost <= 0.0) {
    return 0.0;
  }
  return std::min(cost, static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)) /
         static_cast<double>(nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

double DWAController::getGoalPositionTolerance(nav2_core::GoalChecker * goal_checker) const
{
  if (goal_checker == nullptr) {
    return xy_goal_tolerance_;
  }

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    const double tolerance = std::max(pose_tolerance.position.x, pose_tolerance.position.y);
    if (std::isfinite(tolerance) && tolerance > 0.0) {
      return tolerance;
    }
  }
  return xy_goal_tolerance_;
}

double DWAController::getGoalYawTolerance(nav2_core::GoalChecker * goal_checker) const
{
  if (goal_checker == nullptr) {
    return yaw_goal_tolerance_;
  }

  geometry_msgs::msg::Pose pose_tolerance;
  geometry_msgs::msg::Twist velocity_tolerance;
  if (goal_checker->getTolerances(pose_tolerance, velocity_tolerance)) {
    const double yaw_tolerance = tf2::getYaw(pose_tolerance.orientation);
    if (std::isfinite(yaw_tolerance) && yaw_tolerance > 0.0) {
      return yaw_tolerance;
    }
  }
  return yaw_goal_tolerance_;
}

bool DWAController::shouldRotateToGoal(
  const geometry_msgs::msg::PoseStamped & pose,
  const geometry_msgs::msg::Twist & velocity,
  const geometry_msgs::msg::PoseStamped & goal_pose,
  nav2_core::GoalChecker * goal_checker,
  geometry_msgs::msg::TwistStamped & command)
{
  const double xy_tolerance = getGoalPositionTolerance(goal_checker);
  const double goal_distance = distanceToPose(
    pose.pose.position.x, pose.pose.position.y, goal_pose);
  if (goal_distance > xy_tolerance) {
    return false;
  }

  if (goal_checker != nullptr &&
    goal_checker->isGoalReached(pose.pose, goal_pose.pose, velocity))
  {
    command.twist.linear.x = 0.0;
    command.twist.angular.z = 0.0;
    return true;
  }

  if (!rotate_to_goal_heading_) {
    command.twist.linear.x = 0.0;
    command.twist.angular.z = 0.0;
    return true;
  }

  const double yaw_tolerance = getGoalYawTolerance(goal_checker);
  const double yaw_error = angles::shortest_angular_distance(yawFromPose(pose), yawFromPose(goal_pose));
  if (std::abs(yaw_error) <= yaw_tolerance &&
    std::abs(velocity.linear.x) <= trans_stopped_velocity_ &&
    std::abs(velocity.angular.z) <= theta_stopped_velocity_)
  {
    command.twist.linear.x = 0.0;
    command.twist.angular.z = 0.0;
    return true;
  }

  command.twist.linear.x = 0.0;
  const double raw_wz = clamp(
    std::abs(yaw_error) * 1.2,
    theta_stopped_velocity_,
    rotate_to_goal_angular_vel_);
  command.twist.angular.z = std::copysign(raw_wz, yaw_error);
  return true;
}

void DWAController::publishPlan(
  const nav_msgs::msg::Path & path,
  const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr & publisher) const
{
  if (active_ && publisher) {
    publisher->publish(path);
  }
}

nav_msgs::msg::Path DWAController::trajectoryToPath(
  const Trajectory & trajectory,
  const std::string & frame_id,
  const rclcpp::Time & stamp) const
{
  nav_msgs::msg::Path path;
  path.header.frame_id = frame_id;
  path.header.stamp = stamp;
  path.poses.reserve(trajectory.poses.size());
  for (const auto & pose_2d : trajectory.poses) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = pose_2d.x;
    pose.pose.position.y = pose_2d.y;
    pose.pose.orientation = yawToQuaternion(pose_2d.theta);
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace inspection_robot_dwa_controller

PLUGINLIB_EXPORT_CLASS(
  inspection_robot_dwa_controller::DWAController,
  nav2_core::Controller)
