#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <cmath>
#include <mutex>
#include <optional>
#include <vector>
#include <stdexcept>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using GripperGoalHandle =
  rclcpp_action::ClientGoalHandle<GripperCommand>;

using MoveGroup =
  moveit::planning_interface::MoveGroupInterface;
using DisplayTrajectory = moveit_msgs::msg::DisplayTrajectory;

struct ObjectPoseCapture
{
  std::mutex mutex;
  std::condition_variable condition;
  std::optional<geometry_msgs::msg::PoseStamped> pose;
};

struct GripperCalibrationSolution
{
  double command_rad;
  double closure_drop_m;
  double bracket_low_rad;
  double bracket_high_rad;
};


std::optional<GripperCalibrationSolution> solve_gripper_calibration(
  double object_width_mm,
  double total_compression_mm)
{
  struct Sample
  {
    double command_rad;
    double estimated_pad_gap_m;
    double closure_drop_m;
  };

  static constexpr std::array<Sample, 6> samples = {{
    {0.00, 0.0850000000, 0.0000000000},
    {0.10, 0.0759588652, 0.0034926322},
    {0.20, 0.0662655355, 0.0065165122},
    {0.30, 0.0560168635, 0.0090414263},
    {0.35, 0.0507160854, 0.0101086854},
    {0.40, 0.0453152505, 0.0110421465},
  }};

  const double target_gap_m =
    (object_width_mm - total_compression_mm) / 1000.0;
  for (std::size_t index = 0; index + 1 < samples.size(); ++index)
  {
    const auto & upper_gap = samples[index];
    const auto & lower_gap = samples[index + 1];
    if (upper_gap.estimated_pad_gap_m >= target_gap_m &&
      target_gap_m >= lower_gap.estimated_pad_gap_m)
    {
      const double fraction =
        (upper_gap.estimated_pad_gap_m - target_gap_m) /
        (upper_gap.estimated_pad_gap_m - lower_gap.estimated_pad_gap_m);
      return GripperCalibrationSolution{
        upper_gap.command_rad + fraction *
        (lower_gap.command_rad - upper_gap.command_rad),
        upper_gap.closure_drop_m + fraction *
        (lower_gap.closure_drop_m - upper_gap.closure_drop_m),
        upper_gap.command_rad,
        lower_gap.command_rad};
    }
  }
  return std::nullopt;
}


bool command_gripper(
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  const rclcpp::Logger & logger,
  double position,
  double max_effort)
{
  RCLCPP_INFO(
    logger,
    "Waiting for gripper action server..."
  );

  if (!client->wait_for_action_server(10s))
  {
    RCLCPP_ERROR(
      logger,
      "Gripper action server is not available."
    );
    return false;
  }

  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort;

  RCLCPP_INFO(
    logger,
    "Sending gripper command: position=%.4f",
    position
  );

  auto goal_future = client->async_send_goal(goal);

  if (goal_future.wait_for(10s) != std::future_status::ready)
  {
    RCLCPP_ERROR(logger, "Timed out sending gripper goal.");
    return false;
  }

  auto goal_handle = goal_future.get();

  if (!goal_handle)
  {
    RCLCPP_ERROR(logger, "Gripper goal was rejected.");
    return false;
  }

  auto result_future = client->async_get_result(goal_handle);

  if (result_future.wait_for(20s) != std::future_status::ready)
  {
    RCLCPP_ERROR(logger, "Timed out waiting for gripper result.");
    return false;
  }

  const auto wrapped_result = result_future.get();

  if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED)
  {
    RCLCPP_ERROR(logger, "Gripper action did not succeed.");
    return false;
  }

  if (!wrapped_result.result->reached_goal)
  {
    RCLCPP_ERROR(
      logger,
      "Gripper did not reach its goal. stalled=%s",
      wrapped_result.result->stalled ? "true" : "false"
    );
    return false;
  }

  RCLCPP_INFO(
    logger,
    "Gripper reached position %.4f",
    wrapped_result.result->position
  );

  return true;
}


bool move_to_named_target(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  const std::string & target_name)
{
  RCLCPP_INFO(
    logger,
    "Moving UR5 to named target: %s",
    target_name.c_str()
  );

  move_group.setStartStateToCurrentState();

  if (!move_group.setNamedTarget(target_name))
  {
    RCLCPP_ERROR(
      logger,
      "Named target '%s' was not found.",
      target_name.c_str()
    );
    return false;
  }

  MoveGroup::Plan plan;

  const auto planning_result = move_group.plan(plan);

  if (!static_cast<bool>(planning_result))
  {
    RCLCPP_ERROR(
      logger,
      "Planning to '%s' failed.",
      target_name.c_str()
    );
    RCLCPP_ERROR(logger, "MoveIt error code: %d", planning_result.val);
    return false;
  }

  if (!static_cast<bool>(move_group.execute(plan)))
  {
    RCLCPP_ERROR(
      logger,
      "Execution to '%s' failed.",
      target_name.c_str()
    );
    return false;
  }

  RCLCPP_INFO(
    logger,
    "UR5 reached '%s'.",
    target_name.c_str()
  );

  std::this_thread::sleep_for(500ms);
  return true;
}


bool move_relative(
  MoveGroup & move_group,
  const rclcpp::Publisher<DisplayTrajectory>::SharedPtr & display_publisher,
  const rclcpp::Logger & logger,
  double dx,
  double dy,
  double dz,
  const std::string & stage_name,
  double velocity_scaling,
  bool execute_motion = true)
{
  const auto current_state = move_group.getCurrentState(10.0);
  if (!current_state)
  {
    RCLCPP_ERROR(
      logger,
      "%s: cannot obtain current robot state.",
      stage_name.c_str()
    );
    return false;
  }

  const auto current_pose =
    move_group.getCurrentPose().pose;

  auto target_pose = current_pose;

  target_pose.position.x += dx;
  target_pose.position.y += dy;
  target_pose.position.z += dz;

  RCLCPP_INFO(
    logger,
    "%s Cartesian target: X=%.3f Y=%.3f Z=%.3f",
    stage_name.c_str(),
    target_pose.position.x,
    target_pose.position.y,
    target_pose.position.z
  );

  move_group.setStartStateToCurrentState();

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(target_pose);

  moveit_msgs::msg::RobotTrajectory trajectory;

  const double fraction =
    move_group.computeCartesianPath(
      waypoints,
      0.005,   // 每 5 mm 計算一次
      0.0,     // 關閉相對門檻；下方仍有 1 rad 絕對保護
      trajectory,
      true     // 開啟碰撞檢查
    );

  RCLCPP_INFO(
    logger,
    "%s Cartesian path completed: %.1f%%",
    stage_name.c_str(),
    fraction * 100.0
  );

  if (fraction < 0.99)
  {
    RCLCPP_ERROR(
      logger,
      "%s: Cartesian path is incomplete; refusing execution.",
      stage_name.c_str()
    );
    return false;
  }

  const auto & points =
    trajectory.joint_trajectory.points;

  if (points.empty())
  {
    RCLCPP_ERROR(
      logger,
      "%s: generated trajectory is empty.",
      stage_name.c_str()
    );
    return false;
  }

  const auto & first = points.front().positions;
  const auto & last = points.back().positions;
  const auto & names = trajectory.joint_trajectory.joint_names;

  if (first.size() != last.size())
  {
    RCLCPP_ERROR(
      logger,
      "%s: invalid trajectory dimensions.",
      stage_name.c_str()
    );
    return false;
  }

  // 小幅 Cartesian 動作不應造成任何關節大角度旋轉。
  for (std::size_t joint = 0; joint < first.size(); ++joint)
  {
    const double travel =
      std::abs(last[joint] - first[joint]);

    if (travel > 1.0)
    {
      const char * joint_name =
        joint < names.size() ?
        names[joint].c_str() :
        "unknown_joint";

      RCLCPP_ERROR(
        logger,
        "%s: unsafe joint travel detected: %s moves %.3f rad. "
        "Trajectory rejected.",
        stage_name.c_str(),
        joint_name,
        travel
      );
      return false;
    }
  }

  robot_trajectory::RobotTrajectory timed_trajectory(
    move_group.getRobotModel(), move_group.getName());
  timed_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
  if (!time_parameterization.computeTimeStamps(
      timed_trajectory, velocity_scaling, velocity_scaling))
  {
    RCLCPP_ERROR(
      logger,
      "%s: Cartesian time parameterization failed.",
      stage_name.c_str());
    return false;
  }
  timed_trajectory.getRobotTrajectoryMsg(trajectory);

  const auto & duration = trajectory.joint_trajectory.points.back().time_from_start;
  const double duration_seconds =
    static_cast<double>(duration.sec) +
    static_cast<double>(duration.nanosec) * 1.0e-9;
  RCLCPP_INFO(
    logger,
    "%s Cartesian timing: duration=%.3f s velocity_scale=%.3f",
    stage_name.c_str(), duration_seconds, velocity_scaling);

  MoveGroup::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*current_state, plan.start_state);
  plan.trajectory = trajectory;

  if (!execute_motion)
  {
    DisplayTrajectory display;
    display.model_id = move_group.getRobotModel()->getName();
    display.trajectory_start = plan.start_state;
    display.trajectory.push_back(plan.trajectory);
    display_publisher->publish(display);
    RCLCPP_INFO(
      logger,
      "%s PLAN_ONLY_SUCCEEDED; preview published and no robot command was sent.",
      stage_name.c_str());
    std::this_thread::sleep_for(500ms);
    return true;
  }

  const bool executed =
    static_cast<bool>(move_group.execute(plan));

  if (!executed)
  {
    RCLCPP_ERROR(
      logger,
      "%s Cartesian execution failed.",
      stage_name.c_str()
    );
    return false;
  }

  RCLCPP_INFO(
    logger,
    "%s Cartesian motion completed.",
    stage_name.c_str()
  );

  std::this_thread::sleep_for(500ms);
  return true;
}

std::array<double, 3> rotate_local_z(
  const geometry_msgs::msg::Quaternion & quaternion,
  double distance)
{
  const double norm = std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
  if (norm < 1.0e-9)
  {
    throw std::runtime_error("Target orientation quaternion is invalid.");
  }

  const double x = quaternion.x / norm;
  const double y = quaternion.y / norm;
  const double z = quaternion.z / norm;
  const double w = quaternion.w / norm;

  // Third column of the quaternion rotation matrix: local +Z in world.
  return {
    distance * 2.0 * (x * z + w * y),
    distance * 2.0 * (y * z - w * x),
    distance * (1.0 - 2.0 * (x * x + y * y))
  };
}


geometry_msgs::msg::Quaternion quaternion_from_rpy_degrees(
  double roll_degrees,
  double pitch_degrees,
  double yaw_degrees)
{
  constexpr double degrees_to_radians = M_PI / 180.0;
  const double roll = roll_degrees * degrees_to_radians;
  const double pitch = pitch_degrees * degrees_to_radians;
  const double yaw = yaw_degrees * degrees_to_radians;
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);

  geometry_msgs::msg::Quaternion result;
  result.w = cr * cp * cy + sr * sp * sy;
  result.x = sr * cp * cy - cr * sp * sy;
  result.y = cr * sp * cy + sr * cp * sy;
  result.z = cr * cp * sy - sr * sp * cy;
  return result;
}


bool move_staged_cartesian_to_pose(
  MoveGroup & move_group,
  const rclcpp::Publisher<DisplayTrajectory>::SharedPtr & display_publisher,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target,
  double safe_transit_margin,
  double maximum_joint_travel,
  double maximum_wrist_3_travel,
  double velocity_scaling,
  bool execute_motion)
{
  const auto current_state = move_group.getCurrentState(10.0);
  if (!current_state)
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp: cannot obtain current robot state.");
    return false;
  }

  const auto current_pose = move_group.getCurrentPose("tool0").pose;
  const double safe_z =
    std::max(current_pose.position.z, target.position.z) + safe_transit_margin;

  auto raised_pose = current_pose;
  raised_pose.position.z = safe_z;

  auto oriented_pose = raised_pose;
  oriented_pose.orientation = target.orientation;

  auto above_target_pose = target;
  above_target_pose.position.z = safe_z;

  const std::vector<geometry_msgs::msg::Pose> waypoints = {
    raised_pose,
    oriented_pose,
    above_target_pose,
    target,
  };

  RCLCPP_INFO(
    logger,
    "SAFE_PREGRASP_ROUTE current=(%.3f, %.3f, %.3f) safe_z=%.3f "
    "target=(%.3f, %.3f, %.3f)",
    current_pose.position.x, current_pose.position.y, current_pose.position.z,
    safe_z, target.position.x, target.position.y, target.position.z);

  static constexpr std::array<const char *, 4> stage_names = {
    "raise", "orient", "translate", "descend"};
  for (std::size_t stage = 0; stage < waypoints.size(); ++stage)
  {
    move_group.setStartStateToCurrentState();
    moveit_msgs::msg::RobotTrajectory diagnostic_trajectory;
    const std::vector<geometry_msgs::msg::Pose> prefix(
      waypoints.begin(), waypoints.begin() + stage + 1);
    const double diagnostic_fraction = move_group.computeCartesianPath(
      prefix, 0.005, 0.0, diagnostic_trajectory, true);
    RCLCPP_INFO(
      logger, "SAFE_PREGRASP_STAGE_FRACTION stage=%s prefix=%.1f%%",
      stage_names[stage], diagnostic_fraction * 100.0);
  }

  move_group.setStartStateToCurrentState();
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double fraction = move_group.computeCartesianPath(
    waypoints, 0.005, 0.0, trajectory, true);

  RCLCPP_INFO(
    logger, "Safe pre-grasp Cartesian path completed: %.1f%%",
    fraction * 100.0);
  if (fraction < 0.99)
  {
    RCLCPP_ERROR(
      logger,
      "Safe pre-grasp path is incomplete; refusing preview or execution.");
    return false;
  }

  const auto & points = trajectory.joint_trajectory.points;
  const auto & names = trajectory.joint_trajectory.joint_names;
  if (points.size() < 2)
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp trajectory has too few points.");
    return false;
  }

  const std::size_t joint_count = points.front().positions.size();
  std::vector<double> cumulative_travel(joint_count, 0.0);
  std::vector<double> maximum_step(joint_count, 0.0);
  for (std::size_t point = 1; point < points.size(); ++point)
  {
    if (points[point].positions.size() != joint_count ||
      points[point - 1].positions.size() != joint_count)
    {
      RCLCPP_ERROR(logger, "Safe pre-grasp trajectory dimensions are invalid.");
      return false;
    }
    for (std::size_t joint = 0; joint < joint_count; ++joint)
    {
      const double step = std::abs(
        points[point].positions[joint] -
        points[point - 1].positions[joint]);
      cumulative_travel[joint] += step;
      maximum_step[joint] = std::max(maximum_step[joint], step);
    }
  }

  for (std::size_t joint = 0; joint < joint_count; ++joint)
  {
    const char * joint_name =
      joint < names.size() ? names[joint].c_str() : "unknown_joint";
    const bool is_wrist_3 = std::string(joint_name) == "wrist_3_joint";
    const double allowed_travel =
      is_wrist_3 ? maximum_wrist_3_travel : maximum_joint_travel;
    RCLCPP_INFO(
      logger,
      "SAFE_PREGRASP_JOINT_TRAVEL joint=%s cumulative=%.3f rad "
      "limit=%.3f rad max_step=%.3f rad",
      joint_name, cumulative_travel[joint], allowed_travel, maximum_step[joint]);
    if (cumulative_travel[joint] > allowed_travel ||
      maximum_step[joint] > 0.35)
    {
      RCLCPP_ERROR(
        logger,
        "Safe pre-grasp rejected: %s cumulative=%.3f rad max_step=%.3f rad.",
        joint_name, cumulative_travel[joint], maximum_step[joint]);
      return false;
    }
  }

  robot_trajectory::RobotTrajectory timed_trajectory(
    move_group.getRobotModel(), move_group.getName());
  timed_trajectory.setRobotTrajectoryMsg(*current_state, trajectory);
  trajectory_processing::TimeOptimalTrajectoryGeneration time_parameterization;
  if (!time_parameterization.computeTimeStamps(
      timed_trajectory, velocity_scaling, velocity_scaling))
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp time parameterization failed.");
    return false;
  }
  timed_trajectory.getRobotTrajectoryMsg(trajectory);

  MoveGroup::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*current_state, plan.start_state);
  plan.trajectory = trajectory;

  if (!execute_motion)
  {
    DisplayTrajectory display;
    display.model_id = move_group.getRobotModel()->getName();
    display.trajectory_start = plan.start_state;
    display.trajectory.push_back(plan.trajectory);
    display_publisher->publish(display);
    RCLCPP_INFO(
      logger,
      "SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED; four-stage preview published; "
      "no robot command was sent.");
    std::this_thread::sleep_for(500ms);
    return true;
  }

  if (!static_cast<bool>(move_group.execute(plan)))
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp execution failed.");
    return false;
  }
  RCLCPP_INFO(logger, "Safe staged pre-grasp completed.");
  std::this_thread::sleep_for(500ms);
  return true;
}


bool add_floor_collision(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  double floor_z,
  double floor_size,
  double floor_thickness)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  moveit_msgs::msg::CollisionObject floor;
  floor.header.frame_id = move_group.getPlanningFrame();
  floor.id = "floor";

  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {floor_size, floor_size, floor_thickness};

  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  // Keep the collision top 1 mm below z=0 to avoid numerical contact
  // between the fixed robot base and the floor.
  pose.position.z = floor_z - 0.001 - floor_thickness * 0.5;

  floor.primitives.push_back(primitive);
  floor.primitive_poses.push_back(pose);
  floor.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (!planning_scene.applyCollisionObject(floor))
  {
    RCLCPP_ERROR(logger, "Failed to add floor to the MoveIt planning scene.");
    return false;
  }

  RCLCPP_INFO(
    logger,
    "PLANNING_SCENE_FLOOR_ADDED size=%.2f m top_z=%.4f m",
    floor_size, floor_z - 0.001);
  return true;
}


bool move_to_pose_target(
  MoveGroup & move_group,
  const rclcpp::Publisher<DisplayTrajectory>::SharedPtr & display_publisher,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name,
  bool execute_motion = true)
{
  move_group.setStartStateToCurrentState();
  move_group.setPoseTarget(target, "tool0");

  RCLCPP_INFO(
    logger,
    "%s tool0 target: X=%.4f Y=%.4f Z=%.4f",
    stage_name.c_str(), target.position.x, target.position.y, target.position.z);

  MoveGroup::Plan plan;
  const auto planning_result = move_group.plan(plan);
  move_group.clearPoseTargets();

  if (!static_cast<bool>(planning_result))
  {
    RCLCPP_ERROR(logger, "%s planning failed.", stage_name.c_str());
    return false;
  }

  if (!execute_motion)
  {
    DisplayTrajectory display;
    display.model_id = move_group.getRobotModel()->getName();
    display.trajectory_start = plan.start_state;
    display.trajectory.push_back(plan.trajectory);
    display_publisher->publish(display);
    RCLCPP_INFO(
      logger,
      "%s PLAN_ONLY_SUCCEEDED; preview published and no robot command was sent.",
      stage_name.c_str());
    std::this_thread::sleep_for(500ms);
    return true;
  }

  if (!static_cast<bool>(move_group.execute(plan)))
  {
    RCLCPP_ERROR(logger, "%s execution failed.", stage_name.c_str());
    return false;
  }

  RCLCPP_INFO(logger, "%s completed.", stage_name.c_str());
  std::this_thread::sleep_for(500ms);
  return true;
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pick_at_position",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto logger = node->get_logger();

  auto parameter = [&node](const std::string & name, double default_value) {
      if (!node->has_parameter(name))
      {
        node->declare_parameter<double>(name, default_value);
      }
      return node->get_parameter(name).as_double();
    };
  auto bool_parameter = [&node](const std::string & name, bool default_value) {
      if (!node->has_parameter(name))
      {
        node->declare_parameter<bool>(name, default_value);
      }
      return node->get_parameter(name).as_bool();
    };
  auto string_parameter =
    [&node](const std::string & name, const std::string & default_value) {
      if (!node->has_parameter(name))
      {
        node->declare_parameter<std::string>(name, default_value);
      }
      return node->get_parameter(name).as_string();
    };

  double object_x = parameter("object_x", 0.4869);
  double object_y = parameter("object_y", 0.10915);
  double object_z = parameter("object_z", 0.0100);
  const double pregrasp_clearance = parameter("pregrasp_clearance", 0.1500);
  double approach_distance = parameter("approach_distance", 0.1000);
  const double lift_distance = parameter("lift_distance", 0.1200);
  const double grasp_center_offset = parameter("grasp_center_offset", 0.1090);
  double closed_grip = parameter("closed_grip", 0.3760);
  const double velocity_scale = parameter("velocity_scale", 0.15);
  const double lift_velocity_scale = parameter("lift_velocity_scale", 0.03);
  const bool use_newton_object_pose =
    bool_parameter("use_newton_object_pose", true);
  const bool pose_only = bool_parameter("pose_only", false);
  const std::string object_pose_topic =
    string_parameter("object_pose_topic", "/newton/object_pose");
  const double object_pose_timeout = parameter("object_pose_timeout", 5.0);
  const double grasp_roll_deg = parameter("grasp_roll_deg", 180.0);
  const double grasp_pitch_deg = parameter("grasp_pitch_deg", 0.0);
  const double grasp_yaw_deg = parameter("grasp_yaw_deg", 90.0);
  const double floor_z = parameter("floor_z", 0.0);
  const double floor_size = parameter("floor_size", 3.0);
  const double floor_thickness = parameter("floor_thickness", 0.02);
  const bool plan_only = bool_parameter("plan_only", false);
  const bool use_safe_pregrasp_path =
    bool_parameter("use_safe_pregrasp_path", true);
  const double safe_transit_margin =
    parameter("safe_transit_margin", 0.0500);
  const double maximum_pregrasp_joint_travel =
    parameter("maximum_pregrasp_joint_travel", 1.6000);
  const double maximum_wrist_3_travel =
    parameter("maximum_wrist_3_travel", 3.2500);
  const bool pregrasp_only = bool_parameter("pregrasp_only", false);
  const bool skip_pregrasp_motion =
    bool_parameter("skip_pregrasp_motion", false);
  const bool use_named_start = bool_parameter("use_named_start", false);
  const bool approach_plan_only = bool_parameter("approach_plan_only", false);
  const bool approach_only = bool_parameter("approach_only", false);
  const bool lift_only = bool_parameter("lift_only", false);
  const bool use_width_calibration =
    bool_parameter("use_width_calibration", false);
  const double object_width_mm = parameter("object_width_mm", 50.0);
  const double total_compression_mm =
    parameter("total_compression_mm", 2.0);
  const double uncompensated_approach_distance =
    parameter("uncompensated_approach_distance", 0.1000);

  if (object_width_mm <= 0.0 || total_compression_mm < 0.0 ||
    total_compression_mm >= object_width_mm)
  {
    RCLCPP_ERROR(logger, "Object width or total compression is invalid.");
    rclcpp::shutdown();
    return 1;
  }

  if (use_width_calibration)
  {
    const auto solution = solve_gripper_calibration(
      object_width_mm, total_compression_mm);
    if (!solution)
    {
      RCLCPP_ERROR(
        logger,
        "Target pad gap %.2f mm is outside the calibrated range 45.32--85.00 mm.",
        object_width_mm - total_compression_mm);
      rclcpp::shutdown();
      return 1;
    }

    closed_grip = solution->command_rad;
    approach_distance =
      uncompensated_approach_distance - solution->closure_drop_m;
    RCLCPP_INFO(
      logger,
      "WIDTH_CALIBRATION width=%.2f mm compression=%.2f mm gap=%.2f mm "
      "command=%.6f rad closure_drop=%.3f mm approach=%.6f m "
      "bracket=[%.2f, %.2f] rad",
      object_width_mm, total_compression_mm,
      object_width_mm - total_compression_mm,
      closed_grip, solution->closure_drop_m * 1000.0, approach_distance,
      solution->bracket_low_rad, solution->bracket_high_rad);
  }

  const std::array<double, 23> values = {
    object_x, object_y, object_z, pregrasp_clearance, approach_distance,
    lift_distance, grasp_center_offset, closed_grip, velocity_scale,
    lift_velocity_scale,
    object_pose_timeout, grasp_roll_deg, grasp_pitch_deg, grasp_yaw_deg,
    floor_z, floor_size, floor_thickness, object_width_mm,
    total_compression_mm, uncompensated_approach_distance,
    safe_transit_margin, maximum_pregrasp_joint_travel,
    maximum_wrist_3_travel};
  for (const double value : values)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_ERROR(logger, "All grasp parameters must be finite.");
      rclcpp::shutdown();
      return 1;
    }
  }
  if (pregrasp_clearance <= 0.0 || approach_distance <= 0.0 ||
      approach_distance > pregrasp_clearance || lift_distance <= 0.0 ||
      grasp_center_offset <= 0.0 || velocity_scale <= 0.0 ||
      velocity_scale > 1.0 || lift_velocity_scale <= 0.0 ||
      lift_velocity_scale > 1.0 || object_pose_timeout <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "Clearance, approach distance, lift, offset, or velocity scale is invalid.");
    rclcpp::shutdown();
    return 1;
  }
  if (floor_size <= 0.0 || floor_thickness <= 0.0)
  {
    RCLCPP_ERROR(logger, "Floor size and thickness must be positive.");
    rclcpp::shutdown();
    return 1;
  }
  if (safe_transit_margin <= 0.0 || maximum_pregrasp_joint_travel <= 0.0)
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp limits must be positive.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
    object_pose_subscription;
  if (use_newton_object_pose)
  {
    auto pose_capture = std::make_shared<ObjectPoseCapture>();

    object_pose_subscription =
      node->create_subscription<geometry_msgs::msg::PoseStamped>(
      object_pose_topic, rclcpp::QoS(1),
      [pose_capture](const geometry_msgs::msg::PoseStamped::SharedPtr message) {
        std::lock_guard<std::mutex> lock(pose_capture->mutex);
        pose_capture->pose = *message;
        pose_capture->condition.notify_one();
      });

    RCLCPP_INFO(
      logger, "Waiting up to %.1f s for measured object pose on %s...",
      object_pose_timeout, object_pose_topic.c_str());
    std::unique_lock<std::mutex> lock(pose_capture->mutex);
    const bool received = pose_capture->condition.wait_for(
      lock, std::chrono::duration<double>(object_pose_timeout),
      [pose_capture]() {return pose_capture->pose.has_value();});

    if (!received || pose_capture->pose->header.frame_id != "world")
    {
      RCLCPP_ERROR(
        logger, "No valid world-frame object pose received from %s.",
        object_pose_topic.c_str());
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    object_x = pose_capture->pose->pose.position.x;
    object_y = pose_capture->pose->pose.position.y;
    object_z = pose_capture->pose->pose.position.z;
    if (!std::isfinite(object_x) || !std::isfinite(object_y) ||
      !std::isfinite(object_z))
    {
      RCLCPP_ERROR(logger, "Measured object pose contains a non-finite position.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(
      logger,
      "MEASURED_OBJECT_POSE topic=%s frame=world center=(%.4f, %.4f, %.4f) m",
      object_pose_topic.c_str(), object_x, object_y, object_z);
  }

  MoveGroup move_group(node, "ur_manipulator");
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(velocity_scale);
  move_group.setPlanningTime(10.0);

  auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/robotiq_gripper_controller/gripper_cmd");
  auto display_publisher = node->create_publisher<DisplayTrajectory>(
    "/display_planned_path", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(
    logger,
    "OBJECT_TARGET source=%s frame=world center=(%.4f, %.4f, %.4f) m",
    use_newton_object_pose ? "newton_feedback" : "parameters",
    object_x, object_y, object_z);
  RCLCPP_INFO(
    logger,
    "GRASP_CONFIG offset=%.4f m clearance=%.4f m approach=%.4f m "
    "lift=%.4f m closed=%.4f rad lift_velocity_scale=%.3f",
    grasp_center_offset, pregrasp_clearance, approach_distance,
    lift_distance, closed_grip, lift_velocity_scale);
  RCLCPP_INFO(
    logger,
    "GRASP_ORIENTATION roll=%.1f pitch=%.1f yaw=%.1f deg",
    grasp_roll_deg, grasp_pitch_deg, grasp_yaw_deg);

  if (pose_only)
  {
    RCLCPP_INFO(logger, "POSE_ONLY_CHECK_SUCCEEDED; no robot command was sent.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  if (!add_floor_collision(
      move_group, logger, floor_z, floor_size, floor_thickness))
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  geometry_msgs::msg::Pose pregrasp_tool_pose;
  pregrasp_tool_pose.orientation = quaternion_from_rpy_degrees(
    grasp_roll_deg, grasp_pitch_deg, grasp_yaw_deg);
  const auto offset_world = rotate_local_z(
    pregrasp_tool_pose.orientation, grasp_center_offset);
  pregrasp_tool_pose.position.x = object_x - offset_world[0];
  pregrasp_tool_pose.position.y = object_y - offset_world[1];
  pregrasp_tool_pose.position.z =
    object_z + pregrasp_clearance - offset_world[2];

  if (plan_only)
  {
    const bool planned = use_safe_pregrasp_path ?
      move_staged_cartesian_to_pose(
        move_group, display_publisher, logger, pregrasp_tool_pose,
        safe_transit_margin, maximum_pregrasp_joint_travel,
        maximum_wrist_3_travel,
        velocity_scale, false) :
      move_to_pose_target(
        move_group, display_publisher, logger, pregrasp_tool_pose,
        "Absolute pre-grasp", false);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return planned ? 0 : 1;
  }

  if (approach_plan_only)
  {
    const auto current = move_group.getCurrentPose("tool0").pose.position;
    const double position_error = std::sqrt(
      std::pow(current.x - pregrasp_tool_pose.position.x, 2) +
      std::pow(current.y - pregrasp_tool_pose.position.y, 2) +
      std::pow(current.z - pregrasp_tool_pose.position.z, 2));
    RCLCPP_INFO(
      logger, "PREGRASP_START_ERROR position=%.4f m", position_error);
    if (position_error > 0.02)
    {
      RCLCPP_ERROR(
        logger,
        "Approach preview requires the robot to be within 0.02 m of pre-grasp.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    const bool planned = move_relative(
      move_group, display_publisher, logger,
      0.0, 0.0, -approach_distance,
      "Vertical grasp approach", velocity_scale, false);
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return planned ? 0 : 1;
  }

  if (lift_only)
  {
    const auto current_state = move_group.getCurrentState(10.0);
    if (!current_state)
    {
      RCLCPP_ERROR(logger, "Lift-only test cannot obtain current robot state.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    const double grip_position = current_state->getVariablePosition(
      "robotiq_85_left_knuckle_joint");
    RCLCPP_INFO(
      logger, "LIFT_ONLY_START gripper=%.4f rad distance=%.4f m",
      grip_position, lift_distance);
    if (grip_position < 0.20)
    {
      RCLCPP_ERROR(logger, "Lift-only test refused because the gripper is open.");
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }

    const bool lifted = move_relative(
      move_group, display_publisher, logger,
      0.0, 0.0, lift_distance, "Low test lift", lift_velocity_scale);
    if (lifted)
    {
      RCLCPP_INFO(
        logger,
        "LIFT_ONLY_SUCCEEDED; gripper remains closed and no release was sent.");
    }
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return lifted ? 0 : 1;
  }

  bool success = command_gripper(gripper_client, logger, 0.0, 50.0);

  if (success && use_named_start)
  {
    success = move_to_named_target(move_group, logger, "test_configuration");
  }

  if (success && !move_group.getCurrentState(10.0))
  {
    RCLCPP_ERROR(logger, "Cannot obtain the work-start robot state.");
    success = false;
  }

  if (success)
  {
    RCLCPP_INFO(
      logger,
      "PREGRASP_CENTER world=(%.4f, %.4f, %.4f) m",
      object_x, object_y, object_z + pregrasp_clearance);
    if (skip_pregrasp_motion)
    {
      const auto current = move_group.getCurrentPose("tool0").pose.position;
      const double position_error = std::sqrt(
        std::pow(current.x - pregrasp_tool_pose.position.x, 2) +
        std::pow(current.y - pregrasp_tool_pose.position.y, 2) +
        std::pow(current.z - pregrasp_tool_pose.position.z, 2));
      RCLCPP_INFO(
        logger, "PREGRASP_MOTION_SKIPPED position_error=%.4f m",
        position_error);
      if (position_error > 0.02)
      {
        RCLCPP_ERROR(
          logger,
          "Cannot skip pre-grasp motion: position error exceeds 0.02 m.");
        success = false;
      }
    }
    else
    {
      success = use_safe_pregrasp_path ?
        move_staged_cartesian_to_pose(
          move_group, display_publisher, logger, pregrasp_tool_pose,
          safe_transit_margin, maximum_pregrasp_joint_travel,
          maximum_wrist_3_travel,
          velocity_scale, true) :
        move_to_pose_target(
          move_group, display_publisher, logger, pregrasp_tool_pose,
          "Absolute pre-grasp");
    }
  }

  if (success && pregrasp_only)
  {
    RCLCPP_INFO(
      logger,
      "PREGRASP_ONLY_SUCCEEDED; stopped before descent and gripper closure.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  if (success)
  {
    success = move_relative(
      move_group, display_publisher, logger, 0.0, 0.0, -approach_distance,
      "Vertical grasp approach", velocity_scale);
  }

  if (success && approach_only)
  {
    RCLCPP_INFO(
      logger,
      "APPROACH_ONLY_SUCCEEDED; stopped before gripper closure.");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
  }

  if (success)
  {
    success = command_gripper(gripper_client, logger, closed_grip, 50.0);
  }

  if (success)
  {
    std::this_thread::sleep_for(1s);
    success = move_relative(
      move_group, display_publisher, logger, 0.0, 0.0, lift_distance,
      "Vertical lift", lift_velocity_scale);
  }

  if (success)
  {
    std::this_thread::sleep_for(1s);
    success = command_gripper(gripper_client, logger, 0.0, 50.0);
  }

  if (success)
  {
    RCLCPP_INFO(logger, "========== ABSOLUTE POSITION PICK SUCCEEDED ==========");
  }
  else
  {
    RCLCPP_ERROR(logger, "========== ABSOLUTE POSITION PICK STOPPED ==========");
  }

  executor.cancel();
  if (spinner.joinable())
  {
    spinner.join();
  }
  rclcpp::shutdown();
  return success ? 0 : 1;
}
