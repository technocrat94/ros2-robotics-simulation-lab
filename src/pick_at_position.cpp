#include <algorithm>
#include <array>
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <cmath>
#include <limits>
#include <mutex>
#include <optional>
#include <vector>
#include <stdexcept>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.hpp>
#include <moveit/collision_detection/collision_common.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/link_padding.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
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

struct ObstacleBox
{
  std::string id;
  std::string frame;
  std::array<double, 3> center;
  std::array<double, 3> size;
};

struct MovingObstacleCapture
{
  std::mutex mutex;
  ObstacleBox box;
  std::size_t update_sequence{0};
};

struct DynamicReplanningOptions
{
  bool enabled{false};
  double padding_m{0.0};
  int maximum_replans{0};
  std::shared_ptr<MovingObstacleCapture> obstacle;
};

DynamicReplanningOptions dynamic_replanning;

bool apply_robot_padding(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  double padding_m,
  const std::string & stage)
{
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.robot_state.is_diff = true;
  const auto & links =
    move_group.getRobotModel()->getLinkModelNamesWithCollisionGeometry();
  scene.link_padding.reserve(links.size());
  for (const auto & link_name : links)
  {
    moveit_msgs::msg::LinkPadding link_padding;
    link_padding.link_name = link_name;
    link_padding.padding = padding_m;
    scene.link_padding.push_back(link_padding);
  }

  moveit::planning_interface::PlanningSceneInterface planning_scene;
  if (!planning_scene.applyPlanningScene(scene))
  {
    RCLCPP_ERROR(
      logger, "Failed to apply %.6f m robot padding for %s phase.",
      padding_m, stage.c_str());
    return false;
  }
  RCLCPP_INFO(
    logger, "ACTIVE_PADDING stage=%s padding_m=%.6f links=%zu",
    stage.c_str(), padding_m, links.size());
  return true;
}

moveit_msgs::msg::CollisionObject make_box_obstacle(const ObstacleBox & box)
{
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = box.frame;
  obstacle.id = box.id;
  shape_msgs::msg::SolidPrimitive primitive;
  primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
  primitive.dimensions = {box.size[0], box.size[1], box.size[2]};
  geometry_msgs::msg::Pose pose;
  pose.orientation.w = 1.0;
  pose.position.x = box.center[0];
  pose.position.y = box.center[1];
  pose.position.z = box.center[2];
  obstacle.primitives.push_back(primitive);
  obstacle.primitive_poses.push_back(pose);
  return obstacle;
}

bool report_route_metrics(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  const moveit_msgs::msg::RobotState & start_state_message,
  const moveit_msgs::msg::RobotTrajectory & trajectory_message,
  const ObstacleBox & obstacle,
  double robot_padding_m,
  const std::string & stage_name)
{
  moveit::core::RobotState start_state(move_group.getRobotModel());
  moveit::core::robotStateMsgToRobotState(start_state_message, start_state);
  robot_trajectory::RobotTrajectory trajectory(
    move_group.getRobotModel(), move_group.getName());
  trajectory.setRobotTrajectoryMsg(start_state, trajectory_message);
  if (trajectory.getWayPointCount() == 0)
  {
    RCLCPP_ERROR(logger, "%s route metrics: trajectory is empty.", stage_name.c_str());
    return false;
  }

  planning_scene::PlanningScene analysis_scene(move_group.getRobotModel());
  analysis_scene.getCollisionEnvNonConst()->setPadding(robot_padding_m);
  auto collision_object = make_box_obstacle(obstacle);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!analysis_scene.processCollisionObjectMsg(collision_object))
  {
    RCLCPP_ERROR(logger, "%s route metrics: obstacle could not be created.", stage_name.c_str());
    return false;
  }

  double maximum_end_effector_height = -std::numeric_limits<double>::infinity();
  double minimum_robot_box_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index)
  {
    const auto & state = trajectory.getWayPoint(index);
    maximum_end_effector_height = std::max(
      maximum_end_effector_height,
      state.getGlobalLinkTransform("tool0").translation().z());
    collision_detection::DistanceRequest request;
    collision_detection::DistanceResult result;
    request.type = collision_detection::DistanceRequestTypes::GLOBAL;
    request.enable_signed_distance = true;
    request.group_name = move_group.getName();
    request.enableGroup(move_group.getRobotModel());
    request.acm = &analysis_scene.getAllowedCollisionMatrix();
    analysis_scene.getCollisionEnv()->distanceRobot(request, result, state);
    minimum_robot_box_distance = std::min(
      minimum_robot_box_distance, result.minimum_distance.distance);
  }
  RCLCPP_INFO(
    logger,
    "PLANNED_ROUTE_METRICS stage=\"%s\" samples=%zu max_end_effector_height_m=%.6f "
    "min_robot_box_distance_m=%.6f obstacle_id=%s",
    stage_name.c_str(), trajectory.getWayPointCount(), maximum_end_effector_height,
    minimum_robot_box_distance, obstacle.id.c_str());
  return std::isfinite(maximum_end_effector_height) &&
         std::isfinite(minimum_robot_box_distance);
}


std::pair<ObstacleBox, std::size_t> moving_obstacle_snapshot()
{
  std::lock_guard<std::mutex> lock(dynamic_replanning.obstacle->mutex);
  return {
    dynamic_replanning.obstacle->box,
    dynamic_replanning.obstacle->update_sequence
  };
}


bool remaining_trajectory_within_wall_padding(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  const MoveGroup::Plan & plan,
  const ObstacleBox & obstacle,
  double padding_m)
{
  moveit::core::RobotState start_state(move_group.getRobotModel());
  moveit::core::robotStateMsgToRobotState(plan.start_state, start_state);
  robot_trajectory::RobotTrajectory trajectory(
    move_group.getRobotModel(), move_group.getName());
  trajectory.setRobotTrajectoryMsg(start_state, plan.trajectory);
  if (trajectory.getWayPointCount() == 0)
  {
    return true;
  }

  const auto current_state = move_group.getCurrentState(1.0);
  if (!current_state)
  {
    RCLCPP_WARN(logger, "Dynamic obstacle check could not read current state.");
    return true;
  }
  const auto * group = move_group.getRobotModel()->getJointModelGroup(
    move_group.getName());
  std::size_t nearest = 0;
  double nearest_distance = std::numeric_limits<double>::infinity();
  for (std::size_t index = 0; index < trajectory.getWayPointCount(); ++index)
  {
    const double distance = trajectory.getWayPoint(index).distance(
      *current_state, group);
    if (distance < nearest_distance)
    {
      nearest_distance = distance;
      nearest = index;
    }
  }

  planning_scene::PlanningScene analysis_scene(move_group.getRobotModel());
  auto collision_object = make_box_obstacle(obstacle);
  collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
  if (!analysis_scene.processCollisionObjectMsg(collision_object))
  {
    RCLCPP_WARN(logger, "Dynamic obstacle check could not create the wall.");
    return true;
  }
  for (std::size_t index = nearest; index < trajectory.getWayPointCount(); ++index)
  {
    collision_detection::DistanceRequest request;
    collision_detection::DistanceResult result;
    request.type = collision_detection::DistanceRequestTypes::GLOBAL;
    request.enable_signed_distance = true;
    request.group_name = move_group.getName();
    request.enableGroup(move_group.getRobotModel());
    request.acm = &analysis_scene.getAllowedCollisionMatrix();
    analysis_scene.getCollisionEnv()->distanceRobot(
      request, result, trajectory.getWayPoint(index));
    if (result.minimum_distance.distance <= padding_m)
    {
      RCLCPP_WARN(
        logger,
        "DYNAMIC_WALL_PATH_BLOCKED waypoint=%zu remaining_start=%zu "
        "distance_m=%.6f padding_m=%.6f",
        index, nearest, result.minimum_distance.distance, padding_m);
      return true;
    }
  }
  return false;
}


bool execute_with_dynamic_replanning(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  MoveGroup::Plan plan,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name)
{
  if (!dynamic_replanning.enabled)
  {
    return static_cast<bool>(move_group.execute(plan));
  }

  for (int attempt = 0; attempt <= dynamic_replanning.maximum_replans; ++attempt)
  {
    auto [wall, wall_sequence] = moving_obstacle_snapshot();
    if (!remaining_trajectory_within_wall_padding(
        move_group, logger, plan, wall, dynamic_replanning.padding_m))
    {
      auto execution = std::async(
        std::launch::async,
        [&move_group, &plan]() {
          return static_cast<bool>(move_group.execute(plan));
        });
      bool interrupted = false;
      while (execution.wait_for(50ms) != std::future_status::ready)
      {
        auto [latest_wall, latest_sequence] = moving_obstacle_snapshot();
        if (latest_sequence == wall_sequence)
        {
          continue;
        }
        wall_sequence = latest_sequence;
        if (remaining_trajectory_within_wall_padding(
            move_group, logger, plan, latest_wall,
            dynamic_replanning.padding_m))
        {
          move_group.stop();
          interrupted = true;
          break;
        }
      }
      const bool executed = execution.get();
      if (!interrupted)
      {
        return executed;
      }
    }

    if (attempt == dynamic_replanning.maximum_replans)
    {
      RCLCPP_ERROR(
        logger, "%s exhausted %d dynamic replans.", stage_name.c_str(),
        dynamic_replanning.maximum_replans);
      return false;
    }
    RCLCPP_INFO(
      logger, "DYNAMIC_WALL_REPLAN stage=\"%s\" attempt=%d",
      stage_name.c_str(), attempt + 1);
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target, "tool0");
    const auto result = move_group.plan(plan);
    move_group.clearPoseTargets();
    if (!static_cast<bool>(result))
    {
      RCLCPP_ERROR(logger, "%s dynamic replan failed.", stage_name.c_str());
      return false;
    }
  }
  return false;
}


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

  static constexpr std::array<Sample, 14> samples = {{
    {0.00, 0.0850000000, 0.0000000000},
    {0.10, 0.0759588652, 0.0034926322},
    {0.20, 0.0662655355, 0.0065165122},
    {0.30, 0.0560168635, 0.0090414263},
    {0.35, 0.0507160854, 0.0101086854},
    {0.40, 0.0453152505, 0.0110421465},
    {0.45, 0.0398278050, 0.0118394494},
    {0.50, 0.0342675700, 0.0124986762},
    {0.55, 0.0286483953, 0.0130180717},
    {0.60, 0.0229843219, 0.0133964717},
    {0.65, 0.0172895227, 0.0136328042},
    {0.70, 0.0115781826, 0.0137265325},
    {0.75, 0.0058646306, 0.0136774778},
    {0.80, 0.0001631189, 0.0134856403},
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

  const bool executed = execute_with_dynamic_replanning(
    move_group, logger, plan, target_pose, stage_name);

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
  double robot_padding_m,
  bool execute_motion,
  const std::optional<ObstacleBox> & analysis_obstacle = std::nullopt,
  bool obstacle_active = false,
  double fingertip_drop_m = 0.0,
  double obstacle_clearance_m = 0.0)
{
  const auto current_state = move_group.getCurrentState(10.0);
  if (!current_state)
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp: cannot obtain current robot state.");
    return false;
  }

  const auto current_pose = move_group.getCurrentPose("tool0").pose;
  double safe_z =
    std::max(current_pose.position.z, target.position.z) + safe_transit_margin;
  // tool0 cruises above the obstacle, but the open fingers hang about
  // fingertip_drop_m below tool0; the padded fingertips must clear the top.
  if (obstacle_active && analysis_obstacle)
  {
    const double obstacle_top =
      analysis_obstacle->center[2] + 0.5 * analysis_obstacle->size[2];
    const double obstacle_safe_z =
      obstacle_top + fingertip_drop_m + robot_padding_m + obstacle_clearance_m;
    RCLCPP_INFO(
      logger,
      "SAFE_PREGRASP_OBSTACLE_HEIGHT top=%.3f fingertip_drop=%.3f padding=%.3f "
      "clearance=%.3f obstacle_safe_z=%.3f margin_safe_z=%.3f",
      obstacle_top, fingertip_drop_m, robot_padding_m, obstacle_clearance_m,
      obstacle_safe_z, safe_z);
    safe_z = std::max(safe_z, obstacle_safe_z);
  }

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

  if (analysis_obstacle && !report_route_metrics(
      move_group, logger, plan.start_state, plan.trajectory,
      *analysis_obstacle, robot_padding_m, "safe_staged_pregrasp"))
  {
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
      "SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED; four-stage preview published; "
      "no robot command was sent.");
    std::this_thread::sleep_for(500ms);
    return true;
  }

  if (!execute_with_dynamic_replanning(
      move_group, logger, plan, target, "Safe staged pre-grasp"))
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


bool add_box_obstacle(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  const ObstacleBox & box)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  if (box.frame != move_group.getPlanningFrame())
  {
    RCLCPP_WARN(
      logger, "Obstacle frame '%s' differs from planning frame '%s'.",
      box.frame.c_str(), move_group.getPlanningFrame().c_str());
  }
  auto obstacle = make_box_obstacle(box);
  obstacle.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (!planning_scene.applyCollisionObject(obstacle))
  {
    RCLCPP_ERROR(logger, "Failed to add rigid obstacle to the planning scene.");
    return false;
  }
  RCLCPP_INFO(
    logger,
    "PLANNING_SCENE_OBSTACLE_ADDED id=%s frame=%s center=(%.3f,%.3f,%.3f) "
    "size=(%.3f,%.3f,%.3f)",
    box.id.c_str(), box.frame.c_str(), box.center[0], box.center[1], box.center[2],
    box.size[0], box.size[1], box.size[2]);
  return true;
}

bool remove_box_obstacle(
  const rclcpp::Logger & logger,
  const std::string & obstacle_id,
  const std::string & planning_frame)
{
  moveit::planning_interface::PlanningSceneInterface planning_scene;
  moveit_msgs::msg::CollisionObject obstacle;
  obstacle.header.frame_id = planning_frame;
  obstacle.id = obstacle_id;
  obstacle.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  const bool removal_applied = planning_scene.applyCollisionObject(obstacle);
  const auto known = planning_scene.getKnownObjectNames();
  const bool absent = std::find(known.begin(), known.end(), obstacle_id) == known.end();
  if (!removal_applied && absent)
  {
    RCLCPP_WARN(
      logger, "Obstacle '%s' was already absent; explicit REMOVE was still sent.",
      obstacle_id.c_str());
  }
  RCLCPP_INFO(
    logger, "PLANNING_SCENE_OBSTACLE_REMOVED id=%s remove_response=%s confirmed_absent=%s",
    obstacle_id.c_str(), removal_applied ? "true" : "false",
    absent ? "true" : "false");
  return absent;
}


bool move_to_pose_target(
  MoveGroup & move_group,
  const rclcpp::Publisher<DisplayTrajectory>::SharedPtr & display_publisher,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name,
  double robot_padding_m,
  bool execute_motion = true,
  const std::optional<ObstacleBox> & analysis_obstacle = std::nullopt)
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

  if (analysis_obstacle && !report_route_metrics(
      move_group, logger, plan.start_state, plan.trajectory,
      *analysis_obstacle, robot_padding_m, stage_name))
  {
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

  if (!execute_with_dynamic_replanning(
      move_group, logger, plan, target, stage_name))
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
  auto int_parameter = [&node](const std::string & name, int default_value) {
      if (!node->has_parameter(name))
      {
        node->declare_parameter<int>(name, default_value);
      }
      return static_cast<int>(node->get_parameter(name).as_int());
    };
  auto string_parameter =
    [&node](const std::string & name, const std::string & default_value) {
      if (!node->has_parameter(name))
      {
        node->declare_parameter<std::string>(name, default_value);
      }
      return node->get_parameter(name).as_string();
    };
  auto required_double_parameter = [&node](const std::string & name) {
      if (!node->has_parameter(name))
      {
        throw std::runtime_error("missing required parameter: " + name);
      }
      return node->get_parameter(name).as_double();
    };
  auto required_string_parameter = [&node](const std::string & name) {
      if (!node->has_parameter(name))
      {
        throw std::runtime_error("missing required parameter: " + name);
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
  const double planning_time_s = parameter("planning_time_s", 10.0);
  const std::string scene_profile = string_parameter("scene_profile", "default");
  const double robot_padding_m = parameter("robot_padding_m", 0.0);
  const double contact_padding_m = parameter("contact_padding_m", 0.0);
  const bool obstacles_enabled = bool_parameter("obstacles_enabled", false);
  const bool dynamic_obstacle_replanning =
    bool_parameter("dynamic_obstacle_replanning", false);
  const std::string obstacle_pose_topic =
    string_parameter("obstacle_pose_topic", "/newton/obstacle_poses");
  const int maximum_dynamic_replans =
    int_parameter("maximum_dynamic_replans", 3);
  ObstacleBox obstacle_box;
  try
  {
    obstacle_box = {
      required_string_parameter("obstacle_id"),
      required_string_parameter("obstacle_frame"),
      {
        required_double_parameter("obstacle_x"),
        required_double_parameter("obstacle_y"),
        required_double_parameter("obstacle_z")
      },
      {
        required_double_parameter("obstacle_size_x"),
        required_double_parameter("obstacle_size_y"),
        required_double_parameter("obstacle_size_z")
      }
    };
  }
  catch (const std::exception & error)
  {
    RCLCPP_ERROR(logger, "Rigid obstacle configuration error: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  const bool plan_only = bool_parameter("plan_only", false);
  const bool use_safe_pregrasp_path =
    bool_parameter("use_safe_pregrasp_path", true);
  const double safe_transit_margin =
    parameter("safe_transit_margin", 0.0500);
  // Measured by step_c_geometry_preflight: lowest open fingertip is 0.160 m
  // below tool0 at the start pose; 5 mm added for mesh tolerance.
  const double fingertip_drop_m = parameter("fingertip_drop_m", 0.165);
  const double obstacle_clearance_m = parameter("obstacle_clearance_m", 0.03);
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
        "Target pad gap %.2f mm is outside the calibrated range 0.16--85.00 mm.",
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

  const std::array<double, 26> values = {
    object_x, object_y, object_z, pregrasp_clearance, approach_distance,
    lift_distance, grasp_center_offset, closed_grip, velocity_scale,
    lift_velocity_scale,
    object_pose_timeout, grasp_roll_deg, grasp_pitch_deg, grasp_yaw_deg,
    floor_z, floor_size, floor_thickness, object_width_mm,
    total_compression_mm, uncompensated_approach_distance,
    safe_transit_margin, maximum_pregrasp_joint_travel,
    maximum_wrist_3_travel, robot_padding_m, contact_padding_m, planning_time_s};
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
  if (scene_profile.empty() || robot_padding_m < 0.0 ||
    contact_padding_m < 0.0 || planning_time_s <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "Scene profile must be set, both paddings nonnegative, and planning time positive.");
    rclcpp::shutdown();
    return 1;
  }
  if (obstacles_enabled &&
    (obstacle_box.id.empty() || obstacle_box.frame.empty() ||
    std::any_of(obstacle_box.center.begin(), obstacle_box.center.end(),
    [](double value) {return !std::isfinite(value);}) ||
    std::any_of(obstacle_box.size.begin(), obstacle_box.size.end(),
    [](double value) {return !std::isfinite(value) || value <= 0.0;})))
  {
    RCLCPP_ERROR(logger, "Rigid obstacle configuration is invalid.");
    rclcpp::shutdown();
    return 1;
  }
  if (dynamic_obstacle_replanning &&
    (!obstacles_enabled || maximum_dynamic_replans < 1 || obstacle_pose_topic.empty()))
  {
    RCLCPP_ERROR(
      logger,
      "Dynamic obstacle replanning needs an enabled obstacle, a pose topic, "
      "and at least one replan.");
    rclcpp::shutdown();
    return 1;
  }
  if (safe_transit_margin <= 0.0 || maximum_pregrasp_joint_travel <= 0.0)
  {
    RCLCPP_ERROR(logger, "Safe pre-grasp limits must be positive.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr
    obstacle_pose_subscription;
  if (dynamic_obstacle_replanning)
  {
    dynamic_replanning.enabled = true;
    dynamic_replanning.padding_m = robot_padding_m;
    dynamic_replanning.maximum_replans = maximum_dynamic_replans;
    dynamic_replanning.obstacle = std::make_shared<MovingObstacleCapture>();
    dynamic_replanning.obstacle->box = obstacle_box;
    obstacle_pose_subscription =
      node->create_subscription<geometry_msgs::msg::PoseArray>(
      obstacle_pose_topic, rclcpp::QoS(10),
      [logger](const geometry_msgs::msg::PoseArray::SharedPtr message) {
        if (message->poses.empty())
        {
          RCLCPP_WARN(logger, "Received an empty obstacle pose array.");
          return;
        }
        ObstacleBox updated;
        {
          std::lock_guard<std::mutex> lock(dynamic_replanning.obstacle->mutex);
          updated = dynamic_replanning.obstacle->box;
          updated.frame = message->header.frame_id.empty() ?
            updated.frame : message->header.frame_id;
          updated.center = {
            message->poses.front().position.x,
            message->poses.front().position.y,
            message->poses.front().position.z};
          dynamic_replanning.obstacle->box = updated;
          ++dynamic_replanning.obstacle->update_sequence;
        }
        auto collision_object = make_box_obstacle(updated);
        collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
        moveit::planning_interface::PlanningSceneInterface planning_scene;
        if (!planning_scene.applyCollisionObject(collision_object))
        {
          RCLCPP_WARN(logger, "Failed to update moving wall in planning scene.");
        }
      });
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
  move_group.setPlanningTime(planning_time_s);

  auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/robotiq_gripper_controller/gripper_cmd");
  auto display_publisher = node->create_publisher<DisplayTrajectory>(
    "/display_planned_path", rclcpp::QoS(1).transient_local());

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(
    logger,
    "SCENE_CONFIG profile=%s obstacles_enabled=%s transit_padding_m=%.6f "
    "contact_padding_m=%.6f planning_time_s=%.3f dynamic_replanning=%s "
    "maximum_dynamic_replans=%d",
    scene_profile.c_str(), obstacles_enabled ? "true" : "false", robot_padding_m,
    contact_padding_m, planning_time_s,
    dynamic_obstacle_replanning ? "true" : "false", maximum_dynamic_replans);
  RCLCPP_INFO(
    logger, "USE_SAFE_PREGRASP_PATH %s",
    use_safe_pregrasp_path ? "true" : "false");
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

  if (obstacles_enabled && !add_box_obstacle(
      move_group, logger, obstacle_box))
  {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }
  if (!obstacles_enabled)
  {
    if (!remove_box_obstacle(
        logger, obstacle_box.id, move_group.getPlanningFrame()))
    {
      executor.cancel();
      spinner.join();
      rclcpp::shutdown();
      return 1;
    }
    RCLCPP_INFO(logger, "PLANNING_SCENE_OBSTACLES_DISABLED scene_clean=true");
  }

  if (!apply_robot_padding(move_group, logger, robot_padding_m, "transit"))
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
        velocity_scale, robot_padding_m, false, obstacle_box,
        obstacles_enabled, fingertip_drop_m, obstacle_clearance_m) :
      move_to_pose_target(
        move_group, display_publisher, logger, pregrasp_tool_pose,
        "Absolute pre-grasp", robot_padding_m, false, obstacle_box);
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

    const bool planned = apply_robot_padding(
      move_group, logger, contact_padding_m, "contact") && move_relative(
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

    const bool lifted = apply_robot_padding(
      move_group, logger, contact_padding_m, "contact") && move_relative(
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
          velocity_scale, robot_padding_m, true, obstacle_box,
        obstacles_enabled, fingertip_drop_m, obstacle_clearance_m) :
        move_to_pose_target(
          move_group, display_publisher, logger, pregrasp_tool_pose,
          "Absolute pre-grasp", robot_padding_m, true, obstacle_box);
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
    success = apply_robot_padding(
      move_group, logger, contact_padding_m, "contact");
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
