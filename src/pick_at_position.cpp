#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <cmath>
#include <vector>
#include <stdexcept>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::chrono_literals;

using GripperCommand = control_msgs::action::GripperCommand;
using GripperGoalHandle =
  rclcpp_action::ClientGoalHandle<GripperCommand>;

using MoveGroup =
  moveit::planning_interface::MoveGroupInterface;


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
  const rclcpp::Logger & logger,
  double dx,
  double dy,
  double dz,
  const std::string & stage_name)
{
  if (!move_group.getCurrentState(10.0))
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

  MoveGroup::Plan plan;
  plan.trajectory_ = trajectory;

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


bool move_to_pose_target(
  MoveGroup & move_group,
  const rclcpp::Logger & logger,
  const geometry_msgs::msg::Pose & target,
  const std::string & stage_name)
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

  const double object_x = parameter("object_x", 0.4869);
  const double object_y = parameter("object_y", 0.10915);
  const double object_z = parameter("object_z", 0.0100);
  const double pregrasp_clearance = parameter("pregrasp_clearance", 0.1500);
  const double lift_distance = parameter("lift_distance", 0.1200);
  const double grasp_center_offset = parameter("grasp_center_offset", 0.1090);
  const double closed_grip = parameter("closed_grip", 0.3760);
  const double velocity_scale = parameter("velocity_scale", 0.15);

  const std::array<double, 8> values = {
    object_x, object_y, object_z, pregrasp_clearance,
    lift_distance, grasp_center_offset, closed_grip, velocity_scale};
  for (const double value : values)
  {
    if (!std::isfinite(value))
    {
      RCLCPP_ERROR(logger, "All grasp parameters must be finite.");
      rclcpp::shutdown();
      return 1;
    }
  }
  if (pregrasp_clearance <= 0.0 || lift_distance <= 0.0 ||
      grasp_center_offset <= 0.0 || velocity_scale <= 0.0 ||
      velocity_scale > 1.0)
  {
    RCLCPP_ERROR(logger, "Clearance, lift, offset, and velocity scale are invalid.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});

  MoveGroup move_group(node, "ur_manipulator");
  move_group.setMaxVelocityScalingFactor(velocity_scale);
  move_group.setMaxAccelerationScalingFactor(velocity_scale);
  move_group.setPlanningTime(10.0);

  auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/robotiq_gripper_controller/gripper_cmd");

  RCLCPP_INFO(logger, "Planning frame: %s", move_group.getPlanningFrame().c_str());
  RCLCPP_INFO(
    logger,
    "OBJECT_TARGET frame=world center=(%.4f, %.4f, %.4f) m",
    object_x, object_y, object_z);
  RCLCPP_INFO(
    logger,
    "GRASP_CONFIG offset=%.4f m clearance=%.4f m lift=%.4f m closed=%.4f rad",
    grasp_center_offset, pregrasp_clearance, lift_distance, closed_grip);

  bool success = command_gripper(gripper_client, logger, 0.0, 50.0);

  if (success)
  {
    success = move_to_named_target(move_group, logger, "test_configuration");
  }

  if (success && !move_group.getCurrentState(10.0))
  {
    RCLCPP_ERROR(logger, "Cannot obtain the work-start robot state.");
    success = false;
  }

  geometry_msgs::msg::Pose pregrasp_tool_pose;
  if (success)
  {
    // Hold the verified top-down orientation from test_configuration.
    pregrasp_tool_pose = move_group.getCurrentPose("tool0").pose;
    const auto offset_world = rotate_local_z(
      pregrasp_tool_pose.orientation, grasp_center_offset);

    // grasp_center = tool0 + R_world_tool0 * [0, 0, offset]
    // Therefore tool0 = desired_grasp_center - rotated_offset.
    pregrasp_tool_pose.position.x = object_x - offset_world[0];
    pregrasp_tool_pose.position.y = object_y - offset_world[1];
    pregrasp_tool_pose.position.z =
      object_z + pregrasp_clearance - offset_world[2];

    RCLCPP_INFO(
      logger,
      "PREGRASP_CENTER world=(%.4f, %.4f, %.4f) m",
      object_x, object_y, object_z + pregrasp_clearance);
    success = move_to_pose_target(
      move_group, logger, pregrasp_tool_pose, "Absolute pre-grasp");
  }

  if (success)
  {
    success = move_relative(
      move_group, logger, 0.0, 0.0, -pregrasp_clearance,
      "Vertical grasp approach");
  }

  if (success)
  {
    success = command_gripper(gripper_client, logger, closed_grip, 50.0);
  }

  if (success)
  {
    std::this_thread::sleep_for(1s);
    success = move_relative(
      move_group, logger, 0.0, 0.0, lift_distance,
      "Vertical lift");
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
