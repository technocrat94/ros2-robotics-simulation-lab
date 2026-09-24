#include <chrono>
#include <future>
#include <memory>
#include <cmath>
#include <vector>
#include <string>
#include <thread>

#include <control_msgs/action/gripper_command.hpp>
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
  plan.trajectory = trajectory;

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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>(
    "pick_and_place_demo",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true)
  );

  const auto logger = node->get_logger();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  MoveGroup move_group(node, "ur_manipulator");

  move_group.setMaxVelocityScalingFactor(0.2);
  move_group.setMaxAccelerationScalingFactor(0.2);
  move_group.setPlanningTime(10.0);

  auto gripper_client =
    rclcpp_action::create_client<GripperCommand>(
      node,
      "/robotiq_gripper_controller/gripper_cmd"
    );

  RCLCPP_INFO(
    logger,
    "Planning frame: %s",
    move_group.getPlanningFrame().c_str()
  );

  RCLCPP_INFO(
    logger,
    "End effector: %s",
    move_group.getEndEffectorLink().c_str()
  );

  bool success = true;

  // These variables remember the completed relative motions.
  // Do not edit them here. Edit the values inside APPROACH and
  // LIFT AND TRANSPORT below.
  double approach_dx = 0.0;
  double approach_dy = 0.0;
  double approach_dz = 0.0;

  double transport_dx = 0.0;
  double transport_dy = 0.0;
  double transport_dz = 0.0;

  // 1. Open gripper
  RCLCPP_INFO(
    logger,
    "========== OPEN GRIPPER =========="
  );

  success = command_gripper(
    gripper_client,
    logger,
    0.0,
    50.0
  );

  // 2. Move to the verified work-start configuration
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== MOVE TO WORK START =========="
    );

    success = move_to_named_target(
      move_group,
      logger,
      "test_configuration"
    );
  }

  // 3. Approach the object
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== APPROACH =========="
    );

    // ================================================
    // APPROACH PARAMETERS
    // Only edit these three values for Approach.
    // Unit: meter. Frame: world.
    // ================================================
    approach_dx = 0.03;
    approach_dy = 0.0;
    approach_dz = 0.0;

    success = move_relative(
      move_group,
      logger,
      approach_dx,
      approach_dy,
      approach_dz,
      "Approach"
    );
  }

  // 4. Close gripper
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== CLOSE GRIPPER =========="
    );

    success = command_gripper(
      gripper_client,
      logger,
      0.7929,
      50.0
    );
  }

  // 5. Lift and transport the object
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== LIFT AND TRANSPORT =========="
    );

    // ================================================
    // LIFT AND TRANSPORT PARAMETERS
    // Only edit these three values after gripping.
    // Unit: meter. Frame: world.
    // ================================================
    transport_dx = -0.03;
    transport_dy = 0.0;
    transport_dz = 0.05;

    success = move_relative(
      move_group,
      logger,
      transport_dx,
      transport_dy,
      transport_dz,
      "Lift and transport"
    );
  }

  // 6. Release the object
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== RELEASE =========="
    );

    success = command_gripper(
      gripper_client,
      logger,
      0.0,
      50.0
    );
  }

  // 7. Automatically return to the work-start position
  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== RETURN TO WORK START =========="
    );

    // Return is calculated automatically:
    // Return = -(Approach + Transport)
    const double return_dx =
      -(approach_dx + transport_dx);

    const double return_dy =
      -(approach_dy + transport_dy);

    const double return_dz =
      -(approach_dz + transport_dz);

    RCLCPP_INFO(
      logger,
      "Automatic return: dX=%.3f dY=%.3f dZ=%.3f",
      return_dx,
      return_dy,
      return_dz
    );

    success = move_relative(
      move_group,
      logger,
      return_dx,
      return_dy,
      return_dz,
      "Return to work start"
    );
  }

  if (success)
  {
    RCLCPP_INFO(
      logger,
      "========== PICK AND PLACE DEMO SUCCEEDED =========="
    );
  }
  else
  {
    RCLCPP_ERROR(
      logger,
      "========== DEMO STOPPED BECAUSE A STEP FAILED =========="
    );
  }

  executor.cancel();

  if (spinner.joinable())
  {
    spinner.join();
  }

  rclcpp::shutdown();
  return success ? 0 : 1;
}
