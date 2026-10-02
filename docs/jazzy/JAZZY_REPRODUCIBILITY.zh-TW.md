# Humble 到 Jazzy 的 FEM 夾取重現說明

這份中文翻譯說明如何從 GitHub 原始碼重現原生 ROS 2 Jazzy FEM 地面夾取。Humble 只作為行為基準，不複製 Humble 的 build、install、log 或 virtualenv。

## Source of truth

GitHub 會包含實際修改過的 C++、Python、shell、launch、Xacro、YAML 與設定檔。`scripts/bootstrap_school_jazzy.sh` 會從 tracked prototypes 產生 runtime workspace、Jazzy URDF，建置四個 ROS packages，並驗證 mesh paths。

## 建置

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
git fetch origin
git switch jazzy-final-fem-pick
PIP_NO_INDEX=1 ./scripts/bootstrap_school_jazzy.sh
```

## 啟動

同一個 ROS domain 只能有一套 bringup、controller_manager、move_group、adapter 與 endpoint。

Terminal 1：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_ground_grasp_jazzy.sh
```

Terminal 2：

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 3：

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

Viewer 通常是 <http://127.0.0.1:30000/>。

## 預覽與完整夾取

預覽只做規劃，不會讓手臂動作：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_preview.sh
```

看到 `SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED` 後執行完整流程：

```bash
./scripts/run_jazzy_fem_pick.sh
```

完整流程會張爪、接近、閉合到約 `0.375145 rad`、以 scale `0.030` 抬升 `0.120 m`，再重新張爪。timestamped logs 會放在 `~/ur5_ws/run_logs/`。

## 驗收與排錯

必須看到 bridge `OK`、start-state guard pass、所有 Cartesian fractions `100.0%`、`ABSOLUTE POSITION PICK SUCCEEDED`，且沒有 named-start 或 `test_configuration`。viewer 中要確認膠條留在兩指之間上升，且只在開爪後掉落。

如果沒有動，通常是執行了 preview。如果出現 `execute_action_client_ client/server not ready`，請停止目前使用者多出的第二套 bringup/move_group，只保留同一個 isolated domain 的一套。
