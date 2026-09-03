# 每日操作與參數修改說明書

本頁只保留實際操作需要的步驟。預設環境為 Ubuntu UTM，workspace 位於 `~/ur5_ws`。

## 每次開機後啟動

### Terminal 1：啟動整套系統

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

保留這個 Terminal，不要關閉。等待 RViz 顯示 UR5＋Robotiq。

### Terminal 2：確認控制器

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

應看到以下四項都是 `active`：

```text
joint_state_broadcaster
joint_trajectory_controller
robotiq_activation_controller
robotiq_gripper_controller
```

### Terminal 2：執行任務

```bash
ros2 launch ur5_moveit_demo move_xyz.launch.py
```

成功時最後應顯示：

```text
PICK AND PLACE DEMO SUCCEEDED
process has finished cleanly
```

## 修改動作參數

開啟：

```bash
nano ~/ur5_ws/src/ur5_moveit_demo/src/move_xyz.cpp
```

### Approach：閉爪前的靠近

在 nano 按 `Ctrl+W`，搜尋：

```text
APPROACH PARAMETERS
```

穩定值為：

```cpp
const double approach_dx = 0.03;
const double approach_dy = 0.0;
const double approach_dz = 0.0;
```

三個方向都可以修改，不是只能改 X/Y。單位是公尺，例如 `0.03` 代表 3 cm。

### Lift and transport：閉爪後的搬運

搜尋：

```text
LIFT AND TRANSPORT PARAMETERS
```

穩定值為：

```cpp
const double transport_dx = -0.03;
const double transport_dy = 0.0;
const double transport_dz = 0.05;
```

這一段也可同時使用 X、Y、Z，例如向側邊移動並抬高。

### Return：放開後回工作起點

不需要手動修改第三組位移。程式會自動計算：

```text
return_dx = -(approach_dx + transport_dx)
return_dy = -(approach_dy + transport_dy)
return_dz = -(approach_dz + transport_dz)
```

因此通常只需修改 Approach 與 Transport 兩區。自動回程只保證回到 `test_configuration` 附近的笛卡兒位置與姿態；若中途改成不同座標基準或加入其他動作，公式也要重新設計。

### 夾爪位置與 effort

搜尋 `OPEN GRIPPER` 或 `CLOSE GRIPPER` 附近的 `command_gripper()`：

```text
0.0     = 張開
0.7929  = 關閉
50.0    = fake hardware 使用的 max_effort
```

若要模擬夾較大的物體，可先把閉合位置改為較小數值，例如 `0.4`，再逐步調整。Fake hardware 的 `50.0` 不是已校正的真實牛頓力，不能直接當成實體設定。

### 手臂速度與加速度

搜尋：

```cpp
setMaxVelocityScalingFactor(0.2)
setMaxAccelerationScalingFactor(0.2)
```

`0.2` 表示使用上限的 20%。改用實體機器人時應由低值開始，且需配合現場安全規範。

## 修改後重新編譯

儲存 nano：`Ctrl+O`、Enter；離開：`Ctrl+X`。

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select ur5_moveit_demo \
  --symlink-install

source install/setup.bash
```

保持 Terminal 1 的 bringup 正常，再於 Terminal 2 重跑：

```bash
ros2 launch ur5_moveit_demo move_xyz.launch.py
```

## 修改原則

- `dx, dy, dz` 目前以 `world` 座標表示，不是夾爪自身方向。
- 一次小幅調整，先從 1–3 cm 測試。
- Cartesian 完成率不足 99% 時，應縮小位移或改善姿勢，不要降低安全門檻。
- RViz 的格線不是實體地板，也不會自動形成碰撞限制。
- 實體機器人上線前，必須重新確認碰撞模型、TCP、速度、加速度、夾力與急停機制。

## 關閉

任務節點成功後會自行結束。完成所有工作時，在 Terminal 1 按 `Ctrl+C` 關閉 bringup。
