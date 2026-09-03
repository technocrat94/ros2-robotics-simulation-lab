# UR5＋Robotiq 2F-85 MoveIt 專案

[English](README.md) | [繁體中文](README.zh-TW.md)

這是一個 ROS 2 Humble 學習專案：把 UR5 機械手臂與 Robotiq 2F-85 夾爪組成同一台機器人，再利用 `ros2_control` 控制關節、利用 MoveIt 規劃動作，完成一套可重複執行的模擬夾取流程。

目前使用 fake hardware 與 RViz。它可以驗證模型、控制器、規劃與指令流程，但不會模擬重力、摩擦、接觸力或物體真的被夾住的物理效果。

## 已驗證成果

2026-09-03 已在 RViz 完整驗證：四個控制器皆為 `active`，三段 Cartesian path 都達到 100%，程式正常結束。

```text
張開夾爪
→ 移到 test_configuration 工作基準姿勢
→ 直線靠近物體
→ 閉合夾爪
→ 後退並抬升
→ 放開夾爪
→ 自動回到工作基準姿勢
→ PICK AND PLACE DEMO SUCCEEDED
```

穩定版位移參數：

```text
靠近：       (+0.03, 0.00, 0.00) m
後退與抬升： (-0.03, 0.00,+0.05) m
自動回程：   ( 0.00, 0.00,-0.05) m
```

這些位移都以 MoveIt 的 `world` 世界座標表示，並保持夾爪姿態不變。

## 中文文件導覽

| 文件 | 適合什麼時候閱讀 |
|---|---|
| [完整學習歷程](docs/LEARNING_JOURNEY.md) | 想複習從 UTM、ROS 2、Robotiq 到 MoveIt 整合的完整過程 |
| [每日操作說明書](docs/OPERATIONS_GUIDE.md) | 每次開機、執行任務或修改位置與夾爪力道時 |
| [故障排除紀錄](docs/TROUBLESHOOTING.md) | 規劃失敗、RViz 沒有機械手、控制器異常或 Cartesian path 不完整時 |
| [Git 與 GitHub 說明](docs/GITHUB_GUIDE.md) | 想理解 Repository、branch、commit、tag、push 與 Deploy Key |

## 完成的系統整合

- 合併 UR5 與 Robotiq 的 Xacro/URDF 機器人模型
- 透過轉接座把夾爪接到 UR5 的 `tool0`
- 使用同一套 bringup 啟動手臂、夾爪、MoveIt 與 RViz
- 讓 `/joint_states` 同時包含 UR5 六個關節與夾爪關節
- 啟用 UR5 trajectory controller 與 Robotiq action controller
- 建立組合機器人的 SRDF、kinematics 與 controller mapping
- 使用命名姿勢 `test_configuration`
- 使用 Cartesian path 執行靠近、搬運與回程
- 透過 `GripperCommand` Action 控制張開與閉合
- 加入 99% 完成率、空軌跡與關節繞遠路保護
- 自動根據前兩段位移計算回程，不必維護第三組數字

## 專案檔案怎麼看

可以把整個 Repository 想成一本機械手臂專題報告：

| 路徑 | 中文用途 |
|---|---|
| `src/move_xyz.cpp` | 任務劇本：決定先做什麼、移動多少、何時開合夾爪 |
| `launch/ur5_robotiq_bringup.launch.py` | 總電源開關：一次啟動模型、控制器、MoveIt 與 RViz |
| `launch/move_xyz.launch.py` | 啟動任務程式所需的參數與節點 |
| `urdf/ur5_robotiq.urdf.xacro` | 身體構造圖：手臂、夾爪、link 與 joint 如何相連 |
| `srdf/ur5_robotiq.srdf.xacro` | MoveIt 動作規則書：規劃群組、命名姿勢與碰撞規則 |
| `config/combined_controllers.yaml` | `ros2_control` 實際啟動哪些控制器 |
| `config/controllers.yaml` | MoveIt 要把 trajectory 交給哪個控制器 |
| `config/kinematics.yaml` | 逆運動學求解器設定 |
| `rviz/view_robot.rviz` | RViz 的顯示方式與 Fixed Frame 設定 |
| `docs/` | 學習歷程、操作說明與除錯證據 |
| `CMakeLists.txt` | 編譯與安裝規則 |
| `package.xml` | ROS 套件名稱、版本與相依套件清單 |
| `.gitignore` | 告訴 Git 哪些建置結果與舊備份不應上傳 |

## 每次開機如何執行

Terminal 1 啟動整套系統：

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

Terminal 2 確認控制器：

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 control list_controllers
```

確認四個 controller 都是 `active` 後執行：

```bash
ros2 launch ur5_moveit_demo move_xyz.launch.py
```

完整步驟請閱讀[每日操作說明書](docs/OPERATIONS_GUIDE.md)。

## 修改夾取與搬運位置

編輯：

```bash
nano ~/ur5_ws/src/ur5_moveit_demo/src/move_xyz.cpp
```

搜尋 `APPROACH PARAMETERS` 修改閉爪前的靠近：

```cpp
const double approach_dx = 0.03;
const double approach_dy = 0.0;
const double approach_dz = 0.0;
```

搜尋 `LIFT AND TRANSPORT PARAMETERS` 修改閉爪後的搬運：

```cpp
const double transport_dx = -0.03;
const double transport_dy = 0.0;
const double transport_dz = 0.05;
```

X、Y、Z 都能修改，單位是公尺。回程由程式自動計算：

```text
Return = -(Approach + Transport)
```

修改後必須重新編譯：

```bash
cd ~/ur5_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select ur5_moveit_demo \
  --symlink-install

source install/setup.bash
```

## 夾爪與速度設定

目前 fake hardware 已驗證：

```text
張開位置：0.0
閉合位置：0.7929
max_effort：50.0
速度比例：0.2
加速度比例：0.2
```

Fake hardware 的 `max_effort=50.0` 不代表經過校正的實體夾力。若換成真實機器人，必須重新確認夾爪位置、力道、TCP、碰撞模型、速度、加速度與急停程序。

## 安全保護

程式會拒絕：

- 完成率低於 99% 的 Cartesian path
- 空白或格式錯誤的 trajectory
- 小幅直線動作中，任一關節總行程超過 1 rad 的路徑
- 被拒絕、stalled 或逾時的夾爪指令

RViz 格線只是視覺參考，不是真正的碰撞地板。實體部署前，需要在 Planning Scene 加入桌面、地板與物件。

## 目前限制與未來方向

目前尚未包含：

- 真實硬體控制
- 重力、接觸與摩擦物理
- 被夾物件的 attach/detach
- 桌面與地板碰撞物件
- 以夾爪 TCP frame 表達 Approach

未來預計在同一份學習作品集中加入 NVIDIA Newton Physics／模擬相關內容，但會以獨立目錄與 commit 保存，讓每個階段仍然清楚可追蹤。

## 版本資訊

- `main`：目前最新文件與程式
- `v0.1.0`：第一個已驗證成功的 UR5＋Robotiq 穩定程式版本
- Repository：目前設為 Private

Git/GitHub 的詳細解釋請閱讀 [Git 與 GitHub 說明](docs/GITHUB_GUIDE.md)。

## License

Apache-2.0，與 `package.xml` 設定一致。
