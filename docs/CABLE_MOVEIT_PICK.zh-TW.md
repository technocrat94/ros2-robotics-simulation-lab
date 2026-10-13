# Rod cable × MoveIt 初次夾取

> 此 rigid capsule-chain 路徑目前只保留作對照實驗。正式細線夾取改用
> [FEM cable × MoveIt](FEM_CABLE_MOVEIT_PICK.zh-TW.md)，以保留 VBD 柔體變形與
> full-surface contact。

這條路徑是獨立實驗，不取代已通過的 Jazzy FEM ground-pick baseline。
舊的 `run_*fem*` 腳本與 FEM physics/grasp 參數都保留不變。

## 整合了什麼

Newton 場景新增 `GRASP_OBJECT_MODEL=rod_cable`：

- 長度 `0.40 m`、直徑 `0.006 m`、40 段；
- 使用 cantilever 驗收通過的伸長、剪切、彎曲、扭轉與原始阻尼參數；
- cable 中心位於 `(0.4869, 0.10915)`，沿世界座標 `+x` 平放；
- 初始中心線 `z=0.004 m`，等於半徑 `3 mm` 加 contact gap `1 mm`；
- 根部不固定，整條 cable 可以被手指接觸、抬起及釋放；
- 實際中心位置由 Newton 經 `/newton/object_pose` 傳給 MoveIt。

MoveIt 不負責 cable 形變。它只規劃 UR5/Robotiq 的安全路徑；Newton
負責 cable、地面、手指接觸和摩擦。兩邊透過既有 ROS–Newton bridge
交換機器人命令與物件量測。

## 為什麼不能沿用 50 mm 的夾爪角度

已驗證 FEM strip 寬 `50 mm`，細 cable 只有 `6 mm`。原本約 `0.375 rad`
的閉合命令會留下接近 `48 mm` 的 pad gap，根本碰不到 cable。

校正表已用相同 URDF 運動學延伸至 `0.80 rad`。初次測試使用：

- object width：`6 mm`；
- compression：`0 mm`，先不假設 cable 可以被壓扁；
- 目標 pad gap：`6 mm`；
- 由表格在 `0.70` 與 `0.75 rad` 之間內插；
- 閉合造成的 fingertip midpoint 高度變化也會補償 approach distance。

這仍只是運動學預測。是否真正夾住，必須以 Newton 左右兩側的 loaded
contact、實際 lift 和 release drop 判斷，不能只看 MoveIt success。

## 一次性同步與建置

先確定沒有其他實驗正在使用相同 ROS domain 或連接埠，然後執行：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/bootstrap_school_jazzy.sh
```

這一步會把版本庫中的 cable builder、endpoint 複製到 runtime workspace，
並重新建置含延伸夾爪校正表的 MoveIt node。它不會啟動模擬。

## 四個 terminal

### Terminal 1：MoveIt、controller 與 RViz

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

### Terminal 2：ROS–Newton adapter

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

### Terminal 3：依電腦選擇固定執行環境

家裡 ARM64 UTM 固定使用 CPU：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_cable_grasp_home_cpu.sh
```

應顯示 `CABLE_RUNTIME_PROFILE home-cpu`、`NEWTON_DEVICE cpu` 與 localhost
viewer URL。CPU 物理步進會比真實時間慢。

學校 RTX 電腦固定使用 GPU：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_cable_grasp_school_gpu.sh
```

應顯示 `CABLE_RUNTIME_PROFILE school-gpu` 與 `NEWTON_DEVICE cuda:0`。如果
NVIDIA driver 無法使用，學校啟動器會直接停止，不會靜默退回 CPU。

### Terminal 4：一定先做 plan-only preview

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_cable_preview.sh
```

只有在下列條件都成立時才執行真正夾取：

- `START_STATE_GUARD pass=true`；
- `SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED`；
- RViz 路徑沒有穿過地板；
- Newton viewer 中 cable 位於兩指正下方；
- 沒有第二套 bringup、adapter 或 endpoint。

真正執行：

```bash
./scripts/run_jazzy_cable_pick.sh
```

## 驗收結果

結束 Terminal 3 時，endpoint 會輸出 `MOVEIT_GRASP_RESULT`。初次 cable
候選成功至少需要：

- `closed_seen=true`、`lift_started=true`、`release_seen=true`；
- grasp-region lift 至少 `0.08 m`；
- release drop 至少 `0.05 m`；
- 閉合階段左右 loaded rigid contact 都大於零；
- 抬升階段左右 loaded rigid contact 都大於零；
- `bilateral_lift_rigid_contact_samples > 0`；
- 無 command queue overflow、mimic error 合格且 `finite_state=true`。

`candidate_contact_grasp_pass=true` 才表示物理量測也通過。第一次失敗時先保存
完整 JSON 與 viewer 現象，不要立即增加摩擦、阻尼或夾爪壓縮量；先判斷是路徑、
高度、接觸幾何，還是抓取後滑落。

## 2026-10-08 第一次整合結果：未通過

第一次 CUDA 整合執行中，MoveIt 的 safe pre-grasp、approach、lift 都完成
`100%`，最後也印出 `ABSOLUTE POSITION PICK SUCCEEDED`；但操作者看到 cable
沒有被保留並抬起，因此不能算成功。

Newton endpoint log 顯示：

- 共有 71 筆左右兩側同時 loaded 的 rigid-contact samples；
- 抓取區最高只由約 `3.0 mm` 上升到 `12.56 mm`，約上升 `9.56 mm`；
- commanded lift 是 `120 mm`，驗收最低需求是 `80 mm`；
- contact 在 lift 初期消失，後續夾爪繼續上升但 cable 回到地面；
- endpoint 沒有印出最後的 `MOVEIT_GRASP_RESULT`。

這證明路徑規劃成功與物理夾取成功是兩件事。下一輪應先做 close-only／early-lift
診斷，檢查 finger pad 接觸位置與垂直抓取高度，不應先任意增加摩擦、阻尼、壓縮量或
cable stiffness。版本化證據在
[`experiments/newton-cable-grasp/results/`](experiments/newton-cable-grasp/results/README.md)。

## 2026-10-10 單一變因修正：指尖解析碰撞代理

檢查程式後發現，`rod_cable` 雖然已用 40 段 capsule chain 簡化細線，指尖卻仍以
Robotiq 的複雜 mesh 與 cable 做剛體接觸；先前成功的 FEM strip 才有四個解析 box
proxy。因此這次先不改摩擦、壓縮、路徑或 cable 材料，只修正接觸幾何：

- 保留 Robotiq mesh 作為畫面外觀；
- 停用四個 finger/fingertip mesh 的 rigid-shape collision；
- 依各 mesh 邊界建立四個隱形 analytic box proxy；
- proxy 與 cable 保持碰撞，但排除 proxy 與機器人本體、proxy 彼此的自碰撞；
- 家用 CPU 與學校 GPU 啟動器都使用同一份修正，不需手動改程式。

啟動時 `MOVEIT_GRASP_ENDPOINT_READY` 應由 `robot_shapes=54` 變為
`robot_shapes=58`。這只證明四個代理已載入；是否解決滑落仍須由
`grasp_region_lift_rise_m`、左右 loaded contact 與 release drop 驗證，不能只看動畫。

第一次 proxy 測試把每個 box 向內縮 `1 mm`，但 cable 直徑與目標 pad gap
同為 `6 mm`；兩側共增加 `2 mm` 開口後，整段量到的左右 loaded contact 都是零。
因此 rod proxy 改為不內縮，並明確套用 cable 的 `1 mm` contact gap、接觸剛性與
阻尼。FEM strip 仍保留已驗證的 `1 mm` proxy inset。這次只修正「代理沒碰到」；
接觸恢復後若仍在抬升時滑落，才進一步比較壓縮量與摩擦係數。
