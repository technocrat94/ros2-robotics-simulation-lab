# cuRoboV2 + Viser：UR5 平行實驗

這條實驗線保留已成功的 MoveIt + Newton FEM cable pick，不會直接修改或取代它。
第一階段驗證 cuRoboV2 能正確讀取相同的 UR5 + Robotiq 模型、起始關節、地板、牆壁和抓取目標，並在 Viser 中規劃及動畫播放抓取。

## 架構

```text
既有版本：抓取目標 -> MoveIt -> JointTrajectory -> ROS 2 controller -> Newton
新預覽版：抓取目標 -> cuRoboV2 (CUDA) -> Viser 軌跡動畫
新整合版：抓取目標 -> cuRoboV2 (CUDA) -> FollowJointTrajectory
          -> ros2_control -> 既有 ROS-Newton bridge -> Newton FEM cable
```

Viser 指令不會啟動 MoveIt、Newton 或實體 UR5，也不會發布
`JointTrajectory`。另外的 Step-C ROS 指令才會把軌跡送到 fake-hardware
controller 和 Newton；它仍不會控制實體 UR5。

## 為什麼不是複製整份 MoveIt 程式

`src/pick_at_position.cpp` 同時包含 MoveIt API、抓取參數、場景和執行控制。
如果整份複製，新的程式仍然依賴 MoveIt。這個實驗只重用它的資料契約：

- `urdf/ur5_robotiq.urdf.xacro`
- `config/jazzy_fem_initial_positions.yaml`
- `config/obstacles.yaml`
- `tool0` 和既有 pre-grasp 幾何參數

規劃器本身改由 cuRoboV2 提供。

## 執行

開一個名稱容易辨識的新 Terminal：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_curobo_v2_viser.sh
```

然後用瀏覽器開啟：

```text
http://localhost:8080
```

畫面中：

- UR5 + Robotiq 使用 Jazzy FEM 實驗的相同起始關節角。
- 橘色球是 cable pick center；為了容易看見，球的顯示半徑不是 cable 的物理半徑。
- `tool0` gizmo 是由舊 MoveIt 預設參數換算出的 pre-grasp target。
- 按下 **Plan + pick cable** 會重新在 CUDA 規劃 approach、grasp、lift，再播放有效軌跡。
- **Reset** 會回到 Jazzy FEM 的起始姿態。
- 地板和牆壁來自 `config/obstacles.yaml` 的 `step_c_cable_wall` profile。

目前的橘色 cable 是 visual attachment：抓取後會跟著夾爪抬升，方便檢查 cuRobo 軌跡與抓取方向；它沒有柔性、摩擦或接觸力。真正的 cable physics 仍要在後續接回 Newton。

按 `Ctrl+C` 關閉 Viser。

## 常用選項

只檢查 CUDA、模型與 FK，不開瀏覽器服務：

```bash
./scripts/run_curobo_v2_viser.sh --headless
```

在 headless 模式驗證完整三段抓取規劃：

```bash
./scripts/run_curobo_v2_viser.sh --headless --plan-pick
```

啟動 viewer 後自動播放一次抓取：

```bash
./scripts/run_curobo_v2_viser.sh --auto-pick
```

兩倍速度播放：

```bash
./scripts/run_curobo_v2_viser.sh --animation-speed 2
```

同時顯示 cuRobo collision spheres：

```bash
./scripts/run_curobo_v2_viser.sh --show-spheres
```

強制重新擬合 collision spheres：

```bash
./scripts/run_curobo_v2_viser.sh --rebuild-model
```

切換既有場景 profile：

```bash
./scripts/run_curobo_v2_viser.sh \
  --scene-profile step_c_cable_no_wall
```

## Step C：送到既有 Newton FEM endpoint

`curobo_step_c_pick.py` 不使用 MoveIt planner。它在 `cuda:0` 上以 cuRobo
規劃三段 arm path：start → pre-grasp、垂直下降、垂直抬升；之後透過既有
ROS 2 action server 執行：

```text
cuRobo approach -> FollowJointTrajectory
cuRobo descend  -> FollowJointTrajectory
Robotiq close   -> GripperCommand
cuRobo lift     -> FollowJointTrajectory
Robotiq open    -> GripperCommand
```

它沿用 C1 已通過的設定：

- `step_c_cable_wall`
- transit padding `0.020 m`、contact padding `0.000 m`
- 線材寬 `6 mm`、總壓縮 `2 mm`
- CSV 校正後夾爪命令約 `0.766352 rad`
- closure drop 補償後下降約 `0.091385 m`
- lift `0.120 m`
- 已驗證的下降與抬升執行時間

完整實驗仍需先在 `stepc` tmux session 啟動 bringup、Newton endpoint 和
ROS adapter。pick window 執行：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_step_c_curobo_pick_jazzy.sh
```

腳本會保留原本的 start-state guard 與 trajectory shadow，並產生 cuRobo
每段規劃時間、route 最高點、最小牆距，以及合併後的 C1 JSON。

## Cache 與 GPU

第一次執行約需 30 秒，在 GPU 上為 UR5 + Robotiq 的 collision meshes 擬合 spheres。
UR5 安裝面的 `base_link_inertia` spheres 會裁切在 `z=0`，避免球體擬合誤差讓固定底座穿入地板。
生成物位於：

```text
~/.cache/ur5_moveit_demo/curobo_v2/
```

之後只要 Xacro、cuRobo 版本與建模設定沒有變，就會直接重用 cache。
runner 會檢查 `nvidia-smi` 和 `torch.cuda.is_available()`；GPU 不可用時直接停止，不會悄悄改用 CPU。

## 為什麼 active joints 是 6，不是 7

展開後的 URDF 有六個 UR5 joints，再加上一個 Robotiq 主動 joint：

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
robotiq_85_left_knuckle_joint
```

cuRobo 第一階段只規劃手臂，因此用 joint name 將
`robotiq_85_left_knuckle_joint` 鎖在 `0.0 rad`（張開）。
這使規劃維度回到 UR5 的 6 DOF，也避免依靠容易出錯的數字 index。

## 下一步

目前已完成 collision-aware approach、grasp、lift 的 GPU 規劃、Viser 動畫，
以及 ROS/Newton 執行 client。正式驗證完成前仍不能宣稱 Newton C1 通過。
驗證工作依序是：

1. 在 `stepc` 的四個視窗跑完整 FEM cable 實驗。
2. 確認 `candidate_contact_grasp_pass=true`。
3. 確認 loaded／penetrating robot-wall contacts 都為零。
4. 保存結果、更新狀態並提交證據。
