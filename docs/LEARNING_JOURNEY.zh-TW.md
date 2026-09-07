# 學習歷程：從 UTM 到 UR5＋Robotiq 整合

## 專案目標

這個專案不是單純讓 RViz 出現一支機械手臂，而是把原本各自運作的 UR5、Robotiq 2F-85、`ros2_control` 與 MoveIt 接成一套可重複執行的系統。

最終流程為：

```text
張開夾爪
→ UR5 前往工作基準姿勢
→ 直線靠近
→ 閉合夾爪
→ 後退並抬升
→ 放開夾爪
→ 自動回到工作基準姿勢
```

## 1. 建立開發環境

開發環境是 macOS 上的 UTM 虛擬機，虛擬機內使用 Ubuntu 22.04（aarch64）、ROS 2 Humble 與 MoveIt 2。

可以把這個結構想成「Mac 是教室，UTM 是實驗桌，Ubuntu 才是實際放置 ROS 工具的抽屜」。程式、ROS 節點與控制器都在 Ubuntu 內執行，Mac 主要負責顯示與遠端協作。

這一階段學到：

- ROS workspace 的 `src`、`build`、`install`、`log` 各自負責什麼
- 每次新 Terminal 都要 `source` ROS 與 workspace 環境
- 套件原始碼與編譯後安裝結果是兩個不同位置
- 修改 C++ 後必須重新 `colcon build`，再重新 `source install/setup.bash`

## 2. 先單獨驗證 Robotiq 夾爪

整合前先確認夾爪能獨立工作。這就像組裝電腦前，先確認電源供應器本身能正常供電；若單一零件尚未通過測試，直接組合只會讓問題更難找。

驗證結果：

- `robotiq_gripper_controller` 為 `active`
- `robotiq_activation_controller` 為 `active`
- `/robotiq_gripper_controller/gripper_cmd` Action Server 存在
- `position=0.0` 可張開
- `position=0.7929` 可閉合
- `/joint_states` 能讀到 `robotiq_85_left_knuckle_joint`

因此確認問題不在夾爪驅動，而是在後續的模型、規劃或控制整合。

## 3. 把 UR5 與 Robotiq 組成同一個機器人

建立 `ur5_robotiq.urdf.xacro`，將 UR5、轉接座與 Robotiq 2F-85 組成同一棵 TF／關節樹，並把夾爪接到 UR5 的 `tool0`。

URDF/Xacro 可以想成機器人的「身體構造圖」：有哪些骨頭（link）、關節（joint），以及它們如何連接。若手臂與夾爪使用兩張彼此無關的構造圖，MoveIt 就不會把它們視為同一個機器人。

曾遇到 Xacro 的 `Undefined substitution argument name`。處理方式是補齊需要的 Xacro argument/default，先輸出 `/tmp/ur5_robotiq.urdf`，再確認產生結果同時包含：

- UR5 的 `tool0`
- Robotiq 的 `robotiq_85_base_link`
- UR 與 Robotiq 的 `ros2_control` 定義

## 4. 整合 ros2_control 與 joint states

建立統一 bringup 與 controller 設定，讓同一個 Controller Manager 管理：

- `joint_state_broadcaster`
- `joint_trajectory_controller`
- `robotiq_activation_controller`
- `robotiq_gripper_controller`

整合後，`/joint_states` 同時包含 UR5 六個關節與夾爪關節。這相當於把「手臂的儀表板」和「夾爪的儀表板」合併，MoveIt 才能取得一份完整且一致的目前狀態。

## 5. 讓 MoveIt 正確理解這台組合機器人

URDF 描述身體，SRDF 則像「動作規則書」：哪些關節屬於 `ur_manipulator`、有哪些命名姿勢，以及哪些相鄰零件的接觸可忽略。

早期規劃到 `test_configuration` 時失敗並回傳 MoveIt error `-26`。檢查後改用本專案的組合 SRDF、kinematics 與 controller mapping，不再讓任務節點讀取只描述原始 UR 的語意設定。結果從「Planning request aborted」進步為規劃與執行成功。

這裡也釐清了兩個不同階段：

- Planning 成功：導航軟體找到一條路
- Execution 成功：車子真的沿那條路開完

因此「能規劃但 Execute aborted」不能算任務成功，還要確認 MoveIt 使用的 controller 名稱與實際 active controller 相同。

## 6. 理解 RViz 顯示與座標系

RViz 的 Fixed Frame 曾設為不存在的 `map`，畫面因此沒有機械手。改用系統實際存在的 `world` 後即可顯示。

RViz 中的橘色手臂通常是 MoveIt 的目標／規劃狀態，灰色手臂是目前狀態。橘色模型不消失不代表多出一支真實手臂，也不代表執行失敗。

目前 `dx, dy, dz` 使用 `world` planning frame：

```text
+X/-X、+Y/-Y：世界座標的水平／側向方向
+Z：世界座標向上
-Z：世界座標向下
```

這不是夾爪自己的前、後、左、右。就像「往教室北方走」和「依照自己面向往前走」是兩套不同指令。未來若要讓 Approach 永遠沿著夾爪正前方，就要加入 TCP/tool frame 轉換。

## 7. 從一般 Pose Planning 改成受保護的 Cartesian Motion

最初使用單一 Pose Target 時，末端看似只需移動一小段，但規劃器可能選擇關節繞遠路，畫面出現接近 360 度旋轉。這不是理想的工業動作。

後來改用 `computeCartesianPath()`，要求末端沿直線插值並保持姿態，再加入三層保護：

1. Cartesian 完成率必須至少 99%
2. trajectory 不可為空或格式錯誤
3. 小幅直線動作中，任一關節總行程超過 1 rad 就拒絕執行

這像要求服務生端著一杯水直線送到桌邊，不能為了到達同一個終點先繞餐廳一圈。

測試也證明安全門檻有效：較大的位移曾只完成 12.5% 或 93.4%，程式選擇停止，沒有勉強執行不完整軌跡。

## 8. 將任務參數集中並自動計算回程

為了避免每次修改都在多個程式區塊尋找數字，將 Approach 與 Lift/Transport 參數集中放在 `main()` 前段。

穩定參數：

```text
Approach:           (+0.03, 0.00, 0.00) m
Lift and transport: (-0.03, 0.00,+0.05) m
```

回程不需要另外手動維護，而是用向量相加自動計算：

```text
Return = -(Approach + Transport)
       = (0.00, 0.00, -0.05) m
```

這像記帳：先向東走 3 公尺，再向西走 3 公尺並上樓 5 公尺；程式把總位移結算後，自動算出只需下樓 5 公尺。只要前兩段都使用相同的 `world` 座標基準，就不必同步修改第三組數字。

## 9. 最終驗證成果

2026-09-03 的完整驗證結果：

```text
四個 controllers：active
Move to test_configuration：success
Approach Cartesian path：100.0%
Lift and transport Cartesian path：100.0%
Return Cartesian path：100.0%
Gripper open/close/release：success
PICK AND PLACE DEMO SUCCEEDED
process has finished cleanly
```

## 從困難中建立的能力

這份成果展示的不只是 ROS 指令操作，也包含：

- 將多個 ROS 2 套件整合成單一可啟動系統
- 閱讀 node、topic、action、controller 與 joint-state 狀態
- 區分模型錯誤、規劃失敗與控制執行失敗
- 修改 URDF/Xacro、SRDF、YAML、Python launch 與 C++ MoveIt 程式
- 用可重現的測試逐層縮小問題範圍
- 為危險或不完整軌跡加入 fail-safe，而不是降低門檻強迫執行
- 使用 Git commit 與版本標籤保存可回復的穩定里程碑

## 下一步

目前成果是 fake hardware 上的運動控制驗證。若繼續發展，可依序加入：

1. Planning Scene 的桌面、地板與物件碰撞模型
2. 夾取物件的 attach/detach 狀態
3. 以 TCP/tool frame 表達 Approach
4. Gazebo 或其他物理模擬
5. 實體 UR5 與 Robotiq 的速度、力道、網路及安全重新校正

這些限制被明確寫下，是工程可信度的一部分：知道模擬已證明什麼，也知道它尚未證明什麼。

## 10. Mac mini 移轉驗證

記錄日期：2026-09-07；提供的 demo 日誌日期為 2026-09-06。
完整技術紀錄請見英文版第 10 節，本節為中文摘要。

### 完成與驗證

- 移轉到 Mac mini M4 後，UTM Ubuntu 可正常啟動與登入。
- 確認 aarch64、ROS 2 Humble、磁碟剩餘 38 GB，網路測試成功。
- 既有工作區增量編譯成功，四個控制器全部為 active。
- 手臂成功到達 test_configuration。
- 靠近、抬升搬運、返回三段 Cartesian 路徑皆為 100%，執行成功。
- 夾爪完成張開、閉合與放開，並在 RViz 觀察到手臂及夾爪動作。
- demo 顯示 PICK AND PLACE DEMO SUCCEEDED，程式正常結束。
- 本機 e79ad6f 與 v0.1.0 的差異只有文件。
- 已建立本機回復分支 backup/mac-mini-verified-e79ad6f。

### 更正第 7、8 節的描述

目前的 1 rad 保護只比較軌跡起點與終點的關節角度差，
沒有累加中途行程，不能保證攔下繞遠路後又回到附近的軌跡。
格式檢查只涵蓋空軌跡與首尾位置向量長度，並非全面驗證。

Approach 與 Transport 數值位於各自的執行區塊；
夾爪與速度設定仍分散，尚未完成單一設定區塊。
返回位移已依照前兩段位移自動計算。

### 學習與限制

移轉驗證需要逐層檢查作業系統、工作區、編譯、控制器與完整動作，
也要對照實際程式修正文件，區分已完成成果與後續改善。

本次是一次 fake-hardware 成功執行，不是乾淨安裝或多次穩定性測試，
也未驗證真實抓取、接觸物理或實體硬體。節點實際時間設定仍待確認。
本次版本比較依據本機 Git，尚未重新連線核對 GitHub 最新狀態。

使用者執行指令並觀察 RViz；Codex 協助安排檢查、解讀輸出、
閱讀程式與起草紀錄。本次驗證沒有修改機器人原始碼。
