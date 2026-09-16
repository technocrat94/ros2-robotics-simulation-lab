# Newton 導入 ROS 2：學習筆記

> [English technical record](NEWTON_ROS2_INTEGRATION.md) · [中文主題索引](NEWTON_ROS2_LEARNING_LOG.zh-TW.md)

記錄日期：2026-09-10。

## 這一階段完成了什麼

我們已經讓 ROS 2 可以控制一個獨立的 Newton 物理程序，並將 Newton 算出的球體位置傳回 ROS。已驗證執行、暫停、重設與資料中斷偵測。

目前尚未把 UR5、Robotiq、MoveIt 或 `ros2_control` 接進 Newton。

## 為什麼要有 bridge

ROS 2 Humble 使用 Python 3.10，能正常執行 Newton solver 的隔離環境使用 Python 3.12。兩套環境像使用不同語言的兩個部門：bridge 負責翻譯命令與狀態，避免為了安裝其中一邊而破壞另一邊。

```text
ROS：請開始、暫停、重設
             ↓
           bridge
             ↓
Newton：計算重力與碰撞
             ↓
           bridge
             ↓
ROS：收到實際位置與模擬時間
```

## 我應該會判斷什麼

### 1. 命令不等於狀態

命令是「希望系統怎麼做」，狀態是「系統實際變成什麼」。服務回覆成功，只能證明 bridge 已送出命令；還要查看 Newton 回傳的狀態，才能確認效果。

### 2. 資料必須有新鮮度

Newton 停止後，bridge 回報 `STALE`。如果系統繼續把最後位置當成最新資料，MoveIt 或控制器可能在錯誤資訊上做決定。

Newton endpoint 也提供只綁定 loopback 的 Viser 畫面，而且刻意不放運動按鈕：動作只能從 ROS 發出，面板顯示 Newton 的模擬時間、實際高度、運行狀態、最後 ROS 指令與回傳序號。Viewer 用來觀察；topic 回傳與 `STALE` 故障測試才是整合驗收證據。

### 3. 電腦等待時間不等於模擬時間

Shell 的 `sleep 1` 只保證命令等待約一秒。ROS discovery、程序排程和通訊也會耗時。需要精確實驗時，應用 Newton 的模擬時間或狀態條件作為停止標準。

### 4. 看見模型不代表模型行為正確

展開後的模型有 24 個 links 與 23 個 joints。UR5 有六個獨立關節；Robotiq 雖然列出六個旋轉關節，實際只有左 knuckle 是主關節，另外五個使用 `mimic` 跟隨。

生活化理解：Robotiq 像一個馬達拉動整套連桿。URDF 的 mimic 規則說明其他關節如何跟著主關節轉動。

Newton XPBD 不會自動保證這些 mimic 關節連動。若只看到模型成功載入就宣布完成，夾爪外觀可能正常，物理行為卻是錯的。

## 已驗證：UR5＋Robotiq 靜態載入 Newton

第一次載入展開後的 URDF 時，Newton 得到 24 個 bodies、24 個 Newton joints，但 shapes 是 0。這代表關節拓樸已解析，`package://...` 指向的視覺與碰撞 mesh 卻沒有解析；因此當時還不能顯示或進行接觸模擬。

接著用 `ros2 pkg prefix --share` 找到兩個套件的實際位置，建立 Newton 專用的衍生 URDF，把 `package://ur_description/...` 與 `package://robotiq_description/...` 改為絕對路徑。ROS 的原始 Xacro 與 URDF 都沒有修改。第二次載入得到：

```text
Bodies: 24
Joints: 24
Shapes: 54
```

模型有 shape 後仍未立即出現在 Viewer，因為還需要執行 `newton.eval_fk()`，將關節座標轉換成每個連桿的世界座標。完成 FK 後，實際檢查確認 UR5 連桿連續、Robotiq 位於手腕末端，而且左右手指都存在。

本里程碑只證明靜態幾何與組裝可以進入 Newton。它尚未證明關節方向、mimic 連動、自碰撞、接觸、動力學或 ROS 對機器人的控制。下一個驗收項目是讓一個手臂關節與夾爪主關節運動，並核對所有跟隨關節的回傳值。

### 這次應記住的判斷

- body／joint 數量正確，只代表拓樸可讀。
- `Shapes: 0` 代表沒有可顯示或碰撞的幾何，不能宣稱模型載入成功。
- 畫面出現前需要 FK；幾何存在與世界座標初始化是不同步驟。
- 被動關節雖然沒有馬達，仍必須遵守機械連動關係。
- 靜態外觀正確仍不能證明運動或抓取正確。

## 目前的工程決策

ROS 維持一個夾爪命令。Bridge 讀取 URDF 內每個 mimic 的倍率與偏移，替 Newton 設定所有跟隨關節。Newton 再計算接觸與物體是否被支撐。

後續必須確認：

- 每個 mimic 關節的正負方向、倍率與 offset；
- ROS 和 Newton 的 joint names、順序及單位；
- fake hardware 與 Newton 不會同時發布互相衝突的機器人狀態；
- Viewer、ROS topic 與 Newton 內部狀態彼此一致。

## 身為工程師，我真正需要會什麼

我不需要背出每一行 bridge 程式，但必須能回答以下問題。

### 1. 定義成功，而不是只說「可以動」

先把任務改寫成可以量測的條件。例如夾爪閉合不等於抓取成功；物體必須離開支撐面、在搬運期間保持住，而且相對滑移不能超過事先定義的門檻。

### 2. 找出系統中的事實來源

MoveIt 的軌跡是目標，Newton 算出的關節與物體狀態才是模擬結果。Newton 接管機器人後，fake hardware 不能同時發布另一套 `/joint_states`，否則 ROS 會收到互相矛盾的狀態。

### 3. 檢查介面的契約

兩個系統連接時，要逐項確認：

- 名稱：ROS 與 Newton 的 joint name 是否完全一致；
- 單位：旋轉關節用 rad、直線位移用 m、力矩用 N·m；
- 座標系：數值是相對 `world`、base 還是 tool；
- 順序：陣列中的第幾個值對應哪個關節；
- 更新率與逾時：多久沒有新資料要判定為 `STALE`；
- 控制語意：傳的是位置目標、速度、力，還是實際量測值。

### 4. 分清楚四種時間

- wall time：現實世界經過多久；
- ROS time：ROS 節點使用的時鐘；
- simulation time：Newton 模擬世界經過多久；
- physics `dt`：求解器每一次前進的時間間隔。

它們不一定相等。精確實驗應依 simulation time 或狀態條件停止，不能把 `sleep 1` 當成精確的一秒模擬。

### 5. 用證據區分故障位置

| 現象 | 優先檢查的邊界 |
|---|---|
| ROS service 不存在 | ROS adapter 是否啟動、workspace 是否 source |
| service 回覆成功，但狀態不變 | 命令是否到達 Newton、是否收到 acknowledgement |
| Newton 狀態改變，但 ROS 沒更新 | bridge 回傳路徑、topic type、資料是否 `STALE` |
| 手臂動錯關節或方向 | joint names、陣列順序、單位、axis |
| 夾爪只有一部分動 | mimic multiplier／offset 是否保留 |
| Viewer 與 ROS 顯示不同 | 是否讀到不同程序、重複 publisher、frame 或時間不同 |
| 抓取結果不穩定 | 接觸參數、控制輸入、`dt`、solver iterations 與初始位置 |

### 6. 知道模擬可以證明到哪裡

Fake hardware 可以驗證 ROS 指令流程，不能驗證重力或抓取。Newton 剛體接觸可以研究摩擦與滑移，但未經校準不能直接宣稱真實 Robotiq 需要多少夾力。實體安全、材料變形、感測誤差和 sim-to-real 仍需要另外驗證。

### 7. 設計能推翻自己想法的測試

工程實驗不是找一張支持直覺的畫面。應固定其他條件、一次改一個變因，並加入故障測試。例如停止 Newton，確認 ROS 是否回報 `STALE`；這比只測正常運作更能證明系統設計。

## 可以交給 AI 與不能交出的責任

可以交給 AI：產生程式、修改設定、執行重複測試、整理 log、繪圖與格式化文件。

工程師仍需負責：定義需求、選擇架構與事實來源、確認介面契約、判斷實驗能否區分原因、拒絕超過證據的結論，以及向教授或團隊清楚說明限制。

## 我的工程判斷筆記格式

每個里程碑只要回答六項：

1. **Objective：**要解決什麼問題？
2. **Success criterion：**哪些量測結果才算成功？
3. **System boundary：**命令與狀態經過哪些元件？
4. **Evidence：**實際觀察到什麼？
5. **Limitations：**目前不能證明什麼？
6. **Next decision：**下一個測試要排除哪一種不確定性？

## 教授可能會問

**問：為什麼不用同一個 Python 程序？**

答：已驗證的 ROS 與 Newton 環境使用不同 Python 版本。分離程序可以保護依賴並讓故障邊界清楚。

**問：怎麼證明已經整合，而不是兩邊各自運作？**

答：從 ROS 發出命令，確認 Newton 狀態改變，再確認相同狀態回到 ROS；停止 Newton 後，ROS 必須偵測到資料過期。

**問：模型載入成功，為什麼還不能說 Robotiq 可用？**

答：載入只證明格式可讀。還需驗證 mimic 關節、控制單位、接觸與回傳狀態。

## 可放進英文報告的句子

> I verified a bidirectional command-and-feedback path between ROS 2 Humble and Newton while keeping their Python environments isolated.

> Import success alone is insufficient evidence of correct robot behavior; the Robotiq mimic-joint coupling must also be preserved and verified.

> Simulator failures are reported as stale data instead of allowing the last state to appear current.

## 貢獻說明

Codex 建立 bridge 程式並整理文件。專案負責人實際執行建置與驗證指令、回傳輸出，並參與判讀結果。UR5／Robotiq 導入 Newton 仍是下一階段工作。

## 里程碑 6：ROS 控制 UR5＋Robotiq 運動學展示（2026-09-11）

ROS 呼叫既有的 `/newton/set_running` 後，Newton 執行八秒的 UR5 移動、夾爪閉合、手腕轉動與返回流程。Newton 將 12 個旋轉關節的名稱與角度回傳 `/newton/joint_states`。實測暫停在 `GRIPPER_CLOSE` 時，主關節為 `0.4815 rad`，五個 follower 分別依 `+1` 或 `-1` 倍率跟隨，`mimic_max_error` 為 `0.0 rad`。

### 程式原理：直接給角度，再做正向運動學

目前程式沒有給末端目標位置，也沒有執行逆向運動學。它依模擬時間直接算出關節角度 `q(t)`，寫入 Newton 的 `joint_q`，再呼叫 `newton.eval_fk()`，由正向運動學算出所有連桿在世界座標中的姿態。

```text
時間 t → 預先指定的關節角度 q(t) → FK → 各連桿姿態
```

角度換算公式是 `degree = rad × 180 / pi`。初始六軸角度為 `(0, -90, +90, -90, -90, 0)` 度。

| 模擬時間 | 階段 | 實際指定的變化 |
|---|---|---|
| 0–2 秒 | `ARM_APPROACH` | shoulder pan：`0 → +0.55 rad`（`+31.5°`）；elbow：`+1.571 → +1.221 rad`，變化 `-0.35 rad`（`-20.1°`） |
| 2–4 秒 | `GRIPPER_CLOSE` | 夾爪主關節：`0 → +0.70 rad`（`+40.1°`），其他五軸依 mimic 公式跟隨 |
| 4–6 秒 | `WRIST_MOTION` | wrist 3：`0 → +0.65 → 0 rad`，最大 `+37.2°` |
| 6–8 秒 | `RETURN_AND_OPEN` | 上述手臂關節回到初始角度，夾爪回到 0 |

靠近與返回使用 `s = 3u² - 2u³` 的 smoothstep，使指定位置曲線在一段動作的起點與終點斜率為零。這只是讓角度命令平順，沒有計算馬達力矩，也不能證明真實控制器跟得上。

這次只有 shoulder pan、elbow、wrist 3 與夾爪主關節在動。Shoulder lift、wrist 1、wrist 2 保持固定，因此不能說「六軸都測試正常」。能宣稱的是：被測關節的名稱、索引、方向與單位有初步證據，FK 畫面、mimic 映射及 ROS 狀態回傳一致。

### MoveIt 在前一個專案中負責什麼

MoveIt 接收末端姿態或關節目標，透過運動學與規劃元件產生軌跡，並檢查 planning scene 中的碰撞，再把軌跡交給 `ros2_control`。先前使用 fake hardware，所以驗證了規劃、控制器介面與任務順序，沒有驗證物體接觸。

目前 Newton 展示沒有呼叫 MoveIt、IK 或 path planner；它直接指定 `q(t)`，目的是先排除 bridge、joint mapping、FK 與 mimic 的問題。下一階段才是把 MoveIt 產生的 joint trajectory 送到 Newton 的 actuator target，並讓 Newton 回傳唯一的模擬狀態。

可以用一句話區分：**MoveIt 決定希望機器人沿哪條軌跡走；Newton 應負責計算在物理條件下實際走成什麼樣。**

### 我如何從程式確認上述說法

不用背程式，但要會找到證據：

- `newton_robot_endpoint.py` 的 `update_pose()` 直接寫 `arm[0]`、`arm[2]`、`arm[5]` 與 `grip`，所以輸入是關節角度，不是末端 pose。
- 同一函式最後呼叫 `newton.eval_fk()`，所以目前是由關節角度推算連桿姿態的 FK。
- 程式沒有呼叫 MoveIt、IK solver 或 trajectory planner，因此不能說這段動作是規劃出來的。
- `ros_adapter.py` 將 `joint_names` 與 `joint_positions` 包成 `/newton/joint_states`，所以可以從 ROS 核對 Newton 回傳的狀態。
- `publish_status()` 使用封包年齡判定 `OK` 或 `STALE`，所以畫面停止與通訊中斷能被區分。

英文文件會保留這類實作片段、檔案責任、測試證據與限制；本中文文件則保存你需要理解及口頭回答的判斷方法。

### 工程師應該會判斷什麼

1. **看到機器人移動，不代表動力學正確。** 本次直接指定 joint position 並用 FK 更新連桿，是運動學介面測試。
2. **命令成功與狀態成功必須分開驗證。** ROS service 是輸入；`/newton/joint_states`、phase 與 mimic error 才是回傳證據。
3. **`mimic_error = 0` 只證明數學映射一致。** 它不能證明手指接觸力、摩擦或物體抓取正確。
4. **測試 topic 不是正式狀態來源。** 目前使用 `/newton/joint_states`，尚未取代 fake hardware 的 `/joint_states`，因此不應同時宣稱 Newton 已接管 MoveIt。
5. **展示範圍必須能一句話說清楚。** 已驗證 ROS 命令、FK、mimic 與回傳；尚未驗證動態控制、碰撞抓取及真實硬體。

### 教授可能會問

**問：這算 ROS 2 與 Newton 整合完成了嗎？**

答：已完成可觀察的命令與狀態橋接，也完成機器人運動學展示；MoveIt 控制、動力學接觸與正式 `/joint_states` ownership 尚未完成。

**問：為什麼 `mimic_error = 0` 還不能證明夾得住物體？**

答：mimic error 只比較 follower joint angle 是否符合 URDF 公式。抓取還取決於碰撞幾何、摩擦、接觸力、物體質量與運動加速度。

**問：這次最重要的架構判斷是什麼？**

答：重用已驗證的 ROS–Newton bridge，只替換 Newton 端模型與狀態內容；同時保留 namespaced topic，避免和現有 fake hardware 爭奪狀態來源。

**問：這段動作有用逆向運動學嗎？**

答：沒有。程式直接定義關節角度隨時間的變化，再用 FK 更新連桿姿態。它適合驗證介面與關節對應，不能代表 MoveIt 規劃或動態軌跡追蹤已完成。

**問：`mimic_error = 0`，但畫面上手指互相穿過，模型算正確嗎？**

答：不算。零誤差只證明 follower angle 符合代數公式；還必須檢查 joint axis、origin、mesh、collision geometry 與 self-collision 設定。

## 里程碑 7：MoveIt 介面盤點與起始姿勢安全檢查（2026-09-17）

目前啟用的手臂控制器是 `joint_trajectory_controller`，MoveIt 透過 `FollowJointTrajectory` action 傳送六軸軌跡。六個關節的名稱與順序和 Newton 前六個手臂座標一致。軌跡點的 `positions` 是弧度；`time_from_start` 是從整條軌跡起點算起的累積到達時間，不是每段各自需要的時間。

控制器狀態中的 `desired` 是希望到達的位置，`actual` 是系統回報的目前位置，`error` 約等於兩者之差。目前使用 fake hardware，所以零誤差可能只是命令值被直接當成回報值，不能證明物理系統真的跟得上。

實際盤點發現：ROS 起始姿勢是 `(0, -1.57, 0, -1.57, 0, 0)` rad；Newton 舊展示是 `(0, -pi/2, +pi/2, -pi/2, -pi/2, 0)` rad。`elbow_joint` 與 `wrist_2_joint` 都相差 `pi/2 rad`，也就是 90 度。如果直接接線，這兩個關節可能瞬間跳動。

因此新增 **start-state guard（起始姿勢安全檢查器）**。它從 `/joint_states` 讀取 ROS 姿勢，再和 Newton 起點比較；允許誤差暫定為 `0.2 rad`（約 11.5 度）。實測正確得到：

```text
pass=false
maximum_error_rad=1.5707963267948966
mismatches=[elbow_joint, wrist_2_joint]
```

這裡的 `false` 是成功的安全結果，代表系統拒絕不連續的起點。可重用的 trajectory contract 也會檢查關節缺漏或重複、非有限數值、每點資料數量、時間是否嚴格遞增、關節順序與起點連續性。下一階段才會讓 Newton 依 `time_from_start` 執行 MoveIt 軌跡並回傳真正的 `actual`。

### 這階段要會說的一句話

**MoveIt decides where the UR5 should move; Newton predicts what physically happens; both must agree on the robot's current state before execution.**

MoveIt 決定 UR5 應該怎麼走，Newton 預測物理上會發生什麼；執行前兩者必須對機器人目前姿勢有相同認知。
