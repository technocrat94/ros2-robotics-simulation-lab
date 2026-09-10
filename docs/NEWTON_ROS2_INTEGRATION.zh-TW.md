# Newton 導入 ROS 2：學習筆記

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

### 3. 電腦等待時間不等於模擬時間

Shell 的 `sleep 1` 只保證命令等待約一秒。ROS discovery、程序排程和通訊也會耗時。需要精確實驗時，應用 Newton 的模擬時間或狀態條件作為停止標準。

### 4. 看見模型不代表模型行為正確

展開後的模型有 24 個 links 與 23 個 joints。UR5 有六個獨立關節；Robotiq 雖然列出六個旋轉關節，實際只有左 knuckle 是主關節，另外五個使用 `mimic` 跟隨。

生活化理解：Robotiq 像一個馬達拉動整套連桿。URDF 的 mimic 規則說明其他關節如何跟著主關節轉動。

Newton XPBD 不會自動保證這些 mimic 關節連動。若只看到模型成功載入就宣布完成，夾爪外觀可能正常，物理行為卻是錯的。

## 目前的工程決策

ROS 維持一個夾爪命令。Bridge 讀取 URDF 內每個 mimic 的倍率與偏移，替 Newton 設定所有跟隨關節。Newton 再計算接觸與物體是否被支撐。

後續必須確認：

- 每個 mimic 關節的正負方向、倍率與 offset；
- ROS 和 Newton 的 joint names、順序及單位；
- fake hardware 與 Newton 不會同時發布互相衝突的機器人狀態；
- Viewer、ROS topic 與 Newton 內部狀態彼此一致。

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
