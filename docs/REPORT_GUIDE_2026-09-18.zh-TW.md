# 2026-09-18 專題報告操作單

報告時直接開啟 GitHub `README.md`，依照下列順序往下捲動。主畫面使用英文，口頭用中文解釋；粗體英文句可以直接作為英文摘要。

## 1. 開場：專題問題

> **How can a planned UR5 motion be evaluated against physical effects that fake hardware does not represent?**

我的原始 ROS 2 專案已經能用 MoveIt 規劃 UR5 與 Robotiq 動作，但 fake hardware 不包含重力、接觸、摩擦、滑落與柔性物體。因此本週的工作是導入 Newton，並建立一條可以驗證命令與回傳狀態的橋接路徑。

## 2. 系統分工

- **UR5**：被控制的六軸機器人。
- **MoveIt 2**：做逆向運動學、碰撞檢查及路徑規劃。
- **ROS 2 controller**：執行 MoveIt 軌跡。
- **Newton**：描述重力、接觸、摩擦、柔性物體與機器人物理反應。
- **Bridge**：翻譯兩個 Python 環境並回傳可檢查的狀態。

一句話：**MoveIt decides where the robot should move; Newton predicts what physically happens.**

## 3. 本週成果順序

1. ROS 能開始、暫停及重設 Newton，Newton 能回傳位置、模擬時間與資料新鮮度。
2. UR5＋Robotiq 成功匯入 Newton：24 bodies、24 joints、54 shapes。
3. Robotiq 使用一個 leader 與五個 mimic followers；`mimic error = 0` 只證明映射一致。
4. 建立 FEM 柔性膠條，再建立較快的 compliant-joint segmented model。
5. 失敗抓取顯示邊緣力矩、碰撞範圍與假成功指標會誤導判斷。
6. 中心摩擦抓取成功；零摩擦對照組掉落，證明結果不是隱藏 attachment。
7. 抓取通過約 180 度手腕壓力動作，並在張開夾爪後掉落。
8. MoveIt 影子執行成功：完整任務傳到 Newton，最終最大角度差約 `3.4e-8 rad`。

## 4. 播放媒體

在 README 的 **Report media** 依序點開：

1. FEM rubber-strip oscillation：說明連續柔性模型。
2. Segmented-strip oscillation：說明剛體節段＋柔性關節的快速近似。
3. Center-grasp success：說明中心抓取降低力矩。
4. 180-degree grasp stress test：說明摩擦抓取在較大動作下仍保持，到 release 才掉落。
5. MoveIt-to-Newton shadow execution：說明 MoveIt 的完整 Cartesian 任務已跨 bridge 在 Newton 重現。
6. ROS–Newton screenshot：說明 bridge、joint mapping 與回傳狀態。

## 5. 今天完成的 MoveIt–Newton 里程碑

一開始 MoveIt 與 Newton 對 UR5 起始姿勢的認知不同：`elbow_joint` 與 `wrist_2_joint` 各差 90 度。安全檢查器先回報 `pass=false`，避免直接接線造成跳動。同步後結果變成：

```text
pass=true
maximum start-state error=5.25e-8 rad
```

完整 MoveIt 任務再次完成三段 100% Cartesian path，Newton 同時顯示 `MOVEIT_SHADOW`：

```text
final maximum arm error=3.40e-8 rad
PICK AND PLACE DEMO SUCCEEDED
```

為了讓動作在錄影中清楚可見，另做一組控制變因測試：

```text
Approach  = (+0.03, +0.10, -0.05) m
Transport = (-0.03, -0.30, +0.10) m
Return    = ( 0.00, +0.20, -0.05) m
```

返回向量由 `-(Approach + Transport)` 自動產生，因此最後回到原起點。三段完成率仍為 100%，最終 shadow error 為 `5.86e-8 rad`。

## 6. 工程判斷與限制

今天的結果證明資料路徑、關節順序、起點同步與 Newton 運動學播放正確。它沒有證明動力學追蹤，因為 fake controller 仍是 ROS 的正式狀態來源，Newton 直接套用轉送角度。

如果教授問「你如何判斷不是只有畫面看起來成功」，回答：

> 我同時檢查 Cartesian path completion、起點與終點的向量閉合，以及 ROS reference 與 Newton returned state 的最大角度差。`SUCCEEDED` 只是其中一項證據。

如果教授問「過程中有沒有判斷錯誤」，回答：

> 有。我一度在沒有讀完整參數時假設兩個 approach 分量為零，因而誤判 return target。讀回完整 X/Y/Z 後，向量加總正確閉合。這讓我建立了先確認完整介面資料、再下結論的檢查習慣。

教授若問「是否已完成整合」，回答：

> 已完成可驗證的 MoveIt-to-Newton shadow execution。下一階段會讓 Newton 執行帶時間戳的 actuator trajectory、成為 feedback owner，並把 MoveIt 任務與柔性膠條接觸場景合併。

## 7. 下一步與研究方向

1. `FollowJointTrajectory` → Newton actuator target。
2. Newton actual state → ROS 2 authoritative feedback。
3. MoveIt 規劃＋柔性膠條抓取的同場景測試。
4. 建立速度、抓取位置、摩擦、剛性與成功／滑落標籤資料集。
5. 使用可解釋的 decision tree 選擇抓取策略，再評估更進階的控制方法。

研究題目可以表達為：

> **Physics-aware motion planning and adaptive grasp selection for deformable-object manipulation using ROS 2, MoveIt 2, and Newton.**
