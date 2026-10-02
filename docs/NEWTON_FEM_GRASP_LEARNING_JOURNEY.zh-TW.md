給 Gemini 的簡報製作要求
========================

請將以下內容製作成 16～18 頁的繁體中文工程專題簡報。
風格：大學工程專題、乾淨、專業、重視實驗證據與推理。
請保留英文專有名詞及所有數據，不要把尚未驗證的內容寫成已成功。
每頁以 3～6 個重點為主，公式與數據可做成圖表。失敗歷程請做成
「現象 → 假設 → 實驗 → 結論 → 修正」流程圖。

專題名稱
========

ROS 2 MoveIt 與 Newton 柔體物理整合：
UR5 + Robotiq 2F-85 地面橡膠條夾取、故障分析與 Jazzy 移植

GitHub：
https://github.com/technocrat94/ros2-robotics-simulation-lab

最終 Humble 成功基準：commit 3546672

本報告的心路歷程起點不是最後的 FEM 成功，而是最初的陽春地面夾取：先用
簡化 segmented／近似剛體膠條確認手臂、夾爪、ROS 命令與 Newton 接觸能運作，
再逐步換成 FEM、加入 MoveIt 絕對位置、診斷摩擦與穿模，最後才修好剛柔接觸、
夾爪高度補償和路徑繞行。簡報應按這條時間線講，讓每次修正都對應一個前一階段
尚未解決的問題。


第 1 頁｜研究動機與核心問題
============================

研究問題：MoveIt 規劃成功後，如何確認機器人在含有重力、摩擦、接觸與柔體
變形的環境中，真的能把物體夾起來？

- MoveIt fake hardware 可以驗證路徑、逆向運動學與控制流程。
- fake hardware 不會模擬橡膠變形、摩擦、滑落及穿透。
- Newton 用來計算柔體、接觸、摩擦和重力造成的實際結果。
- 最終目標不是畫面看似成功，而是建立可重建、可量測、可解釋的夾取流程。

一句話結論：
MoveIt 回答「機械手應該怎麼走」，Newton 回答「照這樣走之後，物體實際會
發生什麼」。


第 2 頁｜系統架構與資料流
========================

MoveIt 2
  → 根據物體絕對位置進行 IK（Inverse Kinematics）與路徑規劃
  → ROS 2 controllers 執行 UR5 與 Robotiq 命令
  → ROS–Newton bridge 同步、翻譯並插值離散關節狀態
  → Newton 計算重力、摩擦、碰撞與 FEM 柔體變形
  → 實際物體位置、模擬時間、接觸及誤差回傳 ROS 2

各元件角色：

- UR5 / Robotiq：被描述、控制與模擬的機械系統。
- MoveIt：IK、碰撞感知規劃與 Cartesian motion。
- ros2_control：控制器、action 與關節狀態。
- bridge：連接 ROS Python 3.10 與 Newton Python 3.12，並處理兩套時間尺度。
- Newton：柔體動力學與剛柔接觸驗證。


第 3 頁｜第一階段：建立 ROS 2–Newton Bridge
==========================================

完成項目：

- ROS service 控制 Newton start、pause、reset。
- ROS topics 回傳 object pose、simulation time、running state、bridge status。
- stale-data detection 能辨識 Newton 是否停止回傳資料。
- start-state guard 比較 ROS 與 Newton 的初始關節角。
- trajectory shadow 將 MoveIt 軌跡映射到 Newton robot。

重要驗證：

- bridge status 可顯示 OK 或 STALE。
- reset 後模擬時間回到 0，物體回到初始位置。
- MoveIt shadow 最大關節誤差曾達約 3.4×10^-8 rad，證明資料對接精確。

工程判斷：service 回覆「command sent」只代表指令已送出；仍須從回傳 topic
確認 Newton 的實際狀態。


第 4 頁｜UR5 與 Robotiq 模型匯入
================================

URDF/Xacro 展開後：

- 24 links / bodies
- 23 URDF joints；Newton 匯入後包含基座關係共 24 joints
- 54 collision / visual shapes
- 關節型態包含 fixed 與 revolute

Robotiq 2F-85 的運動學：

- 只有一個 leader joint 接收主要命令。
- 其他五個 movable joints 依 URDF mimic multiplier 跟隨。
- mimic mapping 最大誤差驗證為 0 rad。

工程判斷：模型成功顯示不代表運動正確；還必須確認 joint order、mimic 方向、
關節限制與碰撞幾何。


第 5 頁｜柔體橡膠條與 FEM 網格
==============================

橡膠條尺寸：0.40 × 0.05 × 0.02 m
網格：20 × 3 × 2 cells

- 252 particles（粒子／FEM 頂點）
  (20+1)(3+1)(2+1) = 252
  保存位置、速度與質量，是柔體的自由度。

- 600 tetrahedra（四面體元素）
  20×3×2×5 = 600
  計算拉伸、剪切與體積改變，負責傳遞內部材料力。

- 424 surface triangles（表面三角形）
  4(20×3 + 20×2 + 3×2) = 424
  定義柔體外表面及接觸邊界。

關鍵觀念：增加 cells 是提高 mesh resolution（網格解析度），不是提高 material
density（材料密度）。網格更細會增加計算量，也可能改變預測變形，因此必須做
mesh convergence（網格收斂）比較。


第 6 頁｜VBD 與橡膠材料參數
===========================

Newton 使用 VBD（Vertex Block Descent，頂點區塊下降法）求解體積柔體。

白話說明：膠條內有許多互相影響的節點，VBD 逐一更新頂點，反覆降低系統能量，
同時滿足材料變形與接觸條件。

最終參考設定：

- Young's modulus：1,000,000 Pa
- Poisson ratio：0.45
- density：1,100 kg/m³
- damping：1,000 Pa·s
- VBD iterations：10
- frame rate：60 Hz
- substeps：10
- physics dt = (1/60)/10 = 1/600 s = 0.001667 s

工程判斷：Young's modulus 控制剛硬程度；Poisson ratio 接近 0.5 代表接近不可
壓縮；damping 影響振動衰減。材料參數、網格與時間步都會共同影響結果。


第 7 頁｜兩種柔體模型與學到的事情
==================================

模型 A：Tetrahedral FEM

- 每個四面體描述連續材料變形。
- 物理意義完整，但 CPU 計算慢。
- 網格 20 cells 的 real-time factor 約 0.265；40 cells 約 0.144；80 cells 約 0.094。
- 網格變細後，預測撓度與收斂時間也改變。

模型 B：Segmented compliant-joint strip

- 由 20～80 個剛體片段與彈簧／阻尼關節組成。
- 可用 EI 與 segment length 推導 joint stiffness。
- 速度較快，適合早期整合與控制測試，但不是完整連續體 FEM。

工程結論：簡化模型用來快速定位問題，FEM 用來驗證真正柔體接觸；兩者用途
不同，不是重複做同一件事。


第 8 頁｜SDF、碰撞代理與全表面接觸
===================================

SDF（Signed Distance Field，符號距離場）用來回答：

- 粒子距離指尖表面多遠？
- 是否已經穿透？
- 接觸法向量朝哪裡？

概念式：

phi(p) > 0：表面外部
phi(p) = 0：位於表面
phi(p) < 0：已穿透

penetration = max(0, -phi(p))
normal = normalized(gradient(phi(p)))

問題：Robotiq 的三角 mesh 很複雜，當時 CPU VBD 路徑無法提供可靠的
mesh-to-particle 距離查詢。

最終修正：

- 保留原始 mesh 作為漂亮的視覺模型。
- 關閉原始 finger mesh 對 FEM particles 的碰撞。
- 在四個 finger / fingertip collider 內加入隱藏 analytic box proxy。
- proxy 向內縮 1 mm，避免突出外觀模型。
- 啟用 full-surface rigid–soft contact，使壓力與摩擦分布在接觸面。

白話說明：畫面看到真實手指，物理引擎實際使用穩定的隱形方盒計算柔體接觸。


第 9 頁｜早期夾取測試：看似成功不等於真正夾持
==================================================

早期現象：較大的閉合角度能把膠條帶起來。

可能原因：

- 摩擦夾持（friction-dependent grasp）
- 幾何卡合（geometric capture）
- 指尖托住物體
- 穿模後被模型卡住

診斷方式：matched control（配對對照實驗）

- 0.376 rad、mu=1.5：能抬起，開爪後掉落。
- 0.376 rad、mu=0：接觸期間落地，證明此工作點依賴摩擦。
- 0.386 rad、mu=0 與 mu=1.5 都能抬起，顯示主要是幾何卡合。

中心夾取的工程理由：

- 膠條質量約 0.44 kg，重量約 4.32 N。
- 夾末端時力臂約 0.20 m，重力力矩約 0.86 N·m。
- 改夾 center-of-mass region，可降低旋轉與滑出的力矩。


第 10 頁｜180 度壓力測試
=========================

在已驗證的中心夾取工作點加入更快、更大的動作：

- 肩部掃動
- 約 180 度手腕偏轉
- 加速與姿態改變

觀察結果：

- 膠條在舞動期間保持於兩指之間。
- 沒有穿過機械手臂。
- 只有夾爪張開後才掉落。
- 舞動前後夾持區高度差約 0.88 mm。

結論：Newton 能在指定 kinematic robot motion 下保持接觸與柔體反應。
限制：這不等於真實 UR5 馬達一定能提供相同扭矩與頻寬。


第 11 頁｜MoveIt 絕對位置地面夾取
==================================

原先的 MoveIt 範例使用相對位移，例如「從現在位置往 Y 移動 0.1 m」。
新流程改為：

1. Newton 在 /newton/object_pose 回傳膠條 world 座標。
2. MoveIt 讀取物體中心的絕對位置。
3. 加上 tool0 到兩指中點的 0.109 m offset。
4. MoveIt 使用 IK 將末端目標位姿轉成六個 UR5 關節角。
5. 先到 pre-grasp，再垂直下降、閉爪、抬升與開爪。
6. Planning scene 加入 floor，避免規劃穿過地面。

最後測得物體中心：
(0.4869, 0.1093, 0.0110) m in world

工程判斷：MoveIt 不會從影像猜物體位置；它依賴 Newton 或感測器提供 pose，
再進行 IK 與碰撞感知規劃。


第 12 頁｜夾爪寬度與高度補償
================================

Robotiq 指尖沿連桿圓弧閉合，因此 leader angle 增加時：

- pad gap 變小；
- fingertip midpoint 同時向下移動。

若忽略下降量，手臂到達正確高度後再閉爪，指尖仍會向下穿入地板或膠條。

50 mm 膠條、總壓縮量 2 mm：

target gap = 50 - 2 = 48 mm

校正資料：

- 0.35 rad → gap 50.716 mm，closure drop 10.109 mm
- 0.40 rad → gap 45.315 mm，closure drop 11.042 mm

線性插值結果：

- gripper command = 0.375145 rad
- closure drop = 10.578 mm
- uncompensated approach = 0.105000 m
- compensated approach = 0.105000 - 0.010578 = 0.094422 m

工程意義：物體寬度同時決定夾爪角度與閉爪前的 tool0 高度。


第 13 頁｜最大失敗：有接觸力，物體仍完全不抬升
=================================================

量測曾顯示：

- 左右 loaded contacts：25 / 22
- 左右接觸力大小總和：231.335 / 250.229
- 物體抬升量：0 m

錯誤直覺：「接觸點很多、力量很大，所以應該已經夾住。」

實際判斷：

- force magnitude 沒有告訴我們方向。
- 力可能把物體沿長度推走、壓向地板，或在抬升瞬間消失。
- 將摩擦提高到 10、增加壓縮量，仍然沒有解決。
- 所以根因不是單純「太重、摩擦不足或夾得不夠緊」。

診斷對照：

- 改成同尺寸 rigid block，剛體碰撞會產生反應。
- 換回 FEM 後，膠條不凹陷且被 mesh 指尖穿過。
- 因此將問題定位到 FEM particles 與 Robotiq mesh 沒有可靠剛柔接觸。


第 14 頁｜時間離散問題：等效速度與指令插值
================================================

ROS 送來的是離散關節位置。如果 Newton 直接從 q_k 跳到 q_(k+1)：

v_effective ≈ (q_(k+1) - q_k) / delta_t

單步跳動過大時：

- 指尖可能在兩幀間跨過膠條表面。
- 產生漏碰撞、深穿透或巨大的非物理接觸脈衝。
- 再好的 SDF 與碰撞代理也可能失效。

修正：

- sync_robot_state 先同步 ROS 與 Newton 初始姿態。
- 使用 bounded command queue。
- 每筆 ROS 命令以 NEWTON_COMMAND_DT = 0.02 s 插值。
- 每個 1/60 s frame 再分成 10 個 physics substeps。

工程結論：正確的幾何模型必須搭配正確的時間輸入。


第 15 頁｜MoveIt 路徑修正與最終成功
====================================

曾發生的路徑問題：

- 直接前往 pre-grasp 可能繞行、穿地或讓 wrist 大幅旋轉。
- named start 的 test_configuration 造成不必要的大繞路。
- 重複啟動兩套 bringup 會產生 duplicate action servers、unknown goal response
  或執行被中止。

最終修正：

- 四階段安全 Cartesian route：先抬高、調整姿態、水平移到物體上方、垂直下降。
- 檢查每個關節 cumulative travel 與 maximum single step。
- 抬升獨立使用 velocity scale 0.030。
- use_named_start=false，取消 test_configuration 大繞路。
- 每次只允許一套 MoveIt / controller bringup。

最終 Humble 成功證據（commit 3546672）：

- Safe pre-grasp path：100%
- Vertical approach path：100%
- Vertical lift path：100%
- Lift distance：0.120 m
- Lift trajectory duration：3.517 s
- MoveIt 記錄段：約 12.765 s
- 終端：ABSOLUTE POSITION PICK SUCCEEDED
- 人工物理驗收：沒有大繞路、膠條成功抬起、開爪後才掉落。

重要區分：MoveIt SUCCEEDED 證明規劃與控制器完成；Newton 畫面中的抬升與
釋放才證明物理夾取成功。本次兩層皆通過。


第 16 頁｜學校電腦 Jazzy 移植
================================

參考平台：

- 家中 UTM：Ubuntu 22.04、ROS 2 Humble、aarch64、CPU
- 學校電腦：Ubuntu 24.04、ROS 2 Jazzy、x86_64 / amd64、NVIDIA GPU

不能直接複製的內容：

- build/、install/、log/
- ARM binaries
- Python virtual environment

必須移植並重新建構的內容：

- source code、URDF/Xacro、SRDF
- MoveIt / ros2_control configuration
- calibration data、launch files、bridge protocol
- Newton Python 3.12 amd64 environment

已完成的 Jazzy 工作：

- 原生 Jazzy UR5 + Robotiq MoveIt fake-hardware 建置成功。
- 四個 controllers active，RViz 中 pick-and-place 三段路徑皆 100%。
- 建立 Newton 1.5.1 / Warp 1.17.0 / Python 3.12 amd64 環境。
- 解析 Jazzy Xacro 並產生 Newton URDF，所有 package:// mesh paths 已解析。
- Jazzy controller topic 改為 /joint_trajectory_controller/controller_state。
- controller state 欄位由 Humble 的 desired.positions 適配為 Jazzy 的
  reference.positions。
- bridge、start-state guard、trajectory shadow 與 Newton viewer 可運作。

Jazzy 量測證據：

- start-state guard maximum error：5.62×10^-8 rad
- trajectory shadow error：5.66×10^-8 rad
- mimic maximum error：0 rad
- object pose：(0.48689985, 0.10915001, 0.00999984) m


第 17 頁｜Jazzy 目前邊界與正確的後續策略
=========================================

當時學校 Jazzy 整合結果：

- MoveIt 流程完成並顯示 ABSOLUTE POSITION PICK SUCCEEDED。
- closed_seen、lift_started、release_seen 均為 true。
- grasp-region lift 只有 0.000630 m（約 0.63 mm）。
- candidate_contact_grasp_pass = false。

因此：

- Jazzy 已成功移植「建置、規劃、控制、bridge、同步、接觸與顯示」。
- 當時尚未重現真正的動態 FEM 地面夾取。
- 學校版本建立在較早的 Humble 參考上；最新成功修正現已固化於 commit 3546672。

下一次 Jazzy 工作的正確做法：

1. 從 GitHub commit 3546672 更新 source。
2. 保持最終 Humble 的 FEM、proxy、full-surface contact、插值、夾爪校正與
   use_named_start=false。
3. 只修改 Jazzy API／套件相容部分，不同時調物理參數。
4. 重新驗證無繞路、抬升、保持、穿透限制與開爪釋放。
5. 通過相同驗收後，Jazzy 才能取代 Humble 成為主要開發平台。


第 18 頁｜兩週內培養的工程判斷能力與下一步
================================================

培養的能力：

1. 分開「規劃成功」與「物理成功」。
2. 分開「有接觸」與「有承載能力」。
3. 一次只改一個變因，才能建立因果關係。
4. 先檢查幾何與時間，再調摩擦和材料參數。
5. 使用 rigid block、zero-friction 等 control experiment 定位問題層級。
6. 選擇直接回答研究問題的 metric，例如 grasp-region rise，而不是整條柔體平均高度。
7. 保留失敗 log、參數與推理，使結果可以重建與移植。

目前限制：

- friction=10 是模擬成功參數，尚未以真實橡膠材料校正。
- analytic box proxy 是可靠近似，不是 Robotiq 真實指尖幾何的完全等價物。
- Newton 中的 robot 由關節位置驅動，尚未證明真實馬達扭矩與控制頻寬足夠。
- Humble 物理成功仍包含人工視覺驗收，尚未完成全自動 pass/fail 判斷。

下一步：

- 自動記錄 object rise、bilateral loaded contact、penetration、retention、release。
- 記錄單次實驗的精確 real-time factor；目前 UTM 長時間估算約 0.084，
  即 1 秒模擬時間約需 11.9 秒實際時間。
- 將最新成功 commit 移植到 Jazzy 並做相同驗收。
- 在 Jazzy CPU 基準一致後，再啟用 NVIDIA GPU 與更高解析度 FEM。
- 最後加入真實相機 pose、材料校正與實體 UR5 安全驗證。


可放在結尾的口頭總結
====================

「這兩週我不是只把一個動畫做出來，而是建立了一條從感測位置、MoveIt IK 與
路徑規劃、ROS 控制、bridge 時間同步，到 Newton FEM 接觸驗證的完整工程鏈。
最重要的成果是：我曾經看到接觸點和很大的力，卻仍然夾不起膠條；透過摩擦
對照、剛體方塊對照與 FEM 變形觀察，我把問題定位到 mesh-to-particle 接觸與
離散命令的等效速度。最後使用解析碰撞代理、VBD 全表面接觸、命令插值、夾爪
高度補償與安全路徑，才得到無繞路、能抬升並在開爪後釋放的成功版本。學校的
Jazzy 平台已完成軟體與 bridge 移植，下一步是用最新基準重現同一個物理結果。」


建議插圖／影片素材
==================

1. 系統架構流程圖：MoveIt → controllers → bridge → Newton → feedback。
2. FEM 網格示意：particles、tetrahedra、surface triangles。
3. 解析 box proxy 與 Robotiq 外觀 mesh 疊圖。
4. 失敗案例：有接觸力但物體沒有升高。
5. 成功案例：無繞路地面夾取、抬升、開爪釋放。
6. Humble aarch64 UTM 與 Jazzy amd64 學校電腦比較表。
7. GitHub 成功紀錄：
   https://github.com/technocrat94/ros2-robotics-simulation-lab/blob/main/docs/HUMBLE_FEM_PICK_SUCCESS.md


教授可能提問與建議回答
======================

Q1：MoveIt 已經顯示成功，為什麼還需要 Newton？
A：MoveIt 的成功只證明軌跡與控制器完成。它的 fake hardware 不模擬柔體變形、
   摩擦與滑落；Newton 用來驗證物體是否真的被夾起及何時釋放。

Q2：為什麼把摩擦力調很大仍然夾不起來？
A：摩擦必須建立在正確的法向接觸上。當 FEM particles 與 mesh 指尖沒有可靠
   距離／法向資料時，增加摩擦不會修好接觸管線。

Q3：為什麼要使用隱藏 box，而不直接使用真實 mesh？
A：當時 CPU VBD 路徑需要穩定的 particle distance query。analytic box 能提供
   連續且便宜的距離與法向；原始 mesh 繼續保留作為視覺幾何。

Q4：如何證明不是把物體偷偷 attachment 在夾爪上？
A：系統沒有 attachment constraint；零摩擦對照會失敗，成功案例在閉爪時保持，
   並且只有開爪後才掉落，因此行為來自接觸、摩擦與幾何。

Q5：Jazzy 移植成功了嗎？
A：軟體建置、MoveIt、controller、bridge、同步與 Newton 整合已成功；較早測試的
   動態抬升只有 0.63 mm，所以物理夾取尚未通過。現在已有 commit 3546672 的
   Humble 成功基準，下一步是以相同參數重新驗收 Jazzy。

Q6：目前最大的研究限制是什麼？
A：摩擦與材料參數尚未用真實橡膠校正，指尖使用解析代理，robot 仍是位置驅動，
   且物理成功仍含人工畫面判斷。下一步是自動量測及實體校正。
