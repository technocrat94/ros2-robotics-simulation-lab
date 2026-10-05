# MoveIt 到 Newton FEM 地面夾取：完整工程敘事

狀態：Humble 成功基準為 Git commit `3546672`；本文件記錄從 MoveIt 整合開始的
判斷、誤判、對照實驗、修正原理與最終驗收。

## 閱讀方式

這份文件分成兩部分：

- **第 1～20 章是工程故事**：依照實際發生順序，說明我們看到什麼、為什麼做出
  某個判斷、實驗結果推翻或支持了什麼，以及下一步為什麼那樣修。
- **第 21 章以後是操作與觀念課本**：假設讀者第一次接觸 ROS 2、MoveIt 與
  Newton，逐一解釋終端機、資料流、座標、路徑規劃、FEM、碰撞、時間同步、
  參數調整和驗收方法。

讀這份紀錄時，不要只背「最後用了哪些參數」。真正值得學會的是：**每次只問一個
可被量測回答的問題，先定位失敗層，再修改那一層。**

## 1. 為什麼 MoveIt 之後，我們一開始沒有懷疑膠條模型

在整合 MoveIt 之前，我們已經在 Newton 做過一系列「直接指定機器人動作」的
夾取測試：

- 夾爪能在膠條中心形成接觸並抬升。
- 膠條能在較快運動與約 180° 手腕旋轉時留在兩指之間。
- 沒有 attachment constraint；開爪後膠條才掉落。
- 零摩擦對照曾讓其中一個工作點失敗，表示至少部分成功來自摩擦，不只是把
  物體偷偷黏在手上。

所以 MoveIt 地面夾取第一次失敗時，我們沒有立刻懷疑「膠條根本不能被模擬」。
當時最合理的想法是：以前夾得起來，現在新加入的是 MoveIt、絕對座標、地板、
IK、路徑與新的起始姿態，因此應該先檢查這些新增部分。

這個判斷並不愚蠢，但少了一個重要區分：早期成功的膠條主要是
**segmented compliant-joint model**，由多個剛體片段與彈簧／阻尼關節組成；後來
地面夾取使用的是 **tetrahedral FEM volumetric body**。兩者外觀看起來都像膠條，
但碰撞與求解管線不同。

早期成功真正證明的是：

- UR5／Robotiq 動作與 mimic joints 可以工作；
- 剛體接觸與摩擦能形成某些有效夾持；
- 中心夾取比邊緣夾取合理；
- Newton 能在指定動作下讓物體保持與釋放。

它沒有證明：Robotiq 原始三角 mesh 能對 VBD 的 FEM particles 提供可靠的距離、
法向與全表面接觸。這個差異是後來所有推理的轉折點。

## 2. 第一步：先確定 MoveIt 控制的是哪六個關節

我們先檢查 `joint_trajectory_controller` 的 joints：

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

原因：MoveIt 的 trajectory point 只提供一串 positions，數值必須依 joint_names
順序解讀。如果順序或名稱錯了，即使數字本身合理，實際動作也會完全不同。

我們也讀取 controller state，學會區分：

- desired/reference：控制器想去的位置；
- actual/feedback：目前量到的位置；
- error：兩者差值。

當 error 接近 0，只能說控制器已追到目標；不能推論物體已被夾起。

這一步排除了「MoveIt 把關節值送錯順序」這個根因。

## 3. 第二步：ROS 與 Newton 起始姿態不一致

最早的 `start_state_guard` 顯示 elbow 與 wrist_2 約有 `π/2` 的誤差。原因是 ROS
fake hardware 的當前姿態與 Newton endpoint 內部初始化的 `BASE_ARM` 不同。

若兩套系統從不同姿態開始，後續即使收到相同 trajectory：

- Newton 可能先瞬間跳到另一個姿態；
- 指尖產生極大的等效速度；
- viewer 中看到的動作與 RViz 不同；
- 接觸與穿透結果不再可信。

修正原理：

1. `/newton/sync_robot_state` 把 ROS 現在的關節狀態送給 Newton。
2. `start_state_guard` 比較兩邊每個關節。
3. guard 通過後才開啟 trajectory shadow。

這像在比賽前讓兩支碼表都歸零。若起點不同，後面誤差再小也沒有意義。

## 4. 第三步：先用相對運動證明 bridge，而不是直接挑戰地面夾取

我們先使用既有的 `move_xyz`，並故意把位移改大，因為小動作在 RViz 與 Newton
中很難看清楚。三段 Cartesian path 都達到 `100%`，trajectory shadow error
約 `5.86×10^-8 rad`。

這個實驗證明：

- MoveIt 能產生軌跡；
- controller 能執行；
- bridge 能把參考關節狀態送到 Newton；
- Newton robot 能近乎一致地重播 MoveIt 動作。

它仍沒有證明接觸物理正確。這是第一個重要分層：先驗證 transport，再驗證
contact。

## 5. 第四步：從相對位移改成物體絕對位置

早期程式只知道「從目前位置往某方向移動多少」。真正的抓取應該由物體位置
決定，因此我們讓 Newton 發布：

```text
/newton/object_pose
frame_id: world
```

MoveIt node 讀取物體中心，再加上 `tool0` 到兩指中點的 offset，建立 top-down
grasp pose，最後由 IK 把末端位姿轉成六個 UR5 關節角。

這裡的原理是：

```text
物體 world pose
  + 抓取方向
  + tool0-to-fingertip offset
  = tool0 目標 pose
  → IK
  = UR5 joint trajectory
```

因此不是我們手算六個軸，也不是 MoveIt 自己「看到」膠條；物體 pose 必須由
Newton 或未來的相機提供。

## 6. 第五步：路徑會穿地、繞路，因此先做 plan-only

一開始的全域規劃有時會繞一大圈、手腕大幅旋轉，甚至看起來穿過地板。原因是
只給末端終點時，規劃器可以找到許多 IK 解與關節路徑；數學上可達，不代表工程
上合理。

我們加入：

- planning scene floor：讓 MoveIt 知道地板是障礙物；
- plan-only：只預覽，不執行；
- pre-grasp-only、approach-only、grasp-only、lift-only：逐段隔離問題；
- 四階段路徑：先抬高、調姿態、水平移到物體上方、最後垂直下降；
- joint cumulative travel 與 maximum step 檢查；
- wrist_3 獨立上限，避免手腕為等價姿態轉太多圈。

後來又發現 named start `test_configuration` 本身會造成大繞路。最終設定
`use_named_start=false`，直接從同步後的實際起始姿態規劃。

## 7. 第六步：指尖看似碰到，為什麼仍夾不起來

在 RViz 中，指尖接近膠條時有時會顯示紅色；那代表 MoveIt collision model 的
碰撞／接近狀態，不等於 Newton 已建立可承載的柔體接觸。

我們手動測試不同 gripper command：

- 有時指尖仍有空隙；
- 有時剛好接觸；
- 有時看起來穿進膠條；
- 夾爪抬升後，膠條仍留在地面。

這讓我們先懷疑三件事：

1. 夾爪閉得不夠；
2. 摩擦力不夠；
3. approach 高度不對。

這些都是合理假設，因為早期模型確實曾因夾持位置、摩擦與中心／邊緣差異而
改變結果。

## 8. 第七步：發現夾爪閉合時還會向下

Robotiq 不是兩片平行板水平靠近。leader joint 帶動連桿旋轉，兩指閉合時，
fingertip midpoint 也會向下移動。

校正量測：

| command | estimated gap | closure drop |
|---:|---:|---:|
| 0.35 rad | 50.716 mm | 10.109 mm |
| 0.40 rad | 45.315 mm | 11.042 mm |

50 mm 物體、總壓縮 2 mm，目標 gap 為 48 mm。線性插值得到：

```text
leader command = 0.375145 rad
closure drop = 10.578 mm
```

若原本想下降 `0.105 m`：

```text
compensated approach
= 0.105000 - 0.010578
= 0.094422 m
```

修正原理：手臂先停得稍高，留出閉爪連桿向下的空間。這避免「位置規劃本來
正確，閉爪後卻把指尖推進地板」。

## 9. 第八步：有左右接觸和很大的力，仍然抬升 0 m

某次閉爪量測：

```text
left/right loaded contacts = 25 / 22
left/right force magnitude sum = 231.335 / 250.229
measured lift = 0 m
```

一開始很容易下結論：「既然兩邊都有力，應該只是摩擦不夠。」所以我們曾把
摩擦提高到診斷值 10，也增加壓縮量，但仍無法保持抬升。

這個結果推翻了單純摩擦假設。原因是 force magnitude 沒有方向：

- 力可能沿膠條長度把它推出去；
- 可能把膠條壓向地板；
- 可能閉爪時存在，但第一幀抬升就消失；
- 接觸候選數也可能包含不能承載的接觸。

因此「有碰撞」與「能承載」必須分開。後續正確量測應看左右手指世界座標的
`Fx/Fy/Fz`、接觸是否持續，以及 grasp region 是否真的上升。

## 10. 第九步：為什麼後來才重新懷疑膠條／接觸模型

到這裡，位置、IK、路徑、夾爪角度、閉合高度、摩擦與壓縮都已逐步檢查，失敗
仍然存在。這時才有足夠證據回頭問：「早期成功的膠條，真的和現在的 FEM 是
同一種物理模型嗎？」

答案是否定的：

- segmented strip：多個剛體片段＋compliant joints，主要走剛體碰撞管線；
- FEM strip：particles＋tetrahedra＋surface triangles，由 VBD 求解體積變形。

兩者外觀與尺寸可以相同，接觸演算法卻不同。早期成功讓我們合理地相信抓取策略
本身可行，但不能替 FEM particle contact 背書。

## 11. 第十步：rigid block 對照把問題定位到 FEM contact

我們把物體暫時換成同尺寸 rigid block。目的不是把問題偷偷簡化成最終答案，
而是做 diagnostic control：

- 如果 rigid block 也完全無反應，應繼續查位置、IK、robot collider 或 bridge；
- 如果 rigid block 有反應、FEM 卻不凹陷，問題集中在 rigid-soft contact。

實驗中 rigid block 能產生接觸反應，但 FEM 膠條會被指尖穿過或完全不變形。
因此問題不再是「物體太重」或「摩擦係數太小」，而是原始 Robotiq mesh 沒有
為 VBD particles 提供可靠的距離與法向。

## 12. VBD、SDF 與 analytic proxy：修正原理

### VBD 是什麼

正確名稱是 VBD（Vertex Block Descent，頂點區塊下降法），不是 VBM。

FEM 膠條由 252 particles、600 tetrahedra、424 surface triangles 組成。VBD 將
一個時間步寫成材料能量與接觸約束問題，反覆更新局部頂點，使系統能量下降、
材料與接觸條件趨於一致。

它不是說物理中沒有 `F=ma`；它表示數值上不是只做一次顯式力積分，而是用隱式
能量／約束方法求解耦合變形與接觸。

### SDF 提供什麼

SDF `phi(p)` 告訴求解器某粒子到剛體表面的帶符號距離：

```text
phi > 0：表面外
phi = 0：表面上
phi < 0：已穿透
penetration = max(0, -phi)
normal ≈ normalized(gradient(phi))
```

距離決定穿透量，gradient 決定應該往哪個方向推回去。

### 為什麼用 analytic box proxy

原始 Robotiq 三角 mesh 很漂亮，但當時 CPU VBD 路徑沒有穩定的
particle-to-mesh SDF。修正方式：

- 視覺仍保留原始 mesh；
- 關閉原始 mesh 對 FEM particles 的不可靠碰撞；
- 在四個 finger／fingertip 區域放入隱藏 analytic boxes；
- box 向內縮 1 mm，避免突出視覺表面；
- 啟用 full-surface rigid-soft contact。

Box 有封閉解析距離，可以快速、連續地提供 SDF 與 normal。這不是宣稱真實手指
是方盒，而是選擇可控、可驗證的 collision proxy。

## 13. 第十一步：ROS 命令插值，避免指尖「瞬移」穿透

即使碰撞幾何正確，ROS 關節狀態若一筆一筆直接跳到 Newton：

```text
v_effective ≈ (q[k+1] - q[k]) / dt
```

單步位移過大時，指尖可能在兩次碰撞查詢之間越過膠條表面。Newton 看到的是
很大的等效速度與深穿透，而不是平滑運動。

修正：

- bounded command queue：命令依序處理，不讓舊命令無限堆積；
- `NEWTON_COMMAND_DT=0.02 s`：在兩筆 ROS 命令間插值；
- 60 Hz frame 再分 10 substeps，physics `dt=1/600 s`；
- 每個 substep 更新中間關節姿態後再求碰撞與 VBD。

原理是讓幾何接觸有足夠時間解析度。正確模型配上瞬移命令仍然可能穿模。

## 14. 第十二步：物理成功後，仍要修掉 MoveIt 大繞路

當 FEM 終於能跟著夾爪上升後，流程還有一個工程問題：手臂會先前往 named
target `test_configuration`，造成大繞路。

這與物理夾持是兩個獨立問題。為了確認取消繞路不會破壞夾取，我們做單一變因
測試：保留 FEM、proxy、摩擦、校正、下降與抬升參數，只改：

```text
use_named_start=true → false
```

結果：沒有大繞路，膠條仍成功抬起，開爪後才掉落。因此可以判斷 named start
不是成功所需條件，應移除。

## 15. 最終 Humble 驗收

```text
object center                 (0.4869, 0.1093, 0.0110) m
leader command                0.375145 rad
closure drop                 10.578 mm
compensated approach          0.094422 m
lift distance                 0.120 m
lift velocity scale           0.030
pre-grasp / approach / lift   100% / 100% / 100%
lift trajectory               3.517 s
MoveIt logged sequence        約 12.765 s
final message                 ABSOLUTE POSITION PICK SUCCEEDED
```

人工物理驗收：

- 手臂不再先繞一大圈；
- 閉爪後 FEM 膠條跟著上升；
- 抬升時膠條留在兩指之間；
- 開爪後才掉落；
- 沒有明顯穿地、穿手或 NaN。

所以成功不是來自某個神奇參數，而是一整條條件同時成立：正確 pose、同步起點、
合理路徑、夾爪運動學補償、可靠 collision proxy、VBD full-surface contact、
平滑命令時間史，以及同時檢查 command layer 和 physics layer。

## 16. 這段歷程教會我們如何當工程師

1. 以前成功過，是合理的先驗證據，但必須確認「模型與管線是否真的相同」。
2. 畫面共同移動、MoveIt SUCCEEDED、contact count、force magnitude 都不是單獨
   足夠的夾取證據。
3. 調參數前先定位問題層級：planning、transport、rigid collision、soft contact、
   material solve 或 validation metric。
4. 每次只改一個變因，才能說明因果。
5. control experiment 的價值是排除假設：zero friction、rigid block、plan-only、
   isolated lift 都不是多餘測試。
6. 修正必須能說明原理：proxy 修的是幾何距離、interpolation 修的是時間離散、
   closure compensation 修的是夾爪連桿運動學、staged path 修的是規劃自由度。

## 17. Jazzy 移植：第一次軟體成功，物理仍失敗

學校電腦是 Ubuntu 24.04、ROS 2 Jazzy、x86_64/amd64、RTX 3080。不能直接複製
Humble aarch64 的 `build/`、`install/` 或 Python venv，所以必須從 source 重建。

第一次 Jazzy 整合已完成 build、MoveIt、controllers、bridge、同步與 viewer，
MoveIt 也顯示成功，但 grasp-region lift 只有約 `0.000630 m`，
`candidate_contact_grasp_pass=false`。

這再次證明「移植成功」至少有兩層：

- software portability：能編譯、啟動、規劃與傳輸命令；
- physics equivalence：相同模型和參數必須重現抬升、保持、穿透界線與釋放。

## 18. Jazzy 架構稽核：不是再調摩擦，而是比對成功條件

因為 Humble 已有成功基準，Jazzy 失敗時最有效的方法不是盲目調參，而是逐項
比較兩邊架構。稽核發現最初 Jazzy runtime 尚未完整包含：

- 自由的 `20×3×2` tetrahedral FEM strip；
- `SolverVBD` 與 10 iterations；
- full-surface rigid-soft contact；
- 四個 hidden analytic finger proxies；
- 原始 finger mesh 關閉 particle collision、proxy 開啟 particle collision；
- bounded queue、`0.02 s` command interpolation、10 substeps；
- Humble 已驗證的夾爪校正、下降補償與慢速抬升。

同時必須保留 Jazzy 自己的 API 差異：

- controller topic 使用 `/joint_trajectory_controller/controller_state`；
- state 使用 `reference.positions`；
- MoveIt plan fields 與 Humble 不同；
- Cartesian time parameterization 使用 Jazzy 的
  `TimeOptimalTrajectoryGeneration`；
- URDF 必須從 Jazzy Xacro 重新產生，34 個 mesh paths 全部重新驗證。

原理：移植不是把能跑的檔案搬過去，而是保持「物理與驗收契約」不變，只改平台
相容層。

## 19. Jazzy 最終成功與量化證據

2026-10-02 的原生 Jazzy 版本使用 RTX 3080 `cuda:0`，最終同時通過軟體層與
物理層：

```text
safe pre-grasp 各階段          100%
approach / lift paths          100% / 100%
named-start detour             false
grasp-region lift              0.117290587 m
release drop                   0.112259318 m
閉爪雙側接觸樣本               34
抬升雙側接觸樣本               229
maximum floor penetration      0.000820466 m
allowed penetration limit      0.005 m
finite state                   true
candidate_contact_grasp_pass   true
real-time factor               0.102285
```

viewer 目視確認膠條在兩指之間上升，並在開爪後釋放。這次成功不是因為 Jazzy
「可以編譯」，而是因為它完整重現 Humble 成功所需的物理表示、接觸、時間輸入、
路徑與驗收條件。

## 20. 下一步

後續自動化與研究應記錄：

- 左右指尖世界座標 `Fx/Fy/Fz`；
- grasp-region rise；
- contact retention；
- minimum penetration／strip bottom；
- release time；
- 單次 real-time factor；
- CPU 與 GPU 結果差異與收斂；
- 真實橡膠材料與摩擦校正；
- 實體 UR5 的扭矩、速度、網路與安全限制。

Jazzy 已完成本次模擬移植驗收；下一個研究邊界是把模擬參數和成功判斷逐步連到
真實材料與實體機器人。

---

# 第二部分：從零開始理解與重現

## 21. 先建立整個系統的心智模型

這個專題不是一個程式，而是幾個各自負責不同工作的系統合作：

```text
物體位置回饋
Newton ────────────────────────────────┐
  ▲                                    ▼
  │ 實際關節狀態、接觸、FEM       MoveIt 規劃目標姿態與路徑
  │                                    │
  │                                    ▼
  └──── bridge ← ROS 2 controller ← 關節軌跡
```

可以把它想成一間工廠：

- **UR5 + Robotiq 模型**是要被控制的機器本體描述。UR5 是六軸手臂，Robotiq 是
  夾爪。它們既不是 MoveIt，也不是 Newton。
- **MoveIt**像製程規劃員。你告訴它「夾爪要到這個位置與方向」，它使用逆向運動學
  與碰撞資訊，找出六個關節該如何移動。
- **ROS 2 controller**像馬達命令執行員。它接收 MoveIt 產生的關節軌跡，依時間
  發出各關節目標。
- **bridge**像翻譯員。ROS 2 的 Python 是 3.10，Newton 的虛擬環境是 Python
  3.12；它們不能可靠地在同一個 Python process 內直接混用，所以我們以 UDP 和
  ROS topics/services 在兩個 process 間交換資料。
- **Newton**像物理世界。它根據重力、材料、碰撞、摩擦與時間步，計算機器人和
  膠條下一刻的實際狀態。
- **Viser**只是 Newton 狀態的網頁視窗。它沒有負責路徑規劃，也不代表 ROS
  本身在瀏覽器內執行。
- **RViz**是 ROS／MoveIt 的視覺化工具。RViz 顯示規劃模型與路徑；Viser 顯示
  Newton 的物理狀態。兩者若不同步，畫面可能出現兩個姿態看起來不一致。

因此，「MoveIt 成功」與「物理夾取成功」是兩個不同命題：

1. MoveIt 成功：找得到安全路徑，而且 controller 完成軌跡。
2. Newton 成功：指尖真的對 FEM 膠條形成雙側承載接觸，膠條隨夾爪上升，開爪後
   才落下，而且沒有不合理穿透或數值發散。

## 22. 為什麼需要多個終端機，以及每個終端機在做什麼

每個長時間執行的 process 都會占住一個終端機。這不是重複啟動同一個東西，
而是讓各個角色同時存在。原生 Jazzy 成功版本的典型配置如下。

### 終端機 1：Newton endpoint

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_ground_grasp_jazzy.sh
```

它負責：

- 載入 UR5、Robotiq 與四面體 FEM 膠條；
- 建立地板、重力、材料、碰撞代理與接觸參數；
- 接收 ROS bridge 傳來的機器人關節命令；
- 執行 Newton 時間步；
- 回傳物體位置、關節狀態、模擬時間與接觸量測；
- 啟動 Viser。成功時應看到 `MOVEIT_GRASP_ENDPOINT_READY` 和 viewer URL。

它停在最後一行並不等於當機。這是 server 正在等待 ROS 命令。只有 traceback、
process 結束或長時間沒有 ready 訊息，才是啟動失敗。

### 終端機 2：ROS、MoveIt 與 controllers

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 launch ur5_moveit_demo ur5_robotiq_bringup.launch.py
```

它負責載入 robot description、MoveIt `move_group`、TF、joint state broadcaster、
UR5 trajectory controller 與 Robotiq gripper controller。

`source` 的意思是把環境設定讀進目前這個 shell。若沒有 source 正確的 ROS 版本
與 workspace，`ros2` 可能找不到 package，或錯誤地載入另一個 build。

### 終端機 3：ROS–Newton bridge

```bash
cd ~/ur5_ws
source src/ur5_moveit_demo/scripts/lab_session_env.sh
source /opt/ros/jazzy/setup.bash
source install_jazzy_port/setup.bash
ros2 run newton_ros_bridge ros_adapter
```

bridge 在 ROS 端發布 `/newton/object_pose`、`/newton/bridge_status` 等 topic，並把
service 或軌跡命令翻譯成 endpoint 能理解的 UDP 封包。看到
`ROS_ADAPTER_READY` 之後它會保持執行。

### 終端機 4：一次性檢查與夾取

預覽：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_preview.sh
```

正式執行：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_pick.sh
```

預覽的英文是 **plan-only preview**。它會規劃並顯示路徑，但不送馬達命令。
正式執行才會移動模型、閉爪、抬升與開爪。

### 為什麼不能重複啟動 endpoint 或 bridge

endpoint 和 bridge 各自綁定 UDP port。若舊 process 還在，又開第二份，會看到：

```text
OSError: [Errno 98] Address already in use
```

這不是摩擦、IK 或 FEM 錯誤，而是同一個網路埠已被占用。工程上應先用
`pgrep -af` 確認 process，再決定保留舊的或結束它，不能看到錯誤就重複開更多
終端機。

## 23. ROS 2 名詞：每一種通訊工具解決什麼問題

### Node（節點）

一個有明確職責的 ROS process。例如 `move_group` 做路徑規劃，`ros_adapter`
翻譯 Newton 資料。Node 是工作者，不是資料本身。

### Topic（主題）

持續發布的資料流，適合感測器或狀態。例如：

- `/joint_states`：ROS 看到的關節狀態；
- `/newton/object_pose`：Newton 量到的膠條位置；
- `/newton/bridge_status`：bridge 是否持續收到 Newton 狀態。

`ros2 topic echo ... --once` 只讀一筆資料。它不會命令系統做動作。

### Service（服務）

一次請求、一次回覆，適合 reset、enable、sync。例如：

- `/newton/sync_robot_state`：要求 Newton 與 ROS 起始關節同步；
- `/newton/set_trajectory_shadow`：啟用或停用軌跡轉送。

service 回覆 `success=True` 通常只代表請求已被接受或封包已送出。仍要從狀態 topic
驗證最終效果。

### Action（動作）

需要時間完成、可回報進度及結果的命令。例如 UR5 的
`FollowJointTrajectory` 與夾爪的 `GripperCommand`。Action 回報 `SUCCEEDED` 表示
controller 已完成命令，不表示物體一定被夾住。

這是本專題很重要的判斷：

```text
GripperCommand SUCCEEDED
≠ 指尖產生有效承載力
≠ 膠條被抬升
```

### Parameter（參數）

啟動時設定 node 行為的數值或開關，例如物體寬度、壓縮量、抬升距離與
`plan_only`。參數改變了「這次要怎麼跑」，程式碼定義了「系統有什麼能力」。

### Launch file（啟動檔）

一次啟動多個 nodes 並傳入一致參數的配置。它像樂團總譜，並不是執行物理公式的
地方。

## 24. 關節、座標與 IK：MoveIt 怎麼知道要去哪裡

### 六個 UR5 關節與順序

UR5 controller 使用：

```text
shoulder_pan_joint
shoulder_lift_joint
elbow_joint
wrist_1_joint
wrist_2_joint
wrist_3_joint
```

軌跡中的六個位置數值必須和這個名稱順序完全一致。若數值正確但排序錯誤，機器人
仍會移動，只是動錯軸；這種錯誤比完全不動更危險。

Robotiq 有一個主動關節（leader joint），其他五個關節依 URDF 的 `mimic` 規則
跟隨。`multiplier=-1` 表示跟主關節反方向轉。外觀看到六個活動關節，不代表需要
六顆獨立馬達命令。

### Pose（位姿）不是只有位置

位姿包含：

- position：`x, y, z`，表示位置；
- orientation：表示方向，常用 quaternion（四元數）或 RPY（roll、pitch、yaw）。

夾爪中心到達物體上方但方向錯誤，仍可能從膠條長邊夾、手腕翻轉或碰撞地板。
所以目標必須同時指定位置與方向。

### Frame（座標框架）

- `world`：本專題的全域參考座標；
- `tool0`：UR5 法蘭／工具參考框架；
- finger tip links：左右指尖各自的 link frame。

同樣的數字 `(0.1, 0, 0)`，若放在 `world` 是世界 X 方向 10 cm；若放在 `tool0`
則是工具自身 X 方向 10 cm。未說明 frame 的位置沒有完整意義。

### TF（Transform）

TF 記錄座標框架彼此的平移與旋轉關係。`tf2_echo world tool0` 回答的是：「tool0
在 world 裡的位置與方向是什麼？」最初出現 `frame does not exist`，稍後又有結果，
可能只是 TF publisher 尚未完成啟動；如果一直沒有結果，才需要查 node 或 frame
名稱。

### FK 與 IK

- **Forward Kinematics，正向運動學（FK）**：已知六個關節角，算末端位姿。
- **Inverse Kinematics，逆向運動學（IK）**：已知末端想要的位姿，求一組可達的
  六關節角。

早期 `move_xyz` 使用「從現在往某方向移動多少」的相對目標，MoveIt 仍然需要 IK。
後來 `/newton/object_pose` 提供物體的 world 絕對位置，程式建立物體上方的絕對
夾取位姿，MoveIt 同樣使用 IK。兩者差別是目標的表達方式，不是有沒有用 IK。

### 為什麼物體位置回饋很重要

若在程式裡寫死物體位置，膠條移動後機器人仍會去舊座標。現在的流程會先等待
`/newton/object_pose`，確認 frame 是 `world`，再從實測物體中心建立 pre-grasp、
grasp 與 lift 目標。這形成最基本的閉環：**先量測，再規劃。**

目前它還不是視覺伺服。系統在開始規劃前讀一次位置，不是在整段運動中持續追蹤
移動物體。

## 25. MoveIt 如何把一個夾取拆成可驗證的路徑

### Planning scene（規劃場景）

MoveIt 只會避開它知道的障礙物。Newton 畫面有地板，不表示 MoveIt 自動知道地板
存在。因此我們在 planning scene 加入 floor collision object。這才讓規劃器把
穿地路徑判為碰撞。

這也解釋了早期現象：「Newton 有地板，但 MoveIt 規劃仍穿地。」兩個系統各自有
世界模型，必須明確同步必要的碰撞幾何。

### Pre-grasp（預夾取）

先到物體正上方的安全位置，不直接從任意起始姿態斜切到地面。它的目的有三個：

1. 避免手臂掃過地板或物體；
2. 把最後接近限制為容易檢查的垂直直線；
3. 若失敗，可以區分「大範圍移動失敗」與「最後下降失敗」。

### Safe pre-grasp path（安全預夾取路徑）

規劃器有時會找到數學上可行但視覺上繞一大圈的解，常見原因是：

- 關節是週期角度，同一末端姿態可能有多個 IK 解；
- named start 與目前姿態差很大；
- 規劃器只求可行，不一定求最符合人類直覺；
- wrist joint 可能走等價但很長的角度。

我們的修正不是讓 MoveIt 放棄規劃，而是：

- `use_named_start=false`：不先回到固定命名姿態；
- 從目前實際姿態開始；
- 使用安全分段路徑；
- 限制最大 pre-grasp 關節位移與 wrist 3 位移；
- 先 plan-only，確認後才執行。

### Cartesian path（笛卡兒路徑）

最後的下降與抬升要求 tool0 沿空間直線移動。MoveIt 會把直線切成許多小 waypoint，
每點做 IK，得到關節序列。輸出的 `path completed: 100.0%` 表示整段 waypoint 都有
有效解；若是 60%，只代表前 60% 可行，不能當完整夾取執行。

### Plan-only 為什麼重要

Plan-only 回答：「這條路徑在目前模型中能否規劃，而且看起來安全？」它不回答：

- 實際 controller 能不能準確追蹤；
- bridge 有沒有轉送；
- Newton 接觸是否成立；
- 膠條會不會被夾起。

它是一個安全且便宜的前置檢查，能避免每次調目標都直接撞地或重跑昂貴 FEM。

### Velocity scale（速度比例）

`lift_velocity_scale=0.030` 表示以規劃限制的一小部分速度抬升。降低它可以減少動態
負載與 ROS 到 Newton 的單步位移，但不能修復不存在的碰撞、錯誤的法向或錯誤的
物體座標。

## 26. 夾爪幾何校正：為什麼閉爪會讓指尖往下

Robotiq 2F-85 的手指不是兩塊只做水平平移的板。它是連桿機構；主關節轉動時，
指尖同時向內與向下走。因此「在正確高度閉爪」可能在閉合途中穿地。

我們量測不同主關節角下兩指 frame 的間距與中點高度：

```text
command 0.00 rad → separation 135.517 mm → vertical shift  0.000 mm
command 0.10 rad → separation 126.476 mm → vertical shift -3.493 mm
command 0.20 rad → separation 116.783 mm → vertical shift -6.517 mm
command 0.30 rad → separation 106.534 mm → vertical shift -9.041 mm
command 0.35 rad → separation 101.233 mm → vertical shift -10.109 mm
command 0.40 rad → separation  95.833 mm → vertical shift -11.042 mm
```

這裡的 `link_frame_separation` 是左右 link frame 間距，不一定等於真實 pad gap。
所以我們以 Robotiq 名義最大開口 85 mm 建立 pad-gap 估計，再用實際接觸驗證。

對 50 mm 寬膠條與總壓縮 2 mm：

```text
target pad gap = 50 - 2 = 48 mm
interpolated gripper command = 0.375145 rad
predicted closure drop = 10.578 mm
```

因此原始下降距離必須補償：

```text
compensated approach distance
= uncompensated approach distance - predicted closure drop
```

實際成功配置中：

```text
0.105000 m - 0.010578 m = 0.094422 m
```

白話是：夾爪閉合還會再下降約 10.6 mm，所以張開時不能先下降到最終高度；要先
停高 10.6 mm，讓閉合機構自己走完剩下的垂直距離。

這個校正解決的是**幾何位置問題**，不直接保證摩擦足夠。若補償後左右仍有空隙，
要調閉合量；若接觸有力但物體不升，要查接觸方向、接觸高度與柔體耦合。

## 27. FEM 膠條到底是什麼

### FEM（Finite Element Method，有限元素法）

真實橡膠是一個連續體，理論上每個位置都能變形；電腦不能計算無限多個點，所以把
它切成有限數量的小元素。這個膠條使用四面體元素。

### Particles / vertices（粒子／頂點）

它們是 FEM 網格中的節點，保存位置、速度、質量等自由度。材料變形最後表現在這些
節點怎麼移動。`252 particles` 不是 252 顆彼此獨立的沙子，而是同一條連續膠條的
252 個計算節點。

### Tetrahedra（四面體元素）

四個節點形成一個體積元素。材料的拉伸、壓縮與剪切能量在這些元素內計算。
`600 tetrahedra` 表示內部被切成 600 個小四面體。沒有 tetrahedra，模型只有點，
不能表示三維橡膠體積如何變形。

### Surface triangles（表面三角形）

四面體網格最外層的三角形形成可視表面，也常用於表面接觸候選生成。
`424 surface triangles` 不是額外材料，而是內部四面體邊界的外殼。

三者必須同時合理，是因為：

- particles 回答「哪些位置會動」；
- tetrahedra 回答「材料內部如何抗變形」；
- surface triangles 回答「外表面在哪裡、哪裡可能接觸」。

### Mesh resolution（網格解析度）

把 `CELLS_X` 從 20 改成 40，是把長度方向切得更細，沒有改材料 density。
同一張紙切成更多格，外觀尺寸仍相同，但計算自由度與成本增加。固定面在左端 Y–Z
截面；若 `CELLS_Y`、`CELLS_Z` 不變，固定點數仍是 `(3+1)(2+1)=12`。

更細的網格通常能描述更細的變形，但不保證結果立刻「更正確」。如果結果隨 20、
40、80 格劇烈改變，代表尚未得到網格收斂，或 solver 參數需隨解析度重新檢查。

### 材料參數

- `density`（密度，kg/m³）：同樣體積有多重，影響慣性與重力。
- `Young's modulus, E`（楊氏模數，Pa）：材料抵抗拉伸／壓縮的剛硬程度；越大越硬。
- `Poisson's ratio, ν`（蒲松比）：拉長時橫向縮小的程度。接近 0.5 表示近似不可壓縮，
  不是「不會變形」，而是體積幾乎不變但形狀仍可大幅改變。
- `damping`（阻尼）：把振動能量耗散掉；越大通常越快停止晃動，但也可能掩蓋不合理
  的動態或讓反應過黏。

由 `E` 與 `ν` 可轉成 Lamé parameters：

```text
μ = E / [2(1+ν)]
λ = Eν / [(1+ν)(1-2ν)]
```

當 `ν=0.45`，`λ/μ` 會很大，代表體積壓縮比剪切更難。數值上近不可壓縮材料反而
可能更難求解，因為 solver 必須同時允許形狀變化又強力維持體積。

## 28. VBD：Newton 如何算出膠條變形

VBD 的正確全名是 **Vertex Block Descent（頂點區塊下降法）**，不是
Vertex-Based Dynamics。它是 Newton 用於體積可變形物的求解方法之一。

概念上，每個時間步會：

1. 根據上一刻速度與外力預測節點位置；
2. 建立材料能量、固定邊界與接觸條件；
3. 逐個以頂點為區塊，更新位置以降低總能量；
4. 重複多個 iterations；
5. 由新舊位置更新速度，再進入下一個 substep。

白話比喻：膠條裡有許多互相牽制的節點。一次把全部聯立方程完美解完很貴，VBD
先調一個節點，再調下一個，反覆巡迴，直到整體變形接近穩定解。

### Substep（子時間步）

一個 ROS command 間隔會再切成多個 Newton 小步。小步越多，每一步位移越短，
快速接觸較不容易漏掉；代價是計算更久。

### Iteration（迭代次數）

每個小步內，solver 重複修正材料與接觸條件的次數。增加 iterations 通常讓約束
滿足得更好，但不能彌補完全錯誤的碰撞幾何。

### 固定端與邊界條件

先前懸臂膠條把左端 12 個節點固定，用來驗證材料與阻尼。地面夾取的膠條不能保持
兩端固定，否則機器人當然不可能把整條膠條拿走。固定端誤差為 0 只說明邊界條件
被滿足，不代表夾取接觸正確。

## 29. 碰撞接觸：從「看起來碰到」到「真的能承載」

### Collision geometry（碰撞幾何）與 visual geometry（視覺幾何）

畫面上的漂亮 Robotiq mesh 用於視覺；物理引擎需要穩定、封閉且可查詢距離的碰撞
幾何。兩者可以不同。這是機器人模擬常見做法，不是造假；必要條件是 proxy 尺寸與
位置必須校正並記錄。

### Analytic shape / collision proxy（解析幾何／碰撞代理）

Box、sphere、capsule 有封閉數學公式，可快速計算點到表面的距離與法向。Robotiq
原始 mesh 在當時 CPU 柔體接觸路徑沒有提供穩定的內部距離查詢，因此我們保留 mesh
顯示，並用隱形 analytic proxy 代表指尖接觸面。

白話：眼睛看到真實形狀，物理計算使用尺寸校正過的簡單盒子。

### SDF（Signed Distance Field，符號距離場）

SDF `Φ(x)` 回答空間一點離表面多遠：

```text
Φ(x) > 0：物體外
Φ(x) = 0：表面
Φ(x) < 0：物體內，代表穿透
```

表面法向可由梯度得到：

```text
n = ∇Φ(x) / ||∇Φ(x)||
```

若某 FEM 節點在指尖代理內 5 mm，簡化的一階位置修正概念是：

```text
C = max(0, -Φ(x)) = 0.005 m
Δx ≈ C n
```

實際 VBD 會考慮質量、材料、摩擦、接觸能量與迭代，不是把每個點無條件一次推回
表面；這個公式只用來理解穿透深度與法向的角色。

### GJK

GJK（Gilbert–Johnson–Keerthi）用 Minkowski difference 查詢兩個凸形的距離或是否
相交，常用於剛體凸形碰撞。它可提供最近點等資訊；若要取得重疊後的穿透資訊，
通常需搭配 EPA 或其他方法。

本專題的重點是：剛體 GJK 管線成功，不能自動證明 volumetric FEM 的剛柔接觸也
成功。兩者使用的資料與求解耦合不同。早期「機器人和剛體方塊能碰」只能當控制組。

### Full-surface rigid–soft contact（全表面剛柔接觸）

軟膠條需要一片分布式接觸區，把指尖壓力傳入表面節點和內部四面體。只有少量候選
點、錯誤法向或沒有連到 VBD 的接觸，可能讓畫面看似重疊，膠條卻沒有凹陷或受力。

### Normal force 與 friction

- `normal force`（正向力）：指尖垂直壓向膠條表面的力；
- `static friction`（靜摩擦）：接觸面尚未滑動時抵抗相對運動；
- `kinetic friction`（動摩擦）：開始滑動後的摩擦；
- `coefficient of friction, μ`（摩擦係數）：摩擦上限與正向力的比例。

最簡化的雙指垂直夾持條件是：

```text
2 μ N ≥ m(g + a)
```

`N` 是單側正向力，`a` 是向上加速度。這說明提高 μ 只有在 `N` 真實存在、方向正確
且接觸持續時才有用。如果接觸幾何沒有建立，`N=0`，把 μ 從 1.5 改成 10 仍是
`μN=0`。這就是我們後來停止盲目調摩擦的原因。

### Contact candidate、loaded contact 與 contact force

- `contact candidates`：幾何上可能接觸的候選，不一定承載；
- `loaded contacts`：實際有非零載荷的接觸；
- `contact-force magnitude`：力的大小，但只有大小仍不足以判斷方向；
- `force vector Fx/Fy/Fz`：才能看力是否真的有向上分量或合理的左右夾持分量。

因此「左右各有 250 個 contacts、力總和很大」仍可能 lift=0。力可能主要在把物體
壓向地板、抵銷深穿透，或作用在錯誤高度。

### 接觸 z-range 為什麼重要

若膠條高度約 20 mm，而 loaded-contact z range 集中在接近地板處，指尖可能只刮到
底角；若接觸分布在可承載的側面高度，較可能形成真正夾持。它回答「力施加在哪裡」，
比單純 contact count 更接近故障原因。

## 30. 時間同步、插值與等效速度

ROS controller 的更新頻率與 Newton substep 不同。若 bridge 每收到一筆 ROS 位置
就讓 Newton 機器人瞬間跳到新位置，物理引擎看到的等效速度是：

```text
v_effective ≈ (x_next - x_now) / Δt
```

例如單步跳 50 mm，而 Newton 步長 1 ms：

```text
v_effective = 0.05 / 0.001 = 50 m/s
```

這不是 MoveIt 原本規劃的真實速度，而是離散更新造成的假脈衝。它可能讓指尖一個
時間步跨過膠條、產生深穿透、巨大的接觸修正，甚至數值發散成 NaN。

我們的修正是 command queue + interpolation：

1. bridge 保留有時間順序的關節命令；
2. endpoint 不直接跳到最新值；
3. 在 Newton 小時間步之間插值；
4. 限制單步位置變化，讓接觸有機會被偵測與求解。

### Simulation time 與 wall time

- `simulation time`：物理世界內經過多少秒；
- `wall elapsed time`：現實中電腦算了多久；
- `real-time factor = simulation time / wall time`。

Jazzy 成功紀錄的 real-time factor 約 `0.102285`，表示模擬 1 秒約需現實 9.78 秒。
慢不代表物理錯誤；但它說明目前配置不適合即時控制，需要 GPU、網格、substeps 或
solver 設定的效能研究。

### STALE 是什麼

`bridge_status` 的 age 持續增大並顯示 `STALE`，表示 bridge 還活著，但沒有收到新的
Newton state。常見原因是 endpoint 停止、模擬迴圈卡住或 UDP 資料沒有送達。
這不是 MoveIt IK 錯誤。

### Start-state guard

規劃前比較 ROS `/joint_states` 與 Newton `/newton/joint_states`：

```text
error_i = |q_ros,i - q_newton,i|
maximum_error = max(error_i)
```

若最大誤差超過門檻，就停止執行。早期 elbow 和 wrist 2 相差約 `π/2`，已足以讓
同一條軌跡在兩個世界從不同姿態出發。Jazzy 成功時最大起始誤差約
`1.94×10^-7 rad`，表示同步良好。

### Trajectory shadow

trajectory shadow 讓 Newton 跟隨 ROS controller 的實際關節軌跡。它不是另一隻機器
人，也不是新的規劃器；它是把 ROS 的運動投影到 Newton，讓物理世界用相同動作做
接觸模擬。

## 31. 實際故障故事：每次觀察如何導出下一步

以下把前半段時間線改寫成「證據鏈」，方便理解為何我們不是隨機調參。

### 故事 A：手臂能動，但不能立刻開始地面夾取

**觀察：** controller active，MoveIt 能讓手臂走相對路徑。

**當時可證明：** URDF、六軸順序、controller、基本 IK 與 action 通訊可工作。

**仍不能證明：** Newton 起始姿態一致、物體座標正確、路徑不穿地、FEM 接觸成立。

**下一步：** 建立 start-state guard，再把相對位移改成 `/newton/object_pose` 的絕對
目標。原因是沒有共同起點與共同座標，後面所有接觸判斷都沒有意義。

### 故事 B：路徑到達目標，卻穿地或繞一大圈

**觀察：** Cartesian path 有時 100%，但視覺上曾穿地；另一些執行會先回 named
start，造成大範圍繞行。

**判斷：** IK 找到解不等於路徑符合任務；Newton 有地板不等於 MoveIt planning
scene 有地板；合法 IK 解也可能是另一個手腕分支。

**修正：** 加 floor collision object、分成 pre-grasp／vertical approach／lift、
停用 named start、限制單關節與 wrist 3 行程、每次先 plan-only。

**結果：** 最終不再繞一大圈，各段 path 100%。這只通過運動學層，還沒通過夾持層。

### 故事 C：畫面看到指尖夾到，Action 也成功，膠條仍留在地上

**觀察：** gripper action `SUCCEEDED`；左右都有接觸數；膠條 lift 為 0。

**最初假設：** 指尖高度或閉合量不對。這很合理，因為物體在地上，幾毫米就可能
讓指尖碰地或只擦到頂邊。

**驗證：** 手動掃描 gripper command，發現約 `0.35 rad` 剛好接觸；量測閉合機構
又發現 command 約 `0.375 rad` 時指尖中點下降約 `10.578 mm`。

**修正：** 建立寬度到 command 的內插，以及 closure-drop 高度補償。

**結果：** 沒有再因閉爪直接穿地，雙側 loaded contacts 與力也出現。但 lift 仍是
0。這表示幾何校正必要，卻不是全部原因。

### 故事 D：把摩擦調很大，為什麼仍抬不起來

**觀察：** `μ=10`、左右 loaded contacts、力總和數百牛頓，膠條仍不隨夾爪上升。

**推論：** 若問題只是摩擦係數不足，大幅提高 μ 應有明顯改善；完全沒有改善代表
力的方向、位置或剛柔耦合可能錯。contact count 不能證明接觸在承載。

**修正方向：** 停止繼續增加 μ，改讀 loaded contact、力向量、z-range、penetration
與 object rise。

**工程意義：** 單一參數掃描不只是在找好數值，也可以用來否證假設。

### 故事 E：為什麼我們很晚才懷疑 FEM

**原因：** 之前的膠條確實被夾起、舞動 180° 並在開爪後落下，所以「接觸曾成功」
是強證據。我們自然先查新加入的 MoveIt、地板、絕對座標與路徑。

**關鍵反省：** 早期成功物是 segmented rigid-link strip；地面任務是 tetrahedral
FEM strip。外觀相似讓我們把兩個不同物理模型當成同一個已驗證元件。

**新假設：** robot control 正常，但 FEM surface contact 沒有被正確建立或送進 VBD。

### 故事 F：rigid block 對照如何定位問題

我們保持機器人、路徑、地板與夾爪命令，將柔體換成相同尺度的剛體方塊。

**若方塊也失敗：** 優先查路徑、指尖 collision、閉合或控制。

**若方塊成功而 FEM 失敗：** 共用的機器人控制大致正常，差異集中在 FEM 表面、
soft-rigid contact、SDF/proxy 或 VBD coupling。

實驗支持後者，因此我們回到成功所需的完整 FEM 表示，而不是再亂調 MoveIt 目標。

### 故事 G：真正的物理解法

**修正組合：**

1. 使用 tetrahedral volumetric FEM，而非只有外觀或粒子線；
2. 建立與真實指尖位置和尺寸對齊的 analytic collision proxies；
3. 讓 proxy 提供連續距離／法向查詢；
4. 把 full-surface rigid–soft contact 耦合進 VBD；
5. 用 queue 與 interpolation 消除 ROS 關節位置跳躍；
6. 保持足夠 substeps、iterations 與有限 penetration limit；
7. 同時量測雙側載荷、物體上升、釋放下降與地板穿透。

**結果：** 膠條隨閉爪上升，在開爪後落下；Humble 基準成功。這時才可以說「物理
夾取成立」，而不只是畫面看起來靠近。

### 故事 H：移植到 Jazzy 為什麼第一次仍失敗

**觀察：** Jazzy 可以 build、nodes 可以啟動、GPU 可用，卻沒有重現夾取。

**錯誤結論會是：** Jazzy 不支援，或摩擦再調大。

**正確問題是：** 成功版的物理表示和資料流是否一項不漏地被搬過去？

我們比對：FEM 網格、材料、proxy、SDF/contact 路徑、插值、起始同步、MoveIt 分段
路徑與 acceptance metrics。補齊後，Jazzy 才得到約 117.3 mm 抬升與 112.3 mm
釋放下降。這證明「可以編譯」只是軟體移植，量化行為一致才是工程移植。

## 32. 如何自己看 log，而不是只相信畫面

### 第一層：系統活著嗎

看：

```text
MOVEIT_GRASP_ENDPOINT_READY
ROS_ADAPTER_READY
bridge_status = OK
```

若這層失敗，不要調摩擦或 IK。

### 第二層：起始狀態一致嗎

看：

```text
START_STATE_GUARD pass=true
maximum_error_rad < threshold
```

若失敗，先 sync；不要執行路徑。

### 第三層：目標資料合理嗎

看：

```text
MEASURED_OBJECT_POSE
OBJECT_TARGET source=newton_feedback frame=world
GRASP_ORIENTATION
WIDTH_CALIBRATION
```

檢查物體中心是否位於機器人工作空間、z 是否符合膠條半高、frame 是否為 world、
寬度與 command 是否符合本次物體。

### 第四層：規劃合理嗎

看：

```text
SAFE_PREGRASP_PLAN_ONLY_SUCCEEDED
Cartesian path completed: 100.0%
named-start detour=false
```

也要在 RViz 目視確認沒有穿地、翻腕或繞行。100% 只表示計算完成，不表示路徑符合
研究目的。

### 第五層：controller 有執行嗎

看 action result、`desired`、`actual`、`error`：

- `desired`：controller 希望到達的狀態；
- `actual`：量測或模擬硬體回報狀態；
- `error = desired - actual`：追蹤誤差；
- `reference`：controller 當下採用的參考，介面或閒置狀態下可能為空。

先前讀到 desired 與 actual 相同、error 全 0，只證明當下關節追蹤一致；它不代表
那組角度是「要去的新位置」，也不證明膠條被夾住。

### 第六層：接觸真的承載嗎

最低限度要同時看：

```text
left/right loaded contacts > 0
force direction reasonable
grasp-region lift > threshold
object retained during lift
```

只有 contact candidates 或 force magnitude 不夠。

### 第七層：釋放和數值安全嗎

看：

```text
release drop > threshold
minimum strip bottom > -penetration_limit
finite_state = true
```

如果物體從未升起，`drop after reopening = 0` 沒有驗收價值。如果 lift 有了但開爪後
不掉，可能存在隱藏 attachment、物體卡死或釋放接觸沒有解除。

## 33. 參數修改指南：一次只改一個變因

| 參數 | 它控制什麼 | 增加後常見效果 | 不能解決什麼 |
|---|---|---|---|
| `object_width_mm` | 物體寬度估計 | 推算更開的夾爪目標 | 錯誤物體座標 |
| `total_compression_mm` | 閉合超過物體寬度的量 | 正向力可能增加 | 缺少 collision/SDF |
| `uncompensated_approach_distance` | 張爪時下降基準 | 指尖更接近地面 | 閉爪下沉需另補償 |
| `lift_distance` | 抬升距離 | 更容易量到物體是否隨行 | 無效接觸 |
| `lift_velocity_scale` | 抬升速度比例 | 增加會提高動態負載 | 錯誤法向／錯誤 proxy |
| `friction` | 接觸可承受的切向力上限 | 有正向力時較不易滑 | `N=0` 的接觸 |
| `Young's modulus` | 材料剛性 | 膠條較難被壓縮或彎曲 | 漏碰撞 |
| `Poisson's ratio` | 體積保持程度 | 越近 0.5 越近不可壓縮 | 錯誤幾何 |
| `damping` | 振動耗散 | 較快停止擺動 | 靜態接觸錯誤 |
| `substeps` | 每命令間隔的物理小步數 | 快速接觸較穩、運算更慢 | 完全不存在的 proxy |
| `iterations` | 每小步的求解反覆次數 | 約束可能更收斂 | 資料流接錯 |
| `contact margin` | 開始建立接觸的距離範圍 | 更早生成候選 | 錯誤法向或錯誤 frame |
| `penetration limit` | 驗收允許的最大穿透 | 門檻放寬較易通過 | 真正修復穿透 |

工程實驗要保存 baseline，每次只改一個有假設依據的參數。例如：

```text
假設：抬升加速度超過摩擦承載能力。
操作：保持幾何、材料、閉合量不變，只把 lift velocity 降低。
量測：lift、雙側 loaded contacts、force vector、slip time。
判斷：若明顯改善，支持動態負載假設；若完全不變，查其他層。
```

「看起來好一點」不是完整實驗結果；要先寫假設與成功指標。

## 34. 故障決策表

| 症狀 | 優先懷疑 | 下一個最小測試 |
|---|---|---|
| `/newton/bridge_status` 不存在 | adapter 未啟動或環境錯 | 查 process 與 `ROS_ADAPTER_READY` |
| status `STALE` | endpoint 無新 state | 查 endpoint process／模擬迴圈 |
| `Address already in use` | 重複啟動 | `pgrep -af` 找舊 process |
| start-state guard fail | ROS/Newton 起點不同 | sync 後重跑 guard |
| IK／planning fail | 目標不可達、方向或碰撞 | plan-only，先提高 pre-grasp |
| path 100% 但穿地 | planning scene 缺地板或畫面誤判 | 檢查 floor object 與 RViz |
| 手臂繞一大圈 | named start／IK 分支／週期關節 | 停用 named start、限制 joint travel |
| action succeeded、Newton 不動 | shadow/bridge 未轉送 | 看 `/newton/joint_states` 是否變化 |
| 指尖穿過 FEM、膠條無凹陷 | soft-rigid collision 未建立 | rigid control + proxy/SDF 檢查 |
| candidates 很多、loaded=0 | 只有幾何候選，沒有承載 | 查 penetration、normal、solver coupling |
| loaded contacts 有、lift=0 | 力方向／高度／地板反力錯 | 讀 force vector 與 contact z-range |
| 提高 μ 完全沒效果 | 正向力或接觸模型錯 | 停止調 μ，驗證 `N` 與 proxy |
| 抬起後立即滑落 | 摩擦、閉合或加速度不足 | 降速或增壓，單變因比較 |
| 開爪後仍不掉 | 卡住、attachment 或接觸未解除 | 查 constraint 與 release contacts |
| NaN／爆飛 | 大位置跳躍、深穿透、步長過大 | 插值、減小 dt、增加 substeps |

## 35. 完整驗收：怎樣才可以宣布成功

本專題採兩層驗收。

### 軟體層

- 正確 OS／ROS／GPU 與 Python 環境；
- 所有 packages 能 build；
- endpoint、bringup、adapter 均 ready；
- bridge status 持續 OK；
- start-state guard 通過；
- object pose frame 與數值合理；
- plan-only 不穿地、不繞行；
- approach 與 lift path 均為 100%；
- controller actions 完成。

### 物理層

- 左右兩側在閉爪與抬升期都有承載接觸；
- 膠條 grasp region 隨夾爪明顯上升；
- 抬升期間沒有穿過指尖或手臂；
- 地板穿透小於規定門檻；
- 開爪後膠條明顯下降；
- 沒有 hidden attachment；
- 所有狀態 finite；
- 同一 commit、設定與硬體能重跑。

Humble 基準證明流程可行；Jazzy 最終紀錄進一步證明原生 Ubuntu 24.04、ROS 2
Jazzy 與 RTX 3080 能重現完整行為。這才是可提交、可報告、可繼續做材料校正的研究
基線。

## 36. 中英術語速查

| English | 中文 | 在本專題的意思 |
|---|---|---|
| forward kinematics (FK) | 正向運動學 | 關節角算末端位姿 |
| inverse kinematics (IK) | 逆向運動學 | 末端位姿求關節角 |
| pose | 位姿 | 位置加方向 |
| frame | 座標框架 | 數值所依附的參考座標 |
| transform (TF) | 座標轉換 | 兩個 frame 的相對平移旋轉 |
| planning scene | 規劃場景 | MoveIt 知道的碰撞世界 |
| pre-grasp | 預夾取姿態 | 正式下降前的安全姿態 |
| Cartesian path | 笛卡兒路徑 | 末端沿空間直線的路徑 |
| trajectory | 軌跡 | 帶時間的關節位置序列 |
| controller | 控制器 | 執行軌跡並追蹤誤差 |
| trajectory shadow | 軌跡映射 | 把 ROS 運動同步到 Newton |
| finite element method (FEM) | 有限元素法 | 用有限小元素近似連續材料 |
| particle / vertex | 粒子／頂點 | FEM 的計算節點 |
| tetrahedron | 四面體元素 | 三維材料體積單元 |
| surface triangle | 表面三角形 | 四面體網格的外表面 |
| mesh resolution | 網格解析度 | 空間離散的細緻程度 |
| boundary condition | 邊界條件 | 固定端或受限節點的規則 |
| Young's modulus | 楊氏模數 | 材料抵抗拉壓變形的剛性 |
| Poisson's ratio | 蒲松比 | 軸向變形與橫向變形的比例 |
| damping | 阻尼 | 振動能量耗散 |
| VBD | 頂點區塊下降法 | Newton 的可變形體求解方法 |
| substep | 子時間步 | 一個外部更新內的物理小步 |
| solver iteration | 求解迭代 | 每小步反覆滿足條件的次數 |
| collision proxy | 碰撞代理 | 代替複雜視覺 mesh 的簡化幾何 |
| analytic shape | 解析幾何 | 可直接用公式算距離的形狀 |
| SDF | 符號距離場 | 點到表面的帶符號距離 |
| contact normal | 接觸法向 | 表面向外、決定推開方向的向量 |
| penetration depth | 穿透深度 | 幾何進入另一物體內的距離 |
| normal force | 正向力 | 垂直接觸面的壓力 |
| static friction | 靜摩擦 | 尚未相對滑動時的摩擦 |
| kinetic friction | 動摩擦 | 已滑動時的摩擦 |
| loaded contact | 承載接觸 | 實際具有非零載荷的接觸 |
| interpolation | 插值 | 在兩筆命令間補出平滑中間值 |
| effective velocity | 等效速度 | 離散位置跳變除以時間步所得速度 |
| real-time factor | 即時倍率 | 模擬時間除以現實計算時間 |
| reproducibility | 可重現性 | 他人能用同版本與步驟得到同結果 |
| acceptance criterion | 驗收條件 | 事前定義的成功量化門檻 |

## 37. 你真正應該帶走的工程能力

你不需要比 AI 更快記住每個 API，也不需要徒手重寫 VBD solver。你要能：

1. **畫出系統邊界與資料流。** 知道 MoveIt、controller、bridge、Newton 與 viewer
   分別負責什麼。
2. **把「成功」拆成可量測條件。** 不用「看起來有動」取代 lift、release、contact、
   penetration 與 finite-state 證據。
3. **從觀察提出可否證假設。** 提高摩擦沒有改善，就要願意放棄「只是摩擦不足」。
4. **使用控制組定位層級。** 剛體方塊成功、FEM 失敗，能把搜尋範圍縮到剛柔接觸。
5. **分清必要條件與充分條件。** action success、path 100%、contact count 都是必要
   證據之一，但沒有任何一項單獨足以證明抓取成功。
6. **一次改一個變因並保存 baseline。** 否則結果改善時也不知道是哪個修改造成。
7. **用版本與紀錄保護成果。** commit、tag、重建腳本、log、影片與硬體資訊共同
   構成可重現研究，不是只有一支「曾經跑過」的程式。
8. **能向教授說明失敗如何產生新知。** 這段歷程的價值不只是最後夾起來，而是我們
   從幾何、運動學、控制、時間離散一路定位到剛柔接觸表示，並用對照實驗完成驗證。

這些能力才是機器人工程師的核心：AI 可以幫你寫程式、查 API、整理 log；你負責
定義問題、判斷證據、選擇下一個實驗，並決定結果是否可信。
