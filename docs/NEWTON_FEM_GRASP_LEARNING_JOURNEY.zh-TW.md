# MoveIt 到 Newton FEM 地面夾取：完整工程敘事

狀態：Humble 成功基準為 Git commit `3546672`；本文件記錄從 MoveIt 整合開始的
判斷、誤判、對照實驗、修正原理與最終驗收。

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
