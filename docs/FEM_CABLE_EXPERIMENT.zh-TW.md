# Newton FEM 細線實驗：中文學習筆記

> [English technical record](FEM_CABLE_EXPERIMENT.md) · [中文主題索引](NEWTON_ROS2_LEARNING_LOG.zh-TW.md)

狀態：實驗中，尚未合併到已驗證的 FEM 地面抓取基線。

本實驗把原本 `0.40 × 0.05 × 0.02 m` 的 FEM 軟條，改成
`0.40 × 0.006 × 0.006 m` 的方形截面細線近似。第一階段不是直接讓機械手抓取，而是先確認網格、質量、CUDA、穩定性與長度保存都合理。

## Cells、particles 與 tetrahedra

設定 `40 × 2 × 2` 的意思是：長、寬、厚三個方向分別有 40、2、2 個有體積的小格子，因此：

```text
cells = 40 × 2 × 2 = 160
```

particles 是小格子角落的節點。相鄰 cells 會共用節點，所以每個方向都比 cell 數多一：

```text
particles = (40+1)(2+1)(2+1)
          = 41 × 3 × 3
          = 369
```

可以把它想成 41 層截面，每層有 `3 × 3 = 9` 個 particles。Newton 把每個 cell 切成 5 個四面體，所以：

```text
tetrahedra = 160 × 5 = 800
```

`fix_left=True` 會固定最左邊的一層截面，也就是 9 個 particles。固定粒子的 mass 設為零，代表它們不受力移動；剩下：

```text
dynamic particles = 369 - 9 = 360
```

## 為什麼要修正傳給 Newton 的 density

真實質量應由材料密度乘上幾何體積：

```text
physical mass = rho × L × W × T
              = 1100 × 0.4 × 0.006 × 0.006
              = 0.01584 kg
```

但 Newton 1.5.1 這條 `add_soft_grid` 路徑會替每個動態 particle 配上一份 cell-volume mass。若直接傳入 `1100 kg/m³`，質量會按 360 個動態節點累加，而不是按 160 個 cells 累加：

```text
mass ratio = 360 / 160 = 2.25
actual nodal mass before correction = 0.03564 kg
```

因此材料的真實密度仍保留為：

```python
DENSITY_KG_M3 = 1100.0
```

只對傳進 Newton 的數值密度做離散補償：

```text
grid_density = material_density × cells / dynamic_particles
             = 1100 × 160 / 360
             = 488.888... kg/m³
```

驗算後，360 個動態節點的質量總和會回到 `0.01584 kg`。`488.888` 不是材料的真實密度，而是配合 Newton 節點質量配置的數值輸入。

## 為什麼「穩定」不等於「物理正確」

修正質量前得到兩組觀察：

| 模擬時間 | damping | 最後一秒擺幅 | 最終末端下垂 | 判讀 |
|---:|---:|---:|---:|---|
| 4 s | 100 Pa·s | 0.27194 m | 0.30459 m | 數值有限，但強烈振動 |
| 12 s | 1000 Pa·s | 0.000949 m | 0.49928 m | 已停止振動，但幾何不合理 |

第二次的擺幅低於預先設定的 4 mm 門檻，但固定端在 0.50 m、原始線長只有 0.40 m；未伸長時末端最低只能到約 0.10 m。結果卻接近地面，代表細線有不合理伸長。

修正節點質量後的第三次 12 秒測試證明「質量不是唯一問題」：預期與實際質量的相對誤差只有 `1.47e-7`，但 FEM 中心線最大伸長仍達 `64.1%`、結束時仍伸長 `32.7%`，地面接觸最高 135 點，最後一秒擺幅 `12.44 mm`，也沒有通過 4 mm 穩定門檻。

因此驗證順序必須是：

```text
finite → fixed boundary → settled → correct mass → bounded strain → contact
```

只看到動畫正常、`finite_state=true` 或 `settled=true`，都不足以宣稱模型物理正確。

## 為什麼改用 Newton rod cable

你提出的需求是：

```text
軸向幾乎不可拉長
可以朝左右／上下彎曲
可以沿自身軸線扭轉
有足夠阻尼停止振動
```

這些性質不能只靠一個 FEM Young's modulus 分開控制。Newton 1.5.1 的 `add_rod()` 會建立一串 capsule bodies，並用 cable joints 分別提供：

| 參數 | 抵抗的變形 | 單位 |
|---|---|---|
| `stretch_stiffness` | 沿線方向拉長或壓縮 | N/m |
| `shear_stiffness` | 相鄰節段橫向錯開 | N/m |
| `bend_stiffness` | 上下與左右彎曲 | N·m/rad |
| `twist_stiffness` | 沿 cable 軸線扭轉 | N·m/rad |
| 各自的 `damping` | 消耗對應運動的振動能量 | 對應速度單位 |

第一版 rod 使用：

```text
直徑              = 6 mm（圓形截面）
segments          = 40
stretch stiffness = 1,000,000 N/m
shear stiffness   = 1,000,000 N/m
bend stiffness    = 0.01 N·m/rad
twist stiffness   = 0.005 N·m/rad
```

stretch/shear 很大，用來抑制不合理伸長與橫向斷開；bend/twist 小很多，讓 cable 仍可向多軸彎曲與扭轉。這些是第一輪工程參數，不是實測材料值。

`add_rod()` 的每一節是含圓柱與半球端的 capsule，相鄰端帽會重疊。若直接使用 `1100 kg/m³`，重疊體積會重複計算質量。因此輸入 Newton 的 capsule density 補償成 `785.714 kg/m³`，但真實材料密度仍記錄為 `1100 kg/m³`。補償後圓形 cable 總質量是：

```text
rho × pi × r² × L = 0.0124407 kg
```

它比方形 6×6 mm FEM 的 `0.01584 kg` 輕，是因為圓面積小於方形面積，不是質量計算再次出錯。

第一次 rod 執行中，根部誤差維持為零、狀態有限，最終伸長只有 `0.573%`，已遠低於 FEM；但暫態最大伸長仍有 `2.42%`，最後一秒末端擺幅仍為 `52.15 mm`。最低表面高度是 `99.5 mm`，證明 cable 沒有碰到地板。

當時顯示的 947 個 rigid contacts 也不是地面接觸。Newton builder 的預設 `rigid_gap` 是 `0.1 m`，相對於 6 mm cable 太大，造成相距 10 cm 內的許多非相鄰節段都成為接觸候選。現在 rod cable 明確使用：

```text
contact gap = 0.001 m
```

程式也把 `maximum_ground_contacts` 與 `maximum_self_contacts` 分開輸出。下一次維持 stiffness 和 damping 不變，先單獨觀察 contact-gap 修正的效果，避免一次改兩個原因。

重跑結果是 `maximum_ground_contacts=0`、`maximum_self_contacts=0`，而所有運動數值與前次完全相同。這證明先前 947 筆只是沒有實際施力的接觸候選。移除它們後 wall time 從約 54.9 秒降為 44.1 秒，速度提升約 20%，但軌跡沒有改變。

剩下的失敗已隔離成兩項 dynamics：最後一秒仍擺動 `52.15 mm`，暫態最大伸長 `2.42%`。程式現在提供 `--damping-scale`，會同時倍增 stretch、shear、bend、twist damping，但完全不改 stiffness、質量、幾何與接觸。下一個單一變因測試使用 4 倍阻尼；只有在振動改善後伸長仍超過 1%，才會另外調整 stretch stiffness。

4 倍阻尼測試把 12 秒時的最後一秒擺幅從 `52.15 mm` 降為 `31.61 mm`，最終伸長仍低於 1%。這證明阻尼會加速振幅衰減，但不能證明 4 倍阻尼更接近真實電線。沒有實體 cable 的自由衰減量測前，不應只為了通過門檻任意加大 damping；下一輪回到原始 `damping-scale=1`，改成觀察 30 秒。

驗收也改成分開回答兩個問題：全程最大伸長仍保留，記錄突然施加重力造成的啟動衝擊；cantilever 平衡驗收則使用最後一秒的最大伸長。程式新增 `tip_range_by_second_m`，可直接看到每一秒的振幅是否逐步下降。未來機器人快速移動時，再另外建立動態伸長驗收，不能與靜態平衡混在一起。

30 秒、原始 `damping-scale=1` 的結果已通過：每秒末端擺幅由
`397.49 mm` 持續下降至 `3.98 mm`，最後一秒最大伸長 `0.534%`，固定端誤差為零，
`candidate_rod_cable_pass=true`。因此暫定模型保留原始阻尼，不採用 4 倍阻尼。

## CPU、GPU 與 device index

CPU 與 GPU 都會工作，但角色不同：

| 裝置 | 工作 |
|---|---|
| CPU | Python 控制流程、Viser、JSON、NumPy 統計 |
| GPU | Newton FEM model arrays、collision kernels、VBD solver |

`cuda:0` 是程式目前看見的第一張 GPU。本機是 RTX 3080。`fem_cable_simulation.py` 會拒絕 CPU，並在 `builder.finalize(device=device)` 時把 Newton model 明確配置到指定 GPU。

若未來設定 `CUDA_VISIBLE_DEVICES=1`，實體 GPU 1 會成為程式內的 `cuda:0`；這是 CUDA 的可見裝置重新編號，不是選錯卡。

## 地板摩擦不是零

`builder.add_ground_plane()` 使用 Newton 預設 `ShapeConfig.mu=1.0`，目前 cable 測試也設定 `model.soft_contact_mu=1.0`。因此地板不是無摩擦。已驗證的 FEM 抓取則另外明確設定 `GRASP_GROUND_FRICTION=1.5`。

## MoveIt 到底計算什麼

1. Newton 從 FEM particles 計算物件中心，經 `/newton/object_pose` 傳給 ROS。
2. MoveIt 使用量到的 `x/y/z` 規劃安全 pre-grasp 與垂直 approach，並加入地板碰撞幾何。
3. 專案自己的 gripper calibration 以 `object_width_mm - total_compression_mm` 求目標 pad gap，再內插 gripper angle 與閉合時的指尖下降量。

MoveIt 不會自動量出 cable 直徑。Gripper 校正已用相同 URDF 運動學延伸至
`0.16–85.00 mm`；初次 cable 測試指定 `object_width_mm=6`、compression `0 mm`，
由 `0.70–0.75 rad` 兩筆資料內插。這只是幾何預測，真正接觸仍須由 Newton 左右 loaded
contact 驗證。

## 下一階段

1. 用修正質量後的 cantilever 量測中心線長度與最大伸長率。（已完成）
2. 建立 `fix_left=False`、放在明確摩擦地板上的自由 cable。（已實作）
3. 擴充 Robotiq 小間隙校正。（已實作）
4. 先做 MoveIt plan-only 路徑預覽。（下一次執行）
5. 再做含雙側 rigid-contact 驗收的慢速抬升與放開。
6. 最後才加入圓柱 peg 與 routing。

成功的 `fem_strip` 模式不會被覆蓋；新模型以獨立的 `rod_cable` 模式加入，保留 A/B 比較與回歸驗證能力。

## 重現指令

```bash
cd ~/ur5_ws/src/ur5_moveit_demo/docs/experiments/newton-soft-strip/prototype

~/newton_ws/.venv-cpu/bin/python fem_cable_topology.py

CUDA_VISIBLE_DEVICES=0 ~/newton_ws/.venv-cpu/bin/python \
  fem_cable_simulation.py \
  --device cuda:0 \
  --cells-x 40 \
  --duration 12 \
  --damping 1000 \
  --start-delay 1 \
  --port 8084
```

`.venv-cpu` 只是歷史名稱；程式會印出實際 device 並拒絕在非 CUDA 裝置執行。

Rod cable 使用另一個 port，不會和 FEM viewer 混淆：

```bash
~/newton_ws/.venv-cpu/bin/python rod_cable_topology.py

CUDA_VISIBLE_DEVICES=0 ~/newton_ws/.venv-cpu/bin/python \
  rod_cable_simulation.py \
  --device cuda:0 \
  --duration 30 \
  --damping-scale 1 \
  --start-delay 1 \
  --port 8085
```

結果會輸出伸長率、最大 joint gap、固定根部誤差、地板接觸數與 `candidate_rod_cable_pass`。0.40 m cable 從 0.50 m 高處伸出，若軸向長度保持合理，就不應接觸地板。

## MoveIt 夾取階段

30 秒原始阻尼測試得到 `candidate_rod_cable_pass=true` 後，rod cable 已接入
既有 Jazzy MoveIt–Newton bridge，但使用獨立物件模式和啟動腳本，沒有取代已通過的
FEM baseline。四個 terminal、6 mm 夾爪校正、plan-only 安全關卡與物理接觸驗收請見
[CABLE_MOVEIT_PICK.zh-TW.md](CABLE_MOVEIT_PICK.zh-TW.md)。

2026-10-08 的第一次整合執行尚未通過：MoveIt 路徑全部完成，但 cable 在 lift
初期失去雙側接觸，抓取區只上升約 `9.56 mm`。這個失敗被保留作為下一輪接觸幾何與
抓取高度診斷的基準，不會靠任意增加 friction 或 damping 抹掉。
