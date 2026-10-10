# FEM cable × MoveIt：正確細線候選

這個候選沿用已驗證成功的 FEM strip 夾取管線，不再把細線近似成剛性 capsule
chain。`rod_cable` 保留為對照實驗，不再作為最終柔性夾取模型。

## 保留與改變

保留：

- MoveIt IK、路徑規劃與 start-state guard；
- ROS–Newton bridge、命令 queue 與時間插值；
- VBD、四面體 FEM 與 full-surface rigid–soft contact；
- 四個 Robotiq analytic box contact proxies；
- 已驗證材料參數、摩擦與 2 mm nominal compression。

只改變：

- 尺寸由 `400 × 50 × 20 mm` 改為 `400 × 6 × 6 mm`；
- 家用 CPU 網格為 `40 × 2 × 2` cells：369 particles、800 tetrahedra；
- 學校 GPU 網格為 `80 × 2 × 2` cells：729 particles、1600 tetrahedra；
- MoveIt object width 改為 `6 mm`，compression 保持 `2 mm`。

這裡簡化的是指尖碰撞幾何與家用網格解析度；細線本身仍是可壓縮、可彎曲、可傳遞
分布式接觸力的 FEM 柔體。

## 啟動

Terminal 1 與 2 沿用 Jazzy bringup 與 `ros_adapter`。

Terminal 3，家用 ARM64 UTM：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_fem_cable_grasp_home_cpu.sh
```

學校 RTX：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_newton_fem_cable_grasp_school_gpu.sh
```

Terminal 4 先預覽：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo
./scripts/run_jazzy_fem_cable_preview.sh
```

目前手臂若已在物件正上方，可避免安全路徑的反向折點：

```bash
FEM_CABLE_SAFE_PREGRASP=false ./scripts/run_jazzy_fem_cable_preview.sh
```

預覽通過後才執行：

```bash
FEM_CABLE_SAFE_PREGRASP=false ./scripts/run_jazzy_fem_cable_pick.sh
```

## 驗收

啟動 log 必須顯示 `object=fem_cable` 的 FEM 模式，以及符合 profile 的 particles 與
tetrahedra。夾取成功還必須同時滿足：

- 關閉階段左右都有 soft contact；
- 抬升階段 bilateral soft contact 持續存在；
- grasp region 上升接近命令的 `0.12 m`；
- 打開夾爪後才明顯下降；
- 穿透低於 `5 mm` 且 `finite_state=true`。

MoveIt 顯示 `SUCCEEDED` 只代表機械手軌跡完成，不能取代上述 Newton 物理證據。
