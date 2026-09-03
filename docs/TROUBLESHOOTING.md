# Troubleshooting Record

這份紀錄保留「看到什麼、如何判斷、怎麼修」；它比只保存成功程式更能呈現工程思考。

| 現象 | 判斷依據 | 處理方式 | 學到的事 |
|---|---|---|---|
| Xacro 顯示 `Undefined substitution argument name` | 單獨執行 `xacro` 即可重現 | 補齊 Xacro argument/default，先輸出 URDF 再搜尋重要 link/control | 先驗證模型生成，再啟動整套 ROS |
| launch 檔在 package share 中找不到 | 原始檔存在，但 `ros2 launch` 報 not found | 在 CMake 安裝 `launch`、`urdf`、`config` 等目錄並重新 build/source | ROS 執行的是 install space，不是直接猜測 src 內容 |
| MoveIt 到命名姿勢規劃中止、error `-26` | Action Server 已連上，但 Planning request aborted | 任務節點改用本專案組合 SRDF、kinematics 與正確 robot description | URDF、SRDF 與 MoveIt group 必須描述同一台機器人 |
| Planning 成功但 Execute aborted | 已產生 trajectory，送出 execution 後立即失敗 | 對齊 MoveIt controller mapping 與 active `joint_trajectory_controller` | 規劃層和控制層是兩個不同關卡 |
| RViz 看不到機械手 | Global Status 顯示 `Frame [map] does not exist` | Fixed Frame 改為實際存在的 `world` | 視覺化也依賴 TF，模型存在不代表畫面一定能顯示 |
| RViz 同時出現橘色與灰色手臂 | 灰色會隨 `/joint_states` 更新，橘色代表規劃/目標狀態 | 保留作為比較，必要時在 MotionPlanning display 調整顯示 | 目標模型不是第二台實體機器人 |
| 小幅移動卻出現繞圈或大角度關節旋轉 | 末端終點接近，但 joint trajectory 行程過大 | 改用 Cartesian path，固定姿態並加入 1 rad 關節行程保護 | 相同末端終點可能有多組 IK；不能只看終點 |
| `up` 能到達，但 Approach 為 0% | 命名姿勢成功，直線插值沒有可行路徑 | 回到已驗證的 `test_configuration` | 可到達某姿勢不代表從該姿勢的指定直線也可行 |
| 較大 Approach 只有 12.5% 或 93.4% | `computeCartesianPath()` 回報 fraction 不足 | 拒絕執行並縮回已驗證的 3 cm 位移 | 失敗保護本身也是成功的系統行為 |
| `/joint_states` 偶爾顯示 message lost | 仍能取得最新完整 joint state | 觀察頻率與 QoS；只要狀態持續更新，不把單次警告誤判為控制器故障 | 診斷要看持續狀態，而不是只看一行訊息 |
| `ros2` CLI 出現 `rclpy.ok()` daemon 錯誤 | Controller/bringup 仍可能正常，但 CLI 查詢失敗 | `ros2 daemon stop` 後 `ros2 daemon start`，再重查 | CLI discovery daemon 與正在執行的機器人節點不是同一件事 |

## 建議診斷順序

遇到問題時，依照由底層到上層的順序檢查：

```text
1. Controller 是否 active
2. /joint_states 是否持續更新
3. Gripper Action Server 是否存在
4. TF 與 RViz Fixed Frame 是否存在
5. MoveIt 是否載入正確 robot description / SRDF
6. Planning 是否成功
7. Execution 是否成功
8. Cartesian fraction 與關節行程是否通過安全門檻
```

這像排查家中「燈不亮」：先看總電源，再看開關、燈座與燈泡；不應一開始就拆整面牆。

## 已驗證基準

若新參數失敗，先回復以下基準確認整套系統仍正常：

```text
Named target:       test_configuration
Approach:           (+0.03, 0.00, 0.00) m
Lift and transport: (-0.03, 0.00,+0.05) m
Automatic return:  ( 0.00, 0.00,-0.05) m
Velocity scaling:   0.2
Acceleration scale: 0.2
Cartesian minimum:  99%
Joint-travel guard: 1 rad
```

成功紀錄應包含三段 Cartesian path 皆為 `100.0%`，並以 `PICK AND PLACE DEMO SUCCEEDED` 結束。
