# Newton × ROS 2：中文學習索引

> 英文文件保存可提交給教授與面試者的完整技術證據；中文文件用於複習概念、工程判斷與口頭問答。原本的累積長筆記已按主題拆分，本頁保留為穩定入口。

## 主題文件

| 主題 | English technical record | 中文學習筆記 |
|---|---|---|
| ROS 2 bridge、時間、URDF、mimic 與運動學 | [Newton–ROS 2 Integration](NEWTON_ROS2_INTEGRATION.md) | [Newton 導入 ROS 2](NEWTON_ROS2_INTEGRATION.zh-TW.md) |
| FEM、分段柔性模型與收斂 | [Newton Soft-Strip Experiments](NEWTON_SOFT_STRIP_EXPERIMENT.md) | [Newton 柔性條建模實驗](NEWTON_SOFT_STRIP_EXPERIMENT.zh-TW.md) |
| 接觸夾持、失敗推導與量測修正 | [Newton Robot Contact-Grasp](NEWTON_ROBOT_GRASP_EXPERIMENT.md) | [Newton 機械手接觸夾持](NEWTON_ROBOT_GRASP_EXPERIMENT.zh-TW.md) |

## 里程碑順序

1. 隔離 ROS 2 Python 3.10 與 Newton Python 3.12。
2. 建立雙向 bridge，驗證命令、狀態與 `STALE` 偵測。
3. 分清 ROS time、simulation time、wall time 與 physics `dt`。
4. 展開 UR5＋Robotiq URDF，檢查主動與 mimic 關節。
5. 解析 mesh URI，驗證 shapes 與 forward kinematics。
6. 由 ROS 控制 Newton 中的預設關節運動並回傳狀態。
7. 建立 FEM 橡膠條並檢查 X 向網格收斂。
8. 建立剛體分段柔性近似並與 FEM 比較。
9. 以 Newton 接觸與摩擦完成中心夾持、抬升與釋放。

## 建議閱讀方式

- 快速了解作品：先讀 repository `README.md`。
- 準備英文報告：讀對應的 English technical record。
- 準備教授追問：讀相同主題的中文學習筆記與「教授可能會問」。
- 重現程式：使用英文文件中的 reproduction 指令及 prototype 路徑。

## 核心工程主線

```text
MoveIt 規劃希望的機器人軌跡
              ↓
ROS 2 傳送命令並接收狀態
              ↓
Newton 計算接觸、重力、變形、滑動與釋放
```

目前已驗證 bridge、機器人運動學、兩種柔性條模型及預先放置的接觸夾持。下一階段是以 MoveIt 軌跡取代 Newton 測試程式內預先指定的關節運動。
