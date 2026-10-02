# Jazzy FEM Ground-Pick Success — 2026-10-02

## English

Native ROS 2 Jazzy completed the direct FEM ground pick from the Humble-success IK start pose. The measured object pose came from `/newton/object_pose`; safe pre-grasp, approach, and lift paths were complete, with no named-start detour. Newton remained finite: the grasp region rose by `0.117290587 m`, then dropped `0.112259318 m` only after reopening. Bilateral contact samples were recorded during closure (`34`) and lift (`229`), and the minimum strip bottom was `-0.000820466 m`. The viewer was visually checked: the strip remained between the fingers during lift and was released after reopening.

See [jazzy_fem_pick_success_2026-10-02.json](jazzy_fem_pick_success_2026-10-02.json) and the standalone [jazzy_fem_pick_result_2026-10-02.json](jazzy_fem_pick_result_2026-10-02.json).

## 中文翻譯

原生 ROS 2 Jazzy 已從 Humble 成功 IK 姿態附近完成無繞路 FEM 地面夾取。物件位置來自 `/newton/object_pose`；安全前置、下降與抬升路徑均完整，沒有 named-start 繞路。Newton 維持 finite state：夾取區上升 `0.117290587 m`，並只在重新張爪後下降 `0.112259318 m`。閉爪與抬升期間分別記錄雙側接觸樣本 `34` 與 `229`，膠條最低底部高度為 `-0.000820466 m`。viewer 已目視確認膠條在兩指之間上升，並在重新張爪後釋放。
