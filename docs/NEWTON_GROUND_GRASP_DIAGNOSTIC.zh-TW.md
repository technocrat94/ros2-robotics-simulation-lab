# MoveIt–Newton 地面夾取診斷筆記

更新日期：2026-10-01

這份文件是故障診斷索引。完整心路歷程與 VBD 教學請看
[`NEWTON_FEM_GRASP_LEARNING_JOURNEY.zh-TW.md`](NEWTON_FEM_GRASP_LEARNING_JOURNEY.zh-TW.md)；
最後可重建的成功配方請看 [`HUMBLE_FEM_PICK_SUCCESS.zh-TW.md`](HUMBLE_FEM_PICK_SUCCESS.zh-TW.md)。

## 目前已證明什麼

- Newton 能把膠條的 `world` 絕對位置傳給 ROS。
- MoveIt 能規劃到膠條上方、垂直下降、閉合與抬升。
- MoveIt planning scene 已加入地板。
- 夾爪寬度校正會同時計算閉合角度與閉合造成的指尖下降量。
- Newton 能量測左右接觸數、承載接觸、接觸力大小、穿透與物體升高量。

這些是初期驗證，證明規劃、命令傳輸和接觸量測成立，但當時尚未證明地面膠條
被成功夾起。

## 夾爪校正

50 mm 膠條、總壓縮量 2 mm 時：

- 目標間距：48 mm
- 夾爪主關節：`0.375145 rad`
- 閉合時指尖中點下降：約 `10.578 mm`

因此手臂下降距離必須扣除這個閉合下降量。這是**運動學校正
(kinematic calibration)**，實際碰撞面仍要由 Newton 接觸資料驗證。

## 這次最重要的工程判斷

手動閉合約 `0.394 rad` 時，Newton 測得：

- 左／右承載接觸：`25 / 22`
- 左／右接觸力大小總和：`231.335 / 250.229`
- 抬升量：`0 m`

所以兩側確實「有接觸、有力」，但夾爪一上升，膠條立刻留在地面。

接觸力的**大小 (magnitude)** 不等於有效夾持。力量可能沿膠條長度推動、
壓向地板，或在抬升第一瞬間消失。把摩擦係數提高到診斷值 10、把總壓縮
量提高到 4 mm 仍無效，因此暫時排除「只要更粗糙或夾更緊」的假設。

## 下一步

重置乾淨初始狀態，記錄左右指尖接觸力的世界座標分量：

- `|Fx|`：是否沿 400 mm 長度把膠條推出去；
- `|Fy|`：是否在 50 mm 寬度方向形成左右夾緊；
- `|Fz|`：抬升時是否存在向上的摩擦力。

取得方向資料前，不再增加摩擦或壓縮量。這是在練習工程師最重要的能力：
用量測區分「有碰撞」與「能承載的夾持」。

## 結案：直接 FEM 地面夾取成功

後續受控測試保留 FEM、接觸、摩擦、夾爪校正與抬升參數，只把
`use_named_start` 從 `true` 改成 `false`，因此不再先繞到
`test_configuration`。

MoveIt 的安全 pre-grasp、垂直 approach、垂直 lift 路徑皆為 `100%`；夾爪命令
`0.375145 rad`，抬升 `0.120 m`，velocity scale `0.030`。操作者在 Newton
畫面確認：沒有大繞路、膠條隨閉合夾爪上升、只有重新開爪後掉落。

終端最後顯示 `ABSOLUTE POSITION PICK SUCCEEDED`。這個版本已在 commit
`3546672` 成為 Humble 成功基準。前面的失敗數據仍然保留，因為它們證明只有
接觸數或力的大小不能作為夾持成功判斷。自動 rise、retention、penetration、
release 驗收仍是下一步。

## 版本方向

家中 Ubuntu 22.04 / ROS 2 Humble 保留為本次可重現基準。地面夾取成功並
提交後，學校 Ubuntu 24.04 / ROS 2 Jazzy 建立獨立移植版；通過對照測試後，
未來新功能以 Jazzy 為主。
