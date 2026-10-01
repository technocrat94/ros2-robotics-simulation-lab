# Humble FEM 地面夾取成功紀錄

這次封存的是 UTM 上通過最終整合測試的參考版本：

- MoveIt 讀取 Newton 回傳的物體中心位置。
- `use_named_start=false`，手臂不再先繞到 `test_configuration`。
- 以安全的 Cartesian 路徑移到物體上方，再垂直下降。
- 依 50 mm 物寬與 2 mm 壓縮量計算夾爪命令。
- Newton 使用 VBD、解析指尖碰撞代理與全表面剛柔接觸。
- 操作者在 Newton 畫面確認：沒有大繞路、膠條被抬起、開爪後才釋放。
- MoveIt 最後回報 `ABSOLUTE POSITION PICK SUCCEEDED`。

主要數據：物體中心 `(0.4869, 0.1093, 0.0110) m`、夾爪命令
`0.375145 rad`、閉合下移補償 `10.578 mm`、抬升 `0.120 m`、
MoveIt 記錄段約 `12.765 s`。

這是一個單一變因測試：保留今天已成功的 FEM、碰撞代理、全表面接觸、
夾爪與下降參數，只把 `use_named_start` 從 `true` 改為 `false`。結果仍能
夾起與放開，因此可以判斷先前的大繞路並非成功夾取所需條件。

工程判斷要分兩層：log 的 `ABSOLUTE POSITION PICK SUCCEEDED` 證明
MoveIt 與控制器完成命令；膠條真的上升且開爪後掉落，才證明物理夾取成功。
本次兩層皆通過，但物理結果仍是人工觀察，尚未加入自動升高與釋放斷言。

Newton 在這台 UTM CPU 的長時間估算約為 `0.084× realtime`，也就是
一秒模擬時間約需 `11.9` 秒實際時間。這不是單次實驗的精確計時，
下一版會在每次夾取前後各記錄一次模擬時間。

執行與重建方式請看
[`HUMBLE_FEM_PICK_SUCCESS.md`](HUMBLE_FEM_PICK_SUCCESS.md)。學校的 Jazzy
版本必須重現相同行為後，才算移植成功。
