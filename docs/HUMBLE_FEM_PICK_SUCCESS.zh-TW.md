# Humble FEM 地面夾取成功紀錄

這次封存的是 UTM 上第一個接受的完整參考版本：

- MoveIt 讀取 Newton 回傳的物體中心位置。
- 先抬高、調整姿態、移到物體上方，再垂直下降。
- 依 50 mm 物寬與 2 mm 壓縮量計算夾爪命令。
- Newton 使用 VBD、解析指尖碰撞代理與全表面剛柔接觸。
- 操作者在 Newton 畫面確認膠條被抬起，開爪後才釋放。
- MoveIt 最後回報 `ABSOLUTE POSITION PICK SUCCEEDED`。

主要數據：夾爪命令 `0.375145 rad`、閉合下移補償
`10.578 mm`、抬升 `0.120 m`、完整流程約 `19.231 s`。

Newton 在這台 UTM CPU 的長時間估算約為 `0.084× realtime`，也就是
一秒模擬時間約需 `11.9` 秒實際時間。這不是單次實驗的精確計時，
下一版會在每次夾取前後各記錄一次模擬時間。

執行與重建方式請看
[`HUMBLE_FEM_PICK_SUCCESS.md`](HUMBLE_FEM_PICK_SUCCESS.md)。學校的 Jazzy
版本必須重現相同行為後，才算移植成功。
