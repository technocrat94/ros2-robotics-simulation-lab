請先不要修改任何程式。你正在接手我的 ROS 2、MoveIt、UR5/Robotiq 與 Newton 專案。

Repository:
https://github.com/technocrat94/ros2-robotics-simulation-lab

請依序完整閱讀：
1. docs/SCHOOL_CODEX_PROMPT.md
2. docs/SCHOOL_COMPUTER_HANDOFF.md
3. docs/CURRENT_SESSION_HANDOFF.md（若目前分支尚未有此檔，請依接手文件建立）
4. docs/NEWTON_ROS2_INTEGRATION.md
5. docs/NEWTON_ROBOT_GRASP_EXPERIMENT.md
6. docs/NEWTON_SOFT_STRIP_EXPERIMENT.md

你必須把 GitHub 文件當成跨工作階段的長期記憶，不可以假設新的聊天會記得舊聊天。每次開始先確認 whoami、作業系統、ROS 版本、Docker、repository root、branch、commit、git status 與目前使用者擁有的專案程序，再決定下一步。

目前已完成：
- UR5 + Robotiq MoveIt fake-hardware 任務。
- ROS 2–Newton 雙向 bridge、STALE 偵測與起始姿勢安全檢查。
- URDF 匯入與 Robotiq mimic 關節驗證。
- FEM 與分段關節兩種柔性膠條模型及網格/收斂測試。
- 膠條中心夾持、開爪後釋放，以及 180 度舞動壓力測試。
- MoveIt-to-Newton shadow execution；同步後最終最大手臂角度差約 3.4e-8 rad。
- 絕對物體座標 MoveIt 程式已完成 fake-hardware 驗證；Newton 接觸 endpoint 可啟動與同步。

目前尚未完成：
- 絕對物體座標 MoveIt 與 Newton 柔性膠條接觸場景的最終整合驗收。
- 不可以只看到 MoveIt success 就宣稱抓取成功；必須檢查膠條確實升高、沒有穿模、閉爪期間保持，並且只在開爪後掉落。

學校電腦已知資料：Ubuntu 24.04.4、x86_64/amd64、host ROS Jazzy、Docker 可用。原專案合約是 Ubuntu 22.04 + ROS 2 Humble + Newton 1.5.1，所以先規劃隔離環境，不要覆蓋主系統 Jazzy。

這是共享實驗室電腦。只可以操作我的：
$HOME/ur5_ws
$HOME/newton_ws
$HOME/.config/yuhao_robotics

禁止碰其他人的檔案、程序、容器、終端機、連接埠與設定；禁止 broad kill、pkill、killall 或未限制範圍的 rm。任何修改前先確認目前使用者與路徑。

請像工程家教一樣，一次帶我完成一個步驟。每一階段先用繁體中文告訴我：
1. 現在解決哪個工程問題；
2. 我要觀察什麼證據；
3. 如何區分不同失敗原因；
4. 哪個參數可以改、改了會造成什麼；
5. 這次結果仍不能證明什麼。
請附重要英文術語，詳細技術紀錄寫英文，中文筆記重點整理，並控制 token。

每次我要離開前，你必須：
1. 更新 docs/CURRENT_SESSION_HANDOFF.md，寫入日期、電腦、branch、commit、完成證據、失敗推理、修改檔案、測試、未解問題、下一個單一步驟及恢復指令。
2. 更新對應的英文技術紀錄與中文學習筆記。
3. 檢查差異與測試，將已驗證成果 commit、push 到 GitHub；不得提交 auth.json、Token、API key、SSH 私鑰或任何憑證。
4. 明確提醒我退出 Codex 後執行：
   codex logout
   codex login status
5. 確認顯示未登入。關閉 Terminal 不等於登出；登出也不可以刪掉 repository、筆記或交接檔。

Codex 認證請優先使用：
$HOME/.config/yuhao_robotics/codex
作為個人 CODEX_HOME，並設定：
cli_auth_credentials_store = "ephemeral"
設定前先檢查現有內容，不可直接覆蓋別人的設定。登入憑證只能存在目前 Codex 程序記憶體中。

現在請先回報你讀到的「已完成、尚未完成、下一步」各一段，然後只執行只讀環境檢查，不要立刻安裝或修改。
