# Git 與 GitHub：這個專案做了什麼

## 先用生活比喻理解

可以把這個專案想成一本正在撰寫的專題書：

| Git/GitHub 名稱 | 生活比喻 | 本專案中的意思 |
|---|---|---|
| Repository | 整本專題資料夾 | `ros2-robotics-simulation-lab`，包含程式與學習文件 |
| Git | 自動保存修訂歷史的系統 | 記住每次正式修改前後的差異 |
| GitHub | 放在網路上的私人作品櫃 | 保存備份，未來可展示或協作 |
| `main` branch | 目前正式版本的主幹 | 日後穩定更新都接在這條線上 |
| commit | 一個有名稱的存檔點 | 可查看當時改了哪些檔，也能找到舊版本 |
| tag | 貼在重要存檔上的里程碑標籤 | `v0.1.0` 指向第一個已驗證的穩定版本 |
| `origin` | 本機替遠端地址取的綽號 | 指向 GitHub 的這個 Repository |
| push | 把本機的新存檔送到 GitHub | 讓 UTM 與 GitHub 同步 |
| Deploy Key | 只開這個作品櫃的專用鑰匙 | 讓 UTM 只能讀寫這個 Repository |

Git 和 GitHub 並不是同一個東西。Git 是 UTM 裡負責版本歷史的工具；GitHub 是存放遠端副本的網站。即使沒有網路，Git 仍能建立 commit；有網路後再用 `push` 同步到 GitHub。

## 我們實際完成的 GitHub 工作

### 1. 清理應該上傳的內容

專案原本包含多份 `.old`、`.before_*` 備份，以及 workspace 可能產生的 `build`、`install`、`log`。這些檔案適合留在開發電腦，但不適合放進正式作品集。

因此建立 `.gitignore`：它像投稿前的篩選清單，告訴 Git「這些草稿和自動產物不要裝訂進正式報告」。正式原始碼沒有被刪除。

### 2. 建立本機 Git Repository

在以下位置初始化 Git：

```text
~/ur5_ws/src/ur5_moveit_demo
```

主分支名稱設定為 `main`。

### 3. 建立第一個穩定版本

第一筆 commit：

```text
d01ff7b  Initial stable UR5 and Robotiq MoveIt demo
```

它保存已成功執行的程式、模型與設定。之後建立：

```text
v0.1.0
```

這個 tag 永遠指向第一個穩定里程碑。未來即使 `main` 繼續加入 Newton Physics，仍能回頭查看當時的 UR5＋Robotiq 版本。

### 4. 建立作品集文件版本

第二筆 commit：

```text
9332c4b  Document learning journey and operating workflow
```

它加入：

- 完整學習歷程
- 每日操作與參數修改說明書
- Troubleshooting／除錯紀錄
- README 的作品集入口

把文件獨立做成第二筆 commit，可以清楚區分「程式達成穩定」與「把經驗整理成可閱讀作品集」兩個階段。

### 5. 建立私人 GitHub Repository

遠端位置為：

```text
technocrat94/ros2-robotics-simulation-lab
```

目前是 Private，只有帳號擁有者與日後明確授權的人能看到。面試前若準備改為 Public，應先再檢查是否包含 IP、帳密、私人資訊或不適合公開的第三方內容。

### 6. 建立 UTM 專用 Deploy Key

GitHub 端的 Key 名稱：

```text
UTM Ubuntu - ros2 robotics simulation lab
```

它具有 Read/write 權限，但只限這一個 Repository。這比把能存取整個 GitHub 帳號的通用金鑰放進虛擬機更小範圍。

私鑰只留在 UTM：

```text
~/.ssh/ros2_robotics_simulation_lab_ed25519
```

不要把私鑰複製進 Repository、聊天訊息或履歷。GitHub 只保存對應的 public key。

### 7. 連接並推送

本機 remote `origin` 指向：

```text
git@github.com:technocrat94/ros2-robotics-simulation-lab.git
```

已推送：

- `main` branch
- 兩筆 commit
- `v0.1.0` tag
- 程式、設定、README 與 `docs/`

## GitHub 首頁怎麼看

進入 Repository 後：

1. 上方 `Private`：代表目前不是公開作品。
2. `main`：目前正在看的主分支。
3. `1 tag`：可以找到 `v0.1.0` 穩定版本。
4. 檔案列表：每個資料夾右側會顯示最後修改它的 commit。
5. `README`：GitHub 會自動把首頁說明顯示在檔案列表下方。
6. `History`／Commits：查看每個存檔點及差異。

## 目前資料夾分類

```text
ros2-robotics-simulation-lab/
├── config/       控制器、MoveIt、IK、joint limits 等設定
├── docs/         學習歷程、操作手冊、GitHub 說明與除錯紀錄
├── launch/       系統與任務的啟動檔
├── rviz/         RViz 顯示設定
├── src/          C++ 任務程式
├── srdf/         MoveIt 語意模型
├── urdf/         UR5＋Robotiq 機器人結構模型
├── README.md     英文作品集首頁
└── README.zh-TW.md  繁體中文首頁
```

目前 Repository 本身就是一個 ROS package。未來若 Newton Physics 的內容不適合放進同一個 ROS package，我們會先規劃結構再搬移，而不是直接把不同環境的檔案混在一起。

## 以後修改程式的正常流程

完成修改並測試後：

```bash
cd ~/ur5_ws/src/ur5_moveit_demo

git status
git diff
git add .
git commit -m "用一句話說明這次完成什麼"
git push
```

每一行的意義：

```text
git status  → 看哪些檔案改過
git diff    → 檢查實際修改內容
git add     → 選擇要放進下一個存檔點的內容
git commit  → 在本機建立具名存檔點
git push    → 把新存檔同步到 GitHub
```

原則是一個 commit 對應一個清楚成果，例如「加入桌面碰撞物件」或「加入 Newton Physics 初始場景」，不要把互不相關的修改塞進同一筆 commit。這樣面試官能像閱讀實驗紀錄一樣，看懂能力如何逐步累積。

## 為什麼這對求職有幫助

只有最後程式碼，只能證明「現在有一份成品」。清楚的 commit、tag、學習歷程與 troubleshooting，則能額外證明：

- 能把大問題拆成可驗證的小步驟
- 知道如何保存穩定版本
- 能說明失敗原因與技術取捨
- 能寫出別人可重現的操作文件
- 對模擬與實體硬體的界線有安全意識

這些內容正是面試時可以拿來說明的工程歷程。
