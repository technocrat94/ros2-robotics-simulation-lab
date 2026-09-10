# Demo Script: ROS 2 x Newton Bridge
# 日期：2026-09-12
# 目的：驗證 ROS 2 (Py 3.10) 與 Newton (Py 3.12) 程序隔離通訊與 Mimic 運動學


[以下為終端機執行指令]

# 1. 展前環境準備 (確保背景服務已開)
# Terminal 1: 啟動 ROS 環境
source /opt/ros/humble/setup.bash
source ~/ur5_ws/install/setup.bash

# Terminal 2: 監控 Bridge 狀態 (檢查是否連線且 age 近乎 0)
ros2 topic echo /newton/bridge_status --once


# 2. 現場動態展示 (看著 Viewer 執行)
# 2-1 狀態歸零
ros2 service call /newton/reset std_srvs/srv/Trigger

# 2-2 啟動展示軌跡 (約 8 秒)
ros2 service call /newton/set_running std_srvs/srv/SetBool "{data: true}"


# 3. 驗證資料回傳 (證明雙向介面；本展示不宣稱動力學閉迴路)
# 3-1 檢查 ROS 是否收到最新角度
ros2 topic echo /newton/joint_states --once

# 3-2 證明 5 個 mimic 關節角度完全符合 URDF 數學模型 (預期 0.0)
ros2 topic echo /newton/mimic_max_error --once

# 3-3 確認流程狀態
ros2 topic echo /newton/demo_phase --once
