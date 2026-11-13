# RTK + OpenVINS 最小接續節點（實驗版）

這是場地對齊 + VIO 相對運動接續，不是 EKF、不是完整多感測器機率融合，也不可直接視為自動駕駛安全定位。固定雜訊 baseline 未更動。

## 座標與量測契約

- `field_enu`：固定 WGS84 原點的 East/North/Up（m），右手系，yaw=0 向東、逆時針為正（rad）。原點在 YAML 明確設定，恢復 RTK 不會重設。高度必須使用一致的 datum；目前接收器 altitude 是橢球高還是海拔尚待確認，不能混用。
- `base_link`：車體參考點，x 前、y 左、z 上；實車需選定固定機械基準點。輸出 pose 是此點，不是天線位置。
- OpenVINS `/ov_msckf/odomimu` (`nav_msgs/Odometry`)：`global -> imu`。本機 ROS2Visualizer 說明 JPL global-to-IMU xyzw 隱含等價於 ROS Hamilton IMU-to-global；不要再反轉它。
- `imu_from_body_xyzw`：Hamilton 旋轉 R_IB，將車體向量轉到 IMU；`imu_to_body_in_imu_m`：IMU 原點到車體基準點的向量，以 IMU 軸表示。T_VB=T_VI*T_IB。
- `antenna_in_body_m`：車體基準點到 RTK 位置天線的桿臂，以車體軸表示。扣除 R_FB*lever 後才對齊車體位置。雙天線位置參考天線與基線方向需確認。
- 所有 header.stamp 必須是共同時基、量測時間，不能混用 receipt time、GPS time、system time。bag 一起錄 `/clock` 所需的原始時間；播放使用 `--clock`，節點 `use_sim_time:=true`。真機改 false。附件 stamp 約 1041 s，不能直接與另一次 D435i 錄製的 Unix epoch 相配。
- VIO buffer 以位置線性插值、姿態 SLERP 配合 GNSS 時間，不外插；超過 sync_gap 拒用。輸出 timestamp 是最新 VIO 量測時間；RTK 不會被偽裝為相機頻率的獨立量測。

## 輸入與品質

| Topic | Type | 意義 |
|---|---|---|
| `/ov_msckf/odomimu` | nav_msgs/Odometry | VIO 位姿 |
| `/gps/fix` | sensor_msgs/NavSatFix | 天線位置；status>=0 只是必要條件 |
| `/gps/diagnostics` | diagnostic_msgs/DiagnosticArray | name=`RTK GPS`，同時要求 `Fix Quality=4`、`Status=RTK_FIXED` |
| `/rtk/heading_enu` | geometry_msgs/QuaternionStamped | 已轉成車頭 ENU 航向的四元數，frame_id=field_enu |
| `/rtk/heading_diagnostics` | diagnostic_msgs/DiagnosticArray | name=`Dual antenna heading`、key=`Heading Valid`、value=`true` |

所有輸入可用參數改 topic；採 sensor-data QoS。兩種 diagnostics 均須 level=OK。附件實際包含固定解4、浮點解5；不使用 NavSatFix.status=2 區分固定/浮點。附件 covariance 有水平0.0004、垂直0.0016 m²、type=1，但未確認是否接收器真實估計或驅動常數；本版不據此加權。品質取量測時間之前最近一筆，不拿未來品質追認舊位置。航向使用量測之前最近的有效樣本（最多0.3秒），沒有角速度外推；高速轉彎時此時間差仍可能影響對齊，正式 adapter 應提供與位置同步的 heading。

未來雙天線驅動需 adapter：若提供 heading 為正北0、順時針正，先換 `yaw_ENU=pi/2-heading`，再補償天線基線與車頭夾角；有效性必須讀接收器的 heading 狀態，不沿用位置固定解狀態。第一版不猜測尚未存在的雙天線欄位。

沒有雙天線時可手動指定 `field_from_vio_yaw_deg`（VIO 局部座標到場地的旋轉角，不是車頭角度）。必須知道該次 VIO 局部座標方向；VIO 重啟後不能假設相同。沒有已知對齊角或有效 heading 時，不輸出場地定位。定位過程暫時失去 heading 但 RTK 位置仍有效，可保留上次 yaw 對齊、更新平移；status 會分開報 heading_available。

注意：附件 diagnostics 約1Hz，代表品質轉差可能晚於某些 fix 才被接收。第一版採有期限的 sample-and-hold，無法消除此發布延遲；正式接收器 adapter 最好把固定解旗標與該筆位置時間綁定。timestamp 僅允許最多50ms 的 ROS callback/clock 排程差；這不是感測器時間同步校正。

## 狀態與恢復

`WAIT_ALIGNMENT -> RTK_ANCHORED -> VIO_CONTINUATION -> RECOVERING -> RTK_ANCHORED`。

此最小版本需要有效 VIO 來處理車體姿態、桿臂與接續；VIO 未初始化或失效時不輸出 pose，即使 RTK 有效也沒有另行提供 RTK-only 備援。RTK_ANCHORED 表示以 RTK 對齊、由 VIO 推進至最新時刻，不是純 RTK 位姿。

RTK 正常時每筆可信 fix 更新 yaw+3D translation（roll/pitch 仍來自 VIO，假設 VIO 重力對齊）。RTK 中斷或浮點解時凍結最後對齊，後續場地運動就是旋轉後的 VIO 相對位移。RECOVERING 仍使用舊對齊。

恢復需連續5筆、至少1秒，且相對舊對齊差距<=2m、有效航向差<=20deg。這些是可調測試門檻，不是實車調校結果。接回為硬切換，沒有隱藏平滑；每筆 raw residual 在更新前發出。沒有 heading 時 yaw residual=null，不能說差值為0。過大差值維持 VIO，不自動放寬。可人工確認後呼叫 reset_vio 撤銷全部對齊並重新取得（屬明確重新定位，不是連續恢復）。

VIO 超時不輸出新 pose。相鄰 pose 跳變>2m/45deg、時間倒退或間隔>0.5s 會撤銷對齊並換 epoch，也作廢該輪人工對齊角；須新有效 heading 或重新設定對齊角並重啟 bridge。這是保守 heuristic，無法保證偵測所有內部 reset；啟動/重啟 OpenVINS 必須呼叫 reset_vio。無 RTK 時發生 reset 則停止定位，不用未知新原點接續。場地原點保持不變。時效均依 ROS clock，bag 暫停時模擬時間也暫停，不是車體控制用的 wall-clock 安全 watchdog。

## 輸出

- `/rtk_vio_bridge/pose`：geometry_msgs/PoseStamped，field_enu 下 base_link 位姿；刻意不捏造融合 covariance 或速度，不發布 TF 避免與現有 global->imu 衝突。
- `/rtk_vio_bridge/source`：std_msgs/String 狀態。
- `/rtk_vio_bridge/status`：std_msgs/String JSON，source、available、epoch、ages_s、rtk_fixed_available、heading_available、yaw_alignment_source、raw residual。用10Hz timer 報超時，即使沒有新輸入也會更新。available 只表示本演算法有有效輸出，不代表符合自主行駛安全門檻。
- `/rtk_vio_bridge/events`：JSON，原始 ENU/yaw 差值及 reset/acquired/recovered 事件；需 rosbag record 保存。正號為 RTK 車體位置減舊對齊預測，yaw 為 RTK 減預測。
- `/rtk_vio_bridge/reset_vio`：std_srvs/Trigger。重啟 VIO、bag loop、時鐘改變須重新對齊；不連續沿用舊變換。

## 建置與真機設定

```bash
source /opt/ros/humble/setup.bash
cd /home/tucker/openVINS_ws
colcon build --packages-select rtk_vio_bridge --symlink-install
source /home/tucker/openVINS_ws/install/setup.bash
```

先編輯 `src/rtk_vio_bridge/config/mower.yaml` 的場地原點、車體外參、桿臂，再將兩個 configured flag 設 true。預設拒絕以未校正的 placeholder 啟動；不要把零值當成已量測的車體外參。

```bash
ros2 run rtk_vio_bridge bridge --ros-args --params-file /home/tucker/openVINS_ws/src/rtk_vio_bridge/config/mower.yaml
ros2 service call /rtk_vio_bridge/reset_vio std_srvs/srv/Trigger '{}'
```

上面使用模擬時間播放 bag；真機要在 bridge 指令追加 `-p use_sim_time:=false`，並確認 RTK 與 D435i stamp 的共同時基。

## 合成 RTK 資料流測試

以下只供測試：假設 body=IMU、零桿臂、已知 yaw=.4rad、平移[10,20,0]m，原點25N/121E/30m。**不是實車配置**。

```bash
ros2 run rtk_vio_bridge bridge --ros-args -p use_sim_time:=true \
  -p origin_configured:=true -p extrinsics_configured:=true \
  -p origin_lat:=25.0 -p origin_lon:=121.0 -p origin_alt:=30.0 \
  -p manual_alignment_enabled:=true -p field_from_vio_yaw_deg:=22.918311805 \
  -p fix_topic:=/test/gps/fix -p quality_topic:=/test/gps/diagnostics \
  -p heading_topic:=/test/heading -p heading_quality_topic:=/test/heading_diagnostics
ros2 run rtk_vio_bridge synthetic_rtk --ros-args -p use_sim_time:=true
ros2 launch ov_msckf subscribe.launch.py \
  config_path:=/home/tucker/openVINS_ws/src/open_vins/config/rs_d435i_user/estimator_config_mower_dyn.yaml \
  rviz_enable:=false
ros2 bag play /home/tucker/rosbag2/d435i_vio_test_longdist --clock \
  --topics /camera/camera/imu /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw
```

各指令用不同終端、相同 ROS domain/RMW。合成器只發 `/test/*`，不污染真實 `/gps/*`。每40秒循環：0–10固定解、10–15無資料、15–20浮點解、20–40固定解並加0.4m恢復差。起點為第一筆 VIO，非 bag 起點。單元測試涵蓋已知旋轉平移、桿臂、插值、逾時、恢復、超門檻、重啟、無航向、重複/未來/過期 fix：

```bash
PYTHONPATH=/home/tucker/openVINS_ws/src/rtk_vio_bridge python3 -m unittest discover -s /home/tucker/openVINS_ws/src/rtk_vio_bridge/test -v
ros2 bag record -o /home/tucker/rosbag2/rtk_vio_logic_test \
  /ov_msckf/odomimu /test/gps/fix /test/gps/diagnostics /test/heading /test/heading_diagnostics \
  /rtk_vio_bridge/pose /rtk_vio_bridge/status /rtk_vio_bridge/events /clock
```

合成 RTK 與 VIO 同源，只能證明變換/狀態機/資料流，不能量實車精度、VIO 漂移或 RTK 接手性能。正式同步資料需保留原始 RTK、heading、各自品質、D435i、OpenVINS、事件與時鐘；之後再評估降級融合、拒絕門檻及相關量測問題。不要把此處連續 pose 當成獨立絕對量測送外部 EKF。
# rtk_vio_bridge
