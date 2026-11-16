# 第一版驗證紀錄 — 2026-11-13

## 結果

- `colcon build --packages-select rtk_vio_bridge --symlink-install`：成功。
- `colcon test --packages-select rtk_vio_bridge`：20 passed；test-result：0 errors / 0 failures / 0 skipped。
- 真實 longdist bag 前約50秒 + 本次 OpenVINS 輸出 + 從該輸出產生的合成 RTK：通過。
- 最終回放輸出7035筆場地位姿；首次對齊後沒有 UNAVAILABLE 閃爍。
- 依序觀察到 UNAVAILABLE（等待 OpenVINS 初始化）、WAIT_ALIGNMENT、RTK_ANCHORED、VIO_CONTINUATION、RECOVERING、RTK_ANCHORED。
- 故意在恢復 RTK 加東向0.4m，接回前記錄 delta_ENU=[0.3999999983, 4.89e-10, -1.12e-9]m。
- 同次 yaw residual=-0.00306773rad；合成 heading 與 fix 分 topic、以最近過去樣本匹配，可能留下小幅時間差。不可將此數字解讀為實車航向精度。

## 證據與重跑

最終完整日誌：`test_results/longdist_verified/`，包含 summary.json、status.jsonl、events.jsonl、bridge.log、synthetic.log、openvins.log、bag.log。

```bash
source /home/tucker/openVINS_ws/install/setup.bash
python3 /home/tucker/openVINS_ws/src/rtk_vio_bridge/test/run_bag_smoke.py \
  --seconds 50 --output /home/tucker/openVINS_ws/src/rtk_vio_bridge/test_results/new_run
```

output 必須為尚未存在的目錄。程式自動使用 ROS_DOMAIN_ID=137、rmw_cyclonedds_cpp、localhost，啟動自己擁有的 OpenVINS/bridge/合成器/bag player，結束只停止這些程序。未修改使用者既有 RMW 環境、baseline 或 OpenVINS 配置。

早期測試目錄保留供追查，不是最終結果：01 套件註冊失敗（maintainer email 格式已修）；02 合成器 DiagnosticStatus.level 型別錯誤（Humble 使用 byte，已修）；03 / final 是修正時鐘處理過程中的中間版。以 longdist_verified 為準。

## 覆蓋範圍

14個核心測試：旋轉/ENU、已知座標對齊、VIO 接續與逾時、RTK 恢復原始差值、真正逾時後恢復、位置與航向恢復門檻、VIO timestamp/原點重啟、重啟後新 heading 重新對齊、桿臂與插值、沒有 heading、過期/未來/重複 fix、非法資料、clock callback 排程差。

6個 adapter 測試：固定解/浮點解、ERROR品質拒收、過期/未來品質、禁止使用未來品質、禁止用新有效旗標追認舊 heading、略早於 /clock 的 fix 保留等待。

## 尚未驗證／不可宣稱

- 沒有同時錄製的真實 RTK + D435i，因此沒有實車定位誤差、真實中斷接手時間、真正恢復跳變或降級融合效果。
- 此次只跑 longdist 前段，沒有重跑整段約636秒；使用者原本整段通過的 OpenVINS baseline 仍獨立保留。
- 真實雙天線 adapter、位置/heading 各自有效性與精度欄位、天線基線方向尚未確認。
- 場地原點、車體基準點、IMU到車體外參、天線桿臂尚未由使用者提供；mower.yaml 保留未配置旗標，不能直接當作校正完成的真機設定。
- VIO 重啟檢測為保守 heuristic 加 explicit reset service，不能保證從 odometry 本身辨識所有內部 reset。
- 未實作 EKF、浮點解加權、時間延遲估計、外參線上標定、連續恢復平滑或車體安全控制。
- VIO 無效時停止輸出；尚無 RTK-only 備援。
