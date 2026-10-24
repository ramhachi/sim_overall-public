# test_open_loop_driver.cpp

- **目的**: オープンループ入力生成器を 14DOF モデルへ与え、典型マニューバを CSV 出力します。
- **主要要素**: `test_step_steer()`、`test_sine_steer()`、`test_double_lane_change()` などと `main()`。
- **理論背景**: ステップ、サイン、ダブルレーンチェンジ等の事前定義入力で応答を見る検証です。
- **依存**: `open_loop_driver.hpp`、`vehicle_14dof_new.hpp`、`fstream`。
- **システム上の位置づけ**: 制御器を使わず車両本体を刺激する入力パターン検証用です。

