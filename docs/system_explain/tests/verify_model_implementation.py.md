# tests/verify_model_implementation.py

## 役割
TMeasy 実装の基本応答、感度、力曲線、実データとのズレをまとめて可視化し、モデル実装とパラメータ妥当性を切り分ける検証スクリプトです。

## 入力と出力
- 入力: 固定パラメータ集合、`data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
- 出力: 各ケースの PASS/FAIL、RMSE/MAE/最大誤差、警告ログ
- 生成物: `output/model_verification_characteristics.png`, `output/model_verification_actual_data.png`

## 主な処理
- `test_basic_functionality()` はゼロスリップ、線形域、飽和域の 3 ケースで縦力の符号と大きさを確認します。
- `test_parameter_sensitivity()` は `mu_x_peak`、`Kx`、`k_mu_x` を振り、力が増減すべき方向へ変化するかを見ます。
- `test_force_characteristics()` は 3 つのパラメータセットで `Fx-SR` 曲線と荷重依存曲線を描きます。
- `test_actual_data_comparison()` は実データを簡易フィルタ後、v3 パラメータで予測して RMSE などを出し、予測レンジ不足を警告します。

## プロットと存在理由
- 理論応答の図と実データ比較図を両方保存し、「実装は正しいが係数が悪い」状態を見抜けるようにしています。
- 最適化不良をモデル不具合と混同しないための検証用スクリプトです。
