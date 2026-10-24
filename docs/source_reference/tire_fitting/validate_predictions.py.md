# tire_fitting/validate_predictions.py

## 役割
学習に使った TTC データからランダムサンプルを抜き出し、TMeasy 予測と実測を散布図・誤差図で照合する検証スクリプトです。

## 主な関数
- `load_fitted_parameters()`
- `sample_random_points()`
- `validate_longitudinal()`
- `validate_lateral()`
- `main()`

## 背景
集約指標だけでは外れ点の傾向が見えないため、サンプルベースの視覚検証を行います。縦力は `SR`、横力は `SA` と `FZ` 依存を観察できる構成です。

## 使用ライブラリ
- `numpy`, `pandas`, `matplotlib`, `yaml`

## 入出力
- 入力: フィット済み YAML、DriveBrake/Cornering データ
- 出力: `validation_longitudinal.png`, `validation_lateral.png`

## パイプライン上の位置
同定後の人手レビュー補助スクリプトです。
