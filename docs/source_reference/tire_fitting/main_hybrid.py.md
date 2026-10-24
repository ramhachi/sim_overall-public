# tire_fitting/main_hybrid.py

## 役割
`main.py` と同系統のハイブリッド CLI です。DriveBrake/Cornering を読んで複数モデルを比較し、方向別に最適モデルを採用します。

## 主な関数
- `load_config_file()`
- `merge_config_with_args()`
- `parse_arguments()`
- `main()`

## 特徴
- 前処理は欠損除去と `FZ > 0` を中心にした保守的なフィルタ
- `ModelFitter` を使って `tmeasy` と `magic_formula` を必要分だけ実行
- 自動/手動のモデル選択結果を YAML/C++ へ出力

## 背景
複数モデル比較の明示的なエントリポイントとして分離されており、混成タイヤモデル開発の実験 CLI という位置づけです。

## パイプライン上の位置
ハイブリッド同定系の専用 CLI です。
