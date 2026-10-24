# tire_fitting/main.py

## 役割
タイヤフィッティングの主 CLI です。実装上は TMeasy 単独ではなく、複数モデルを比較して最適な縦力/横力モデルを選ぶハイブリッド実行器になっています。

## 主な関数
- `load_config_file()`: YAML 設定読込と `model_long/model_lat` 妥当性確認
- `merge_config_with_args()`: CLI と設定ファイルの優先順位を統合
- `parse_arguments()`: 入出力、モデル選択、前処理条件の CLI 定義
- `main()`: 7 段階の実行（読込→前処理→複数モデル同定→選択→検証→比較レポート→YAML/C++ 出力）

## 背景・理論
縦方向と横方向で最適モデルが異なる前提を採り、RMSE/TIC/R² に基づくモデル選択を行います。TTC 実験値から TMeasy と簡易 Magic Formula を並列評価するのが本ファイルの核です。

## 使用ライブラリ
- `argparse`, `logging`, `yaml`
- `numpy`
- `tire_fitting` 配下のローダ・前処理・最適化・レポート生成モジュール

## 入出力
- 入力: DriveBrake/Cornering データ、設定 YAML、CLI 引数
- 出力: 比較レポート、ハイブリッド YAML、C++ ヘッダ、ログ

## パイプライン上の位置
TTC データからシミュレータ投入用タイヤパラメータを作る最上位オーケストレータです。
