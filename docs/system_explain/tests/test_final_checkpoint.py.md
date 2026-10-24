# tests/test_final_checkpoint.py

## 役割
Task 16 の最終チェックポイントとして、既存テスト実行、既存 YAML を使った実データ照合、品質閾値確認を順に行う CLI スクリプトです。

## 入力と出力
- 入力: `output/tire_params/hoosier_18x6_r20.yaml`、DriveBrake / Cornering 実データ、既存 pytest スイート
- 出力: 標準出力の進捗ログ、サンプル予測表、品質判定
- 生成物: なし（既存 YAML とテスト結果を読む側）

## 主な処理
- `run_test_suite()` は `tests/unit/`、`tests/property/`、主要統合テスト 4 本を `subprocess.run()` で順に叩き、失敗時はその場で終了します。
- `verify_data_samples()` は既存 YAML から TMeasy パラメータを読み、DriveBrake / Cornering からランダム 10 点ずつ抜いて予測誤差を表示します。
- `check_quality_thresholds()` は `rmse < 100`, `tic < 0.15`, `r2 > 0.95` を FX/FY 別に確認します。
- `main()` は 3 段を通した要約を出し、例外時は traceback 付きで 1 を返します。

## 存在理由と注意点
- 新規アーティファクト生成よりも、既に作られた成果物と既存テスト群が最終品質を満たすかを一括で確認するためのスクリプトです。
- YAML や実データが無い場合は該当段階をスキップして先へ進みます。
