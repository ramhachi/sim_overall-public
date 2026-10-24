# tests/test_pipeline.m

## 概要
- `test_pipeline.m` は、フロント A アーム最適化とステアリング最適化を順番に起動するスモークテスト用スクリプトです。
- 名前はテストですが、内部に `assert` は無く、主要関数がエラーなく呼び出せるかを確認する実行確認スクリプトに近い構成です。
- スクリプト実行時に MATLAB パスへ複数ディレクトリを追加します。

## 種別
- スクリプトです。
- ローカル関数はありません。

## 入力
- 直接の引数はありません。
- `optimized_autocross.json` の位置はスクリプト自身のディレクトリ基準で解決します。

## 出力
- 戻り値はありません。
- 標準出力へ進捗メッセージを表示します。
- 副作用として、最適化関数が `suspension_base_geometry.json` などを更新する可能性があります。

## スクリプトの実行フロー

### 1. 自身の位置の取得
- `script_dir = fileparts(mfilename('fullpath'));` でこのスクリプトの絶対位置を取得します。
- これを基準に相対パスを構築するため、呼び出し元のカレントディレクトリへ依存しにくい構成です。

### 2. MATLAB パスの追加
- `addpath(fullfile(script_dir, '..'));` で `suspension_kinematics_design` 本体を追加します。
- `addpath(fullfile(script_dir, '..', '..', 'matlab', 'steering_hardpoint_design'));` でステアリング最適化ディレクトリも追加します。
- パス重複確認や後始末はありません。

### 3. Layer 1 の起動
- `fprintf('Running Layer 1 (A-Arm)\n');` を表示します。
- `main_a_arm_optimizer(fullfile(script_dir, '..', '..', 'matlab', 'optimized_autocross.json'));` を呼びます。
- ここでフロント左 A アームの最適化が走ります。

### 4. Layer 1.5 の起動
- `fprintf('\nRunning Layer 1.5 (Steering)\n');` を表示します。
- 続けて `main_steering_optimizer(...)` を同じ JSON で呼びます。
- `main_steering_optimizer` はこのディレクトリ外の `matlab/steering_hardpoint_design` に依存します。

### 5. 完了表示
- 最後に `fprintf('\nPipeline Test Complete\n');` を表示して終了します。
- 個別出力の妥当性判定やファイル内容比較は行いません。

## スクリプト各セクションの責務
- パス設定: 依存関数の解決
- Layer 1 実行: A アーム最適化の起動
- Layer 1.5 実行: ステアリング最適化の起動
- 完了表示: 人間向けの終了通知

## 依存関係
- `main_a_arm_optimizer`
- `main_steering_optimizer`
- `addpath`, `fullfile`, `fileparts`, `mfilename`, `fprintf`
- 外部入力ファイル: `matlab/optimized_autocross.json`

## 前提・注意点
- `try/catch` が無く、どちらかの最適化が失敗するとその場で停止します。
- 生成物のバックアップや復元は行いません。
- 厳密な自動テストというより統合起動確認用です。
