# `matlab/visualize_debug.m`

## 1. ファイルの目的

ASE autocross 検証 CSV とトラック CSV を重ね描きし、デバッグ用軌跡画像を非表示で生成するスクリプトです。

## 2. 主な入出力または依存関係

- 入力:
  - `../laptime_sim_ASE_autocross_verification.csv`
  - `../data/ASE_autocross_sim.csv`
- 出力: `debug_traj.png`
- 依存関係: `readtable`, `scatter`, `saveas`, `exit`

## 3. 処理の流れ / アルゴリズム

1. シミュレーション結果とトラック点列を読み込みます。
2. 非表示 figure を作成し、トラック点列と車両軌跡を重ねます。
3. 開始点・終了点・トラック開始点を目印付きで描きます。
4. `debug_traj.png` を保存して MATLAB を終了します。

## 4. ファイル内の定義

### スクリプト本体

- 軌跡は速度で色分けした scatter、トラックは黒点線で表示します。
- `figure('Visible','off')` と `exit` を使うため、バッチ診断向けのスクリプトです。

## 5. 注意点・前提条件

- 実行すると最後に `exit` するため、対話 MATLAB セッション内で走らせると終了します。
- 入力パスはこのスクリプト位置からの相対参照です。
