# `matlab/dl_tire_model/augment_tire_data_for_nn.m`

## 1. 概要

`augment_tire_data_for_nn.m` は、DriveBrake と Cornering の TTC CSV を読み込み、学習に使う列だけを抜き出したうえで、異常値除去・ゼロスリップ点追加・対称点生成を行い、拡張済みデータセットを CSV/MAT の両方で保存する MATLAB スクリプトです。

## 2. ファイルの役割

- `../../data/tiredata/` 配下の 2 つの固定 CSV を読み込みます。
- `SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz` の 9 列にそろえます。
- 学習に不向きな行を閾値ベースで除外します。
- ゼロスリップ点と対称点を追加して学習データの偏りを減らします。
- 拡張後データを `augmented_tire_data.csv` と `augmented_tire_data.mat` に保存します。

## 3. インターフェース

### 3.1 種別

スクリプトです。引数や戻り値はありません。

### 3.2 主な入力

- `../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
- `../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`

### 3.3 主な生成物

- ワークスペース変数
  - `db_data`, `corn_data`: 生データテーブル
  - `filtered_data`: 閾値で絞り込んだ元データ
  - `zero_slip_table`: 追加したゼロスリップ点
  - `symmetric_data`: 元データから反転生成した対称データ
  - `augmented_data`: 最終出力テーブル
- 保存ファイル
  - `../../data/tiredata/augmented_tire_data.csv`
  - `../../data/tiredata/augmented_tire_data.mat`

## 4. 依存関係

- MATLAB 標準関数: `detectImportOptions`, `readtable`, `array2table`, `randperm`, `writetable`, `save`, `mean`, `std`, `min`, `max`
- 外部依存はなく、このファイル内で全処理を完結させています。

## 5. 処理の流れ

1. DriveBrake と Cornering の CSV を別々の `detectImportOptions` で読み込みます。`DataLines` と `VariableNames` を個別指定して列名を固定化しています。
2. 両データから学習用の 9 列だけを抜き出し、`FZ` は符号を反転して `Fz` として保存します。
3. `Fz > 50 & Fz < 3000`、`|Fx| < 5000`、`|Fy| < 5000`、`|SR| < 0.5`、`|SA| < 20` の条件で不正値・外れ値を除外します。
4. `Fz_values × TSTC_values × IA_values` の全組合せに対し、`SR=0, SA=0` の行を生成します。`Fy, Mx, Mz` は 0 ですが、`Fx` は `-0.01 * Fz` の転がり抵抗相当値です。
5. `filtered_data` を複製して、`SR/SA/IA/Fx/Fy/Mx/Mz` を符号反転した `symmetric_data` を作ります。`Fz` と `TSTC` は維持されます。
6. 元データ・対称データ・ゼロスリップ点を結合し、`rng(42)` と `randperm` でシャッフルします。
7. 拡張後データの平均・標準偏差・レンジを表示し、`mean(Fx)` と `mean(Fy)` が 50 未満かでバランス状態をメッセージ出力します。
8. CSV と MAT に保存して終了します。

## 6. 関数・ローカル関数詳細

このファイルにローカル関数はありません。

### 6.1 Step 1: 元データの読み込み

- DriveBrake は 4 行目以降をデータ行として読み込みます。
- Cornering は 3 行目以降をデータ行として読み込みます。
- どちらも `VariableNames` を明示しており、CSV の列名に依存しない実装です。

### 6.2 Step 2: 必要列抽出

- 両データを同じ列構成のテーブルに変換します。
- `FZ` は「負値で記録されている」という前提で `-db_data.FZ`, `-corn_data.FZ` に変換します。

### 6.3 Step 3: 不正データ除去

- 閾値はコード中に固定値で直書きされています。
- `valid_idx` を段階的に AND 更新して最終フィルタを構築しています。

### 6.4 Step 4: ゼロスリップ点追加

- `Fz` は 500 から 2000、`TSTC` は 30 から 80、`IA` は -3 から 3 の離散値です。
- 生成点数は `7 × 6 × 7 = 294` 点です。
- コメントは「Fx=Fy=0」と述べていますが、実装上の `Fx` は転がり抵抗を模した負値です。

### 6.5 Step 5: 対称点生成

- 対称則として、コードは `Fy(SA,IA) = -Fy(-SA,-IA)` と `Fx(SR) = -Fx(-SR)` に相当する変換をテーブル全体へ適用しています。
- `Mx` と `Mz` も符号反転しています。

### 6.6 Step 6〜8: 統合・統計・保存

- 統合後にシャッフルされるため、元データの並び順は保持されません。
- `writetable` と `save` の両方を行うため、後続処理は CSV/MAT のどちらも利用できます。

## 7. 注意点

- 入力ファイル名・出力先はすべて固定パスです。
- `Fz` は読み込み時に正値化されますが、元 CSV が別形式の場合は前提が崩れます。
- ゼロスリップ点の `Fx` は完全な 0 ではなく `-0.01 * Fz` です。コメントだけ見て解釈すると実装とずれます。
- 末尾で次手順として `train_nn_with_augmented_data.m` を表示しますが、このファイル自身はその呼び出しを行いません。
