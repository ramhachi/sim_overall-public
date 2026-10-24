# run_and_export.m

## 概要
`run_and_export.m` は、`quarter_car_ode()` を既定設定で実行し、結果を `CSV` と画像へ保存し、標準出力へ要約を表示するスクリプトです。コメントでは「C++結果との比較用」とされており、出力表の列名と単位変換も比較しやすい形に整えています。

## 種別
スクリプトです。関数定義は含まず、先頭から順に処理されます。

## 実行フェーズ

### 1. 初期化とタイトル表示
`clear; clc; close all;` を実行したあと、区切り線と `MATLAB Quarter Car Simulation` を表示します。

### 2. シミュレーション実行
```matlab
[t, x, data] = quarter_car_ode();
```
を呼び出します。引数を渡していないため、`quarter_car_ode` 側の既定パラメータ、既定時間範囲、既定初期値、既定のステップ路面入力をそのまま使用します。処理時間は `tic` と `toc` で計測し、データ点数は `length(t)` で表示します。

### 3. `CSV` 出力
`table` を作成して `writetable` で保存します。列は次の 9 個です。
- `time`
- `z_s`
- `z_u`
- `dz_s`
- `dz_u`
- `F_spring`
- `F_damper`
- `F_tire`
- `z_road`

単位変換はコード上で明示されています。
- `x(:,1:4)` は 1000 倍してミリ系へ変換します。
- `data.F_spring`、`data.F_damper`、`data.F_tire` はそのままニュートンです。
- `data.z_road` は `quarter_car_ode` 側で既にミリメートル化された値をそのまま使います。

出力先は
```matlab
fullfile(fileparts(mfilename('fullpath')), '..', '..', 'output')
```
で求めます。ディレクトリが無い場合は `mkdir(output_dir)` で作成します。最終的な保存先は `output/quarter_car_matlab.csv` です。

### 4. 結果サマリー表示
標準出力へ以下を表示します。
- `t(end)` による総時間
- `max(x(:,1)) * 1000` による `z_s` 最大値
- `max(x(:,2)) * 1000` による `z_u` 最大値
- `x(end,1) * 1000` による最終 `z_s`
- `x(end,2) * 1000` による最終 `z_u`

最大値は絶対値ではなく単純な `max` です。

### 5. プロット作成
`figure('Position', [100, 100, 1200, 800])` で 2 行 2 列の図を作ります。

#### 左上: 変位
- `z_s`
- `z_u`
- `data.z_road`

を同じ軸へ描きます。単位はミリメートルです。

#### 右上: 速度
- `x(:,3) * 1000`
- `x(:,4) * 1000`

を描きます。単位はミリメートル毎秒です。

#### 左下: 力
- `data.F_spring`
- `data.F_damper`
- `data.F_tire`

をニュートンで描きます。

#### 右下: ダンパのヒステリシス散布図
```matlab
v_rel = (x(:,3) - x(:,4)) * 1000;
scatter(v_rel, data.F_damper, 3, t, 'filled');
```
で、相対速度とダンパ力の関係を時刻色付きで表示します。点サイズは `3` です。

図全体タイトルは `MATLAB Quarter Car Simulation (Ground Truth)` で、`output/quarter_car_matlab.png` に `saveas` で保存します。

## 外部依存
- `quarter_car_ode`
- `table`、`writetable`
- `figure`、`subplot`、`plot`、`scatter`、`colorbar`、`saveas`
- `fullfile`、`fileparts`、`mfilename`、`exist`、`mkdir`

## 生成物
- `output/quarter_car_matlab.csv`
- `output/quarter_car_matlab.png`

## 補足
このスクリプトにはローカル関数はありません。処理はすべてトップレベルのスクリプト本体に記述されています。
