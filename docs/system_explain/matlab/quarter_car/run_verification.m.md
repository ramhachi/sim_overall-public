# run_verification.m

## 概要
`run_verification.m` は、`quarter_car_ode` で `MATLAB` 側の基準応答を生成し、`../../output/quarter_car_cpp.csv` が存在すれば C++ 側結果を読み込んで比較し、図と `CSV` を保存する検証スクリプトです。

## 種別
スクリプトです。末尾に `default_params` というローカル関数を 1 つ持ちます。

## 実行フェーズ

### 1. 初期化
`clear; clc; close all;` を実行し、検証開始メッセージを表示します。

### 2. `MATLAB` 側 Ground Truth 生成
ローカル関数 `default_params()` でパラメータ構造体を作成し、直後に `params.use_bouc_wen = true;` を設定します。初期状態は
```matlab
x0 = [0; 0; 0; 0; 0];
```
で、状態の並びはコメントどおり `[z_s, z_u, dz_s, dz_u, z_bw]` です。

路面入力は
```matlab
z_step = 0.01;
input_func = @(t) z_step * (t >= 0.1);
```
なので、`t = 0.1` 秒で 10 ミリメートルの段差へ切り替わります。シミュレーション区間は `[0, 1]` 秒で、
```matlab
[t_matlab, x_matlab] = quarter_car_ode(params, t_span, x0, input_func);
```
を呼び出します。

### 3. C++ 結果の条件付き読み込み
比較対象ファイルは固定で `../../output/quarter_car_cpp.csv` です。`exist(..., 'file')` が真なら `readmatrix` で読み込み、
- 1 列目を `t_cpp`
- 2 列目以降を `x_cpp`

として扱います。

時刻軸を合わせるため、
```matlab
x_cpp_interp = interp1(t_cpp, x_cpp, t_matlab, 'linear', 'extrap');
```
で C++ 側を `t_matlab` 上へ線形補間します。補間後、
- `error_abs = abs(x_matlab - x_cpp_interp)`
- `error_rms = sqrt(mean(error_abs.^2, 1))`
- `error_max = max(error_abs, [], 1)`

を計算し、先頭 4 状態の誤差を表示します。ファイルが無い場合は「Run C++ simulation first.」を表示し、`has_cpp = false` にします。

### 4. プロット
`figure('Position', [100, 100, 1200, 800])` を作成し、4 分割で描画します。

#### 左上: ばね上変位
`x_matlab(:,1)` をミリメートル化して描きます。`has_cpp` が真なら `x_cpp(:,1)` も破線で重ねます。

#### 右上: ばね下変位
`x_matlab(:,2)` と、存在すれば `x_cpp(:,2)` を描きます。

#### 左下: ばね上速度
`x_matlab(:,3)` と、存在すれば `x_cpp(:,3)` を描きます。この軸はコード上でミリ変換していません。

#### 右下: 誤差表示または案内文
C++ 結果がある場合は
- `error_abs(:,1)`
- `error_abs(:,2)`
- `error_abs(:,3)`

を `semilogy` で描きます。`error_abs(:,4)` や 5 列目の履歴変数はここでは描きません。C++ 結果が無い場合は中央に案内文を表示します。

図全体タイトルは `Quarter Car Model Verification` で、`saveas(gcf, '../../output/verification_plot.png')` により保存されます。

### 5. Ground Truth 保存
```matlab
output_data = [t_matlab, x_matlab];
writematrix(output_data, '../../output/quarter_car_matlab.csv');
```
で時刻と状態をそのまま保存します。ここでは列名や単位変換は付けません。

### 6. 検証サマリー
C++ 結果がある場合は `pass_threshold = 1e-4` を使い、`error_rms(1:4)` の全要素が閾値未満なら `PASS`、それ以外は `FAIL` を表示します。C++ 結果が無い場合は `PENDING` を表示します。

## ローカル関数

### `default_params`
以下のフィールドを持つ構造体を返します。
- `m_s = 62.5`
- `m_u = 12`
- `k_s = 25000`
- `c_s = 1500`
- `k_t = 200000`
- `use_bouc_wen = true`
- `bw_alpha = 5000`
- `bw_A = 1`
- `bw_beta = 500`
- `bw_gamma = 100`
- `bw_n = 2`

## 外部依存
- `quarter_car_ode`
- `readmatrix`、`writematrix`
- `interp1`
- `figure`、`subplot`、`plot`、`semilogy`、`saveas`

## 注意点
- 出力先 `../../output` に対して、このスクリプト自身は `mkdir` を行いません。保存先ディレクトリの存在はコード上では前提です。
- このファイルの `default_params` が作る構造体は `c_s` や `bw_*` を持ちますが、`quarter_car_ode` 本体の既定パラメータ関数で使われている名前は `c0`、`alpha`、`A`、`beta`、`gamma`、`n` です。コード上ではパラメータ名の対応付け処理は入っていません。
