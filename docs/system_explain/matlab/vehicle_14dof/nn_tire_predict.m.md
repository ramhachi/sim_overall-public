# nn_tire_predict.m

## 目的
`nn_tire_predict` は、ニューラルネットワークの入力前処理と出力後処理を担当するラッパー関数です。スリップ率、スリップ角、荷重、キャンバー角、表面温度を 5 要素ベクトルにまとめ、Z スコア正規化して `predict` に渡し、4 出力を逆正規化して返します。

## 入口とシグネチャ
```matlab
function [Fx, Fy, Mx, Mz] = nn_tire_predict(SR, SA, Fz, IA, TSTC, net, norm_params)
```

### 入力
- `SR`: スリップ率 `[-]`
- `SA`: スリップ角 `[deg]`
- `Fz`: 垂直荷重 `[N]`
- `IA`: キャンバー角 `[deg]`
- `TSTC`: タイヤ表面温度 `[°C]`
- `net`: 学習済み NN モデル
- `norm_params`: 正規化パラメータ構造体
  - `input_mean (1x5)`
  - `input_std (1x5)`
  - `output_mean (1x4)`
  - `output_std (1x4)`

### 出力
- `Fx`: 前後力 `[N]`
- `Fy`: 横力 `[N]`
- `Mx`: 転覆モーメント `[Nm]`
- `Mz`: セルフアライニングトルク `[Nm]`

## 外部依存
- `predict(net, X_norm)`

このファイル内にローカル関数はありません。

## 処理の流れ
1. 入力を `[SR, SA, Fz, IA, TSTC]` の順で `X` に並べます。
2. `norm_params.input_mean` と `norm_params.input_std` で
   ```matlab
   X_norm = (X - input_mean) ./ input_std
   ```
   を計算します。
3. `predict(net, X_norm)` を呼んで正規化済み出力 `Y_norm` を得ます。
4. `output_std` と `output_mean` を使って
   ```matlab
   Y = Y_norm .* output_std + output_mean
   ```
   で逆正規化します。
5. `Y(1:4)` を `Fx`、`Fy`、`Mx`、`Mz` に展開します。

## 注意点
- 入力や出力の次元検証は行っていません。
- `Mx`、`Mz` は返しますが、この関数自身は値の使い方を決めません。
- 例外処理やフォールバックはなく、`net` や `norm_params` が不正なら MATLAB 側のエラーになります。

## 保存物・描画
保存処理も描画処理もありません。
