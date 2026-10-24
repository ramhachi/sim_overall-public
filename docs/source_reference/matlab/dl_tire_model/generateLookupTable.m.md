# generateLookupTable.m

## 概要
学習済みネットワークを 5 次元格子上で評価し、C++ 用のルックアップテーブルを生成する関数です。軸は `SR`、`SA`、`Fz`、`IA`、`TSTC` で、各格子点に対する `Fx`、`Fy`、`Mx`、`Mz` を事前計算します。

## 主要関数・クラス
- `generateLookupTable(trained_net, norm_params, output_path, grid_config)`
- `ndgrid`
- `predictTireForces`
- `.mat` 保存

## 理論・背景
リアルタイム C++ シミュレーションでは、巨大な NN を毎ステップ直接実行するより、オフラインで計算した格子値を補間するほうが高速です。本関数はモデル精度と実行速度の妥協点として、事前計算テーブル化を行います。格子密度を上げるほど補間誤差は減りますが、メモリと生成時間は増加します。

## 外部ライブラリ・ツールボックス
- Deep Learning Toolbox
- MATLAB 標準配列・保存機能

## 入出力
- 入力:
  - 学習済みネットワーク
  - 正規化係数
  - 出力先
  - 格子設定構造体
- 出力: 軸配列、4 出力グリッド、メタデータを含む `lookup_table`

## システム上の役割
`exportForCpp` の計算中核です。MATLAB 内の高精度関数近似を、C++ 側で扱いやすい数表形式へ変換する実装上の要です。
