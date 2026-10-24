# `matlab/quarter_car/run_verification.m`

## 概要

`run_verification.m` は quarter-car モデルの MATLAB 結果と C++ 結果を比較する検証スクリプトである。  
まず MATLAB 側で 10 mm ステップ入力の ground truth を生成し、その後 `quarter_car_cpp.csv` が存在すれば時間整列・誤差計算・比較図作成を行う。

## 主要関数・クラス

- `quarter_car_ode(params, t_span, x0, input_func)`  
  MATLAB 基準波形の生成。
- `interp1(...)`  
  MATLAB / C++ の時間軸差を吸収する線形補間。
- 誤差評価部  
  `z_s`, `z_u`, `dz_s`, `dz_u` に対して RMS 誤差と最大誤差を算出する。
- 可視化部  
  4 パネルで変位・速度・誤差を表示する。

## 理論・背景

異なる実装間比較では、同一入力条件・同一状態定義・同一時間軸への再配置が重要になる。  
このスクリプトはそのための検証手順を定型化し、数値誤差を RMS / Max の両方で評価する。

また、単に誤差総量を見るだけでなく、時刻歴上で誤差がいつ発生するかをプロットするため、初期条件差・イベント直後差・減衰特性差を切り分けやすい。  
1e-4 の閾値で pass/fail を表示するため、継続的な整合監視にも使える。

## 外部ライブラリ・ツールボックス

外部ツールボックスは不要。  
標準の `readmatrix`、`interp1`、描画機能のみを使う。

## 入出力

- 入力
  - MATLAB 側条件: 1 秒解析、`t=0.1 s` で 10 mm 段差
  - C++ 側入力ファイル: `../../output/quarter_car_cpp.csv`
- 出力
  - 比較図: `../../output/verification_plot.png`
  - MATLAB ground truth: `../../output/quarter_car_matlab.csv`
  - コンソール上の RMS / Max 誤差と判定結果

## システム上の役割

MATLAB ↔ C++ の基礎整合性を保証するための quarter-car 検証フレームワークである。  
上位自由度モデルへ進む前に、サスペンション縦系の差異を局所化して確認する品質ゲートとして機能する。
