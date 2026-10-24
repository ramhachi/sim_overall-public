# generateAllVisualizations.m

## 概要
予測結果の主要図を一括生成する統合可視化関数です。散布図、残差図、時系列図、条件別性能図をまとめて PNG 出力します。

## 主要関数・クラス
- `generateAllVisualizations(X_features, Y_actual, Y_pred, output_dir, varargin)`
- 呼び出し先: `generateScatterPlots`、`generateResidualPlots`、`generateTimeSeriesPlots`、`generateOperatingConditionPlots`

## 理論・背景
単一図ではモデルの欠点を見落としやすいため、複数視点の可視化をまとめて生成する設計になっています。予測一致度、誤差分布、条件依存性を横断的に把握できます。

## 外部ライブラリ・ツールボックス
- MATLAB 標準可視化機能

## 入出力
- 入力: 特徴量、実測値、予測値、出力先ディレクトリ、追加オプション
- 出力: 複数の PNG 画像

## システム上の役割
評価結果の解釈層を担います。学習後の報告資料作成、ハイパーパラメータ比較、異常挙動の探索に不可欠です。
