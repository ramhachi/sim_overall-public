# exportNormalizationParams.m

## 概要
学習時に用いた入力・出力の平均値と標準偏差をテキスト形式で保存する関数です。MATLAB と外部実装の数値整合性を維持します。

## 主要関数・クラス
- `exportNormalizationParams(norm_params, output_path)`

## 理論・背景
正規化係数が一致しないと、同じ重みでも全く異なる予測になります。そのため、モデル本体と同等に重要な補助情報として明示的に外部保存する必要があります。

## 外部ライブラリ・ツールボックス
- MATLAB 標準 I/O

## 入出力
- 入力: `norm_params`、保存先パス
- 出力: 正規化係数テキスト

## システム上の役割
`exportForCpp`、`exportModelForCpp`、推論再現の共通補助ファイルを作る役割です。
