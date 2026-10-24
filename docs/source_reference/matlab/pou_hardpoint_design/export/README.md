# `matlab/pou_hardpoint_design/export/`

## 概要
POU設計結果を外部ツールへ受け渡すためのエクスポート系モジュールです。CAD用CSV、C++シミュレータ用JSON、設計レビュー向けレポート、既存車両パラメータJSONの更新を担当します。

## 主要関数・クラス
- [`export_hardpoints_csv.m.md`](export_hardpoints_csv.m.md)
- [`export_hardpoints_json.m.md`](export_hardpoints_json.m.md)
- [`export_summary_report.m.md`](export_summary_report.m.md)
- [`update_vehicle_params_json.m.md`](update_vehicle_params_json.m.md)

## 理論・背景
最適化結果は数値ベクトルのままでは設計・実装へ接続しにくいため、利用先に応じた形式変換が必要です。POU機構ではCADでの寸法検討とC++での挙動再現の両方が重要であり、出力整形は設計ループの一部です。

## 外部ライブラリ・ツールボックス
- MATLAB の CSV / JSON 入出力
- CAD連携用テキスト整形
- C++シミュレータ連携用JSON更新処理

## 入出力
- 入力: 最適化結果、`geom` 構造体、出力先ディレクトリ、既存車両パラメータJSON
- 出力: CSV、JSON、テキストレポート、更新済み車両パラメータ

## システム上の役割
POU設計を解析結果で終わらせず、設計審査・CAD検討・シミュレータ反映までつなぐ実務インターフェースです。MATLAB内で決まったハードポイントを他工程へ渡す最終段を担います。
