# `matlab/pou_hardpoint_design/`

## 概要
POU（Push-rod Operated Upright）ハードポイント設計系の索引です。車両仕様と既存サスペンション情報から、逆運動学と最適化を用いてプッシュロッド・ベルクランク・Mピボット位置を導出し、可視化とエクスポートまでを一連で扱います。ディレクトリは、ルートの実行入口、`geometry/` の幾何ソルバ、`optimization/` の目的関数・制約、`visualization/` の描画、`export/` の出力系に分かれます。

## 主要関数・クラス
- 入口:
  - [`main_pou_hardpoint_optimizer.m.md`](main_pou_hardpoint_optimizer.m.md)
  - [`run_all_wheels.m.md`](run_all_wheels.m.md)
  - [`example_usage.m.md`](example_usage.m.md)
  - [`example_usage_jp.m.md`](example_usage_jp.m.md)
- 検証:
  - [`test_bounds.m.md`](test_bounds.m.md)
  - [`test_json_loading.m.md`](test_json_loading.m.md)
  - [`test_pou_optimizer.m.md`](test_pou_optimizer.m.md)
  - [`verify_installation.m.md`](verify_installation.m.md)
- 下位モジュール:
  - [`geometry/README.md`](geometry/README.md)
  - [`optimization/README.md`](optimization/README.md)
  - [`visualization/README.md`](visualization/README.md)
  - [`export/README.md`](export/README.md)

## 理論・背景
この系は、車両全体最適化で得たジャックアップ・キャンバー目標を、ダブルウィッシュボーンとPOU機構の具体座標へ落とし込むための設計レイヤです。`fmincon` による非線形最適化、キングピン回りの剛体運動、ロッド長変化と取付比の評価、実装可能性を担保するパッケージ制約が核になります。

## 外部ライブラリ・ツールボックス
- MATLAB Optimization Toolbox
- MATLAB の `jsondecode` / `jsonencode`
- MATLAB 可視化機能
- CAD 連携用の CSV / JSON 出力

## 入出力
- 入力: `optimized_autocross.json` などの車両パラメータ、ホイール番号、マルチスタート回数、探索範囲、評価用ステア角レンジ
- 出力: Mピボット・ベルクランク座標、ジャックアップ曲線、キャンバー曲線、取付比、検証レポート、CSV / JSON / テキストレポート

## システム上の役割
このディレクトリは、Aアーム幾何が決まった後段でPOU機構を実装可能な形に仕上げる役目です。MATLABで幾何設計を確定し、その結果をCAD検討とC++シミュレータの双方へ受け渡す中継点として機能します。
