# verify_suspension_geometry.m
## ファイル目的
- 前輪サスペンション hardpoint 読み込み結果を可視化し、FBH とアーム配置を目視検証するスクリプト。

## 実行フロー
- 作業ディレクトリと JSON パスを計算する。
- `load_suspension_hardpoints` を呼ぶ。
- 取得座標を標準出力へ列挙する。
- FBH、地面、取付点、アップライト、A アームを 3D 描画する。

## スクリプト詳細
### 初期化部
- シグネチャ: スクリプト
- 入力: なし（内部で `offset_x=0.5`, `offset_z=0.0` を設定）
- 出力: 図とコンソール出力
- 責務:
  - 手動検証用可視化
- 主要ロジック:
  - `clear/clc/close all`
  - `suspension_base_geometry.json` を相対パス解決
- 依存関係:
  - `load_suspension_hardpoints`
  - `plot3`, `scatter3`, `surf`, `text`
- 前提条件・注意事項:
  - 自動テストではなく対話的検証用
  - 失敗時は `error` で停止

### 表示セクション
- シグネチャ: なし
- 入力:
  - `mount_nodes`, `upright_nodes`, `fbh_nodes`
- 出力:
  - ラベル付き 3D 図
- 責務:
  - 取付点配置と左右対称性の確認
- 主要ロジック:
  - FL/FR の前後内点を破線で接続
  - upright までアーム線を描き、ホイール中心も別記号で表示
- 依存関係:
  - MATLAB 可視化 API
- 前提条件・注意事項:
  - 凡例項目は固定文字列で、表示順と厳密一致していない場合がある
