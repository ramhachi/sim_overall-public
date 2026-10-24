# test_frame_obj.m
## ファイル目的
- 実データに近い設定で `frameObjectiveFunction` を 1 回実行し、構文・主要出力を確認する。

## 実行フロー
- `real_vehicle_geometry.json`、材料、カタログ、サスペンション JSON を読み込む。
- exclusion zone を `config.exclusion_zones` へ変換する。
- テスト用設計変数 `x` を設定する。
- FEM サーバを起動し、目的関数を 1 回評価して主要指標を表示する。
- 成否にかかわらずサーバ停止を試みる。

## テスト詳細
### try ブロック全体
- シグネチャ: スクリプト
- 入力: 実 geometry / materials / catalog / suspension
- 出力: コンソール出力
- 責務: 実運用に近い単発評価
- 主要ロジック:
  - `config` に質量・CG・剛性・joint factor を設定
  - exclusion zone の `min/max` または `min_bounds/max_bounds` を正規化
  - `beam_fem_solver_tcp_java('start')` → `frameObjectiveFunction` → `stop`
- 依存関係:
  - `loadMaterials`, `loadPipeCatalog`, `frameObjectiveFunction`, `beam_fem_solver_tcp_java`
- 前提条件・注意事項:
  - 自動 assert 中心ではなくスモークテスト兼デバッグ出力型
  - `res.f_stiff` を表示しようとしているが、現 `frameObjectiveFunction` は `P_stiff` を返す実装で不整合がある

### catch ブロック
- シグネチャ: `catch e`
- 入力: 例外
- 出力: `getReport(e)`
- 責務: 障害時の詳細表示とサーバ停止
- 主要ロジック: 例外報告後も `beam_fem_solver_tcp_java('stop')`
- 依存関係: `getReport`
- 前提条件・注意事項: 停止処理自体も失敗し得る
