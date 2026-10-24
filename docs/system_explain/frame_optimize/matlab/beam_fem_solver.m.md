# beam_fem_solver.m
## ファイル目的
- スタンドアロン C++ FEM ソルバをローカル実行または Docker 経由で呼び出す JSON ベースラッパ。

## 実行フロー
- 一時 JSON 入出力パスを `frame_optimize/temp` に作成する。
- `nodes/elements/bcs/loads` を JSON 化する。
- ローカル実行ファイルを優先探索し、無ければ Docker コマンドを生成する。
- ソルバ実行後に出力 JSON を読み戻し、`U/mass/K_torsion` を MATLAB 形式へ戻す。

## 関数詳細
### `result = beam_fem_solver(nodes, elements, bcs, loads, options)`
- シグネチャ: `function result = beam_fem_solver(nodes, elements, bcs, loads, options)`
- 入力:
  - `nodes`, `elements`, `bcs`, `loads`
  - `options`: 現状未使用
- 出力:
  - `result.success`, `result.mass`, `result.K_torsion`, `result.U`
- 責務:
  - 非常駐 FEM ソルバ呼び出し
- 主要ロジック:
  - 成功時のみ一時 JSON を削除
  - `U` は cell/struct/numeric いずれにも対応
- 依存関係:
  - `find_local_standalone_binary`
  - `build_docker_solver_command`
  - `force2D`
- 前提条件・注意事項:
  - Docker fallback は `docker-compose.yml` と Docker daemon が必要

### `out_cell = force2D(mat)`
- シグネチャ: `function out_cell = force2D(mat)`
- 入力: `mat`
- 出力: JSON 化しやすい行列/セル
- 責務:
  - 1 行配列を `[[...]]` としてエンコードさせる
- 主要ロジック:
  - 1xN のみ cell 化
- 依存関係: なし
- 前提条件・注意事項:
  - 空配列はそのまま返す

### `binary_path = find_local_standalone_binary(project_dir)`
- シグネチャ: `function binary_path = find_local_standalone_binary(project_dir)`
- 入力: `project_dir`
- 出力: 実行ファイルパス
- 責務:
  - ローカルソルバ探索
- 主要ロジック:
  - `bin`, `cpp/build*` を順に探索
- 依存関係: `isfile`
- 前提条件・注意事項:
  - 実行形式のネイティブ判定はしない

### `binary_path = find_incompatible_standalone_binary(project_dir)`
- シグネチャ: `function binary_path = find_incompatible_standalone_binary(project_dir)`
- 入力: `project_dir`
- 出力: Windows から見た非互換バイナリパス
- 責務:
  - Windows 向け診断
- 主要ロジック:
  - `.exe` ではない `beam_fem_standalone` を探索
- 依存関係: `isfile`
- 前提条件・注意事項:
  - Windows 以外では空文字を返す

### `docker_cmd = build_docker_solver_command(project_dir, timestamp)`
- シグネチャ: `function docker_cmd = build_docker_solver_command(project_dir, timestamp)`
- 入力:
  - `project_dir`, `timestamp`
- 出力:
  - `docker_cmd`
- 責務:
  - Docker 実行コマンド組立
- 主要ロジック:
  - `docker compose` 優先、無ければ `docker-compose`
  - `fsae-sim` コンテナ内で `beam_fem_standalone` を実行
- 依存関係:
  - `system`, `shell_quote`
- 前提条件・注意事項:
  - Docker 未使用環境では詳細エラーを返す

### `quoted = shell_quote(value)`
- シグネチャ: `function quoted = shell_quote(value)`
- 入力: `value`
- 出力: ダブルクォート囲み文字列
- 責務:
  - シェル引数の簡易クォート
- 主要ロジック:
  - `"` で包むのみ
- 依存関係: なし
- 前提条件・注意事項:
  - 内部にダブルクォートを含む値の完全エスケープはしない
