# build_mex.m
## ファイル目的
- Eigen3 依存の C++ MEX ソルバをビルドする補助スクリプト。

## 実行フロー
- `mex/beam_fem_solver.cpp` の存在を確認する。
- `detect_eigen_path` で Eigen インクルード先を探索する。
- OS ごとの最適化フラグを付けて `mex` を実行する。
- 出力 MEX ファイルの存在を確認する。

## 関数詳細
### `build_mex()`
- シグネチャ: `function build_mex()`
- 入力: なし
- 出力: なし
- 責務:
  - MEX コンパイル実行
- 主要ロジック:
  - `-R2018a` と C++14 フラグを付与
  - 作業ディレクトリを `mex_dir` へ一時変更し、失敗時も元へ戻す
- 依存関係:
  - `detect_eigen_path`
  - `mex`
- 前提条件・注意事項:
  - Eigen3 が未導入なら `error`

### `eigen_path = detect_eigen_path()`
- シグネチャ: `function eigen_path = detect_eigen_path()`
- 入力: なし
- 出力:
  - `eigen_path`
- 責務:
  - Eigen インストール先探索
- 主要ロジック:
  - 既知パスを順に探索し、Unix 系では `pkg-config --cflags eigen3` も試す
- 依存関係:
  - `system`, `regexp`
- 前提条件・注意事項:
  - Windows は代表的手動配置先のみ探索
