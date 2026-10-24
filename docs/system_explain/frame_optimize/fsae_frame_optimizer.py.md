# `frame_optimize/fsae_frame_optimizer.py`

## 役割

- Python 版フレーム最適化サブシステムの中核です。
- **3D 梁要素 FEM、継手柔性補正、重量計算、ねじり剛性評価、断面最適化、デモモデル生成**を 1 ファイルにまとめています。
- `fsae_regulation_checker.py` はこのソルバーとデータ構造を前提に動作します。

## 運用上の入口

### `run_demo()`
- スタンドアロン実行時の入口です。
- デモフレームを生成し、ベースライン評価→最適化→結果表示までを実行します。

### ライブラリ利用
- 他モジュールからは主に以下を import して使います。
  - `BeamFEMSolver`
  - `DesignVariable`
  - `FSAEFrameOptimizer`
  - `create_demo_fsae_frame()`
  - `create_demo_fsae_frame_with_map()`

## 主要データ構造

### `TubeMaterial`
- 管材物性を持つ dataclass です。
- 主属性: `name`, `E`, `G`, `rho`, `yield_stress`

### `TubeSection`
- 管断面を表す dataclass です。
- 入力属性: `shape`, `D_outer`, `t`
- 派生量として `D_inner`, `A`, `Iy`, `Iz`, `J`, `Asy`, `Asz` を property で返します。
- **副作用はなく**、断面性能の計算ユーティリティとして振る舞います。

### `Node`, `Element`, `BoundaryCondition`, `Load`
- FEM モデルの節点・要素・拘束・荷重を表す基礎 dataclass 群です。
- `Element` は `group` と `joint_factor` を持ち、最適化グループ管理と継手柔性補正に使われます。

### `DesignVariable`
- 最適化対象グループの外径・肉厚範囲をまとめる dataclass です。
- 属性: `group_name`, `shape`, `D_min`, `D_max`, `t_min`, `t_max`

## 主要関数・クラス

### `beam3d_local_stiffness(...)`
- **入力**: 梁長さ、材料定数、断面諸量、Timoshenko 使用有無
- **出力**: 12x12 ローカル剛性行列
- **役割**: 軸力・ねじり・2 軸曲げを持つ 3D 梁要素剛性を構成します。

### `rotation_matrix_3d(node_i, node_j, v_ref=None)`
- **入力**: 始終点座標、任意参照ベクトル
- **出力**: 12x12 座標変換行列
- **役割**: ローカル梁座標をグローバル座標へ回す変換を生成します。

### `joint_flexibility_factor(...)`
- **入力**: 主管/支管径、肉厚、接合角度
- **出力**: 0.5〜1.0 の柔性補正係数
- **役割**: CIDECT 系の経験則を簡略化し、溶接継手の曲げ・ねじり剛性低下を近似します。

### `class BeamFEMSolver`
- **役割**: 3D 梁 FEM のモデル管理と解析実行
- **主メソッド**:
  - `add_node/add_element/add_bc/add_load()` : モデル構築
  - `assemble()` : 全体剛性行列 `K` と荷重 `F` を組立
  - `solve()` : ペナルティ法で拘束を適用し `spsolve` で変位を解く
  - `total_mass()` : 全要素長さから質量を積算
  - `torsional_rigidity(front_nodes, rear_nodes, torque=1000.0)` : 前後端条件付きでねじり剛性を算出
- **副作用**:
  - `torsional_rigidity()` は内部で `bcs` と `loads` を一時的に差し替え、計算後に復元します。

### `class FSAEFrameOptimizer`
- **入力**: `solver`, `front_nodes`, `rear_nodes`, `design_vars`, 任意の `semantic_map`
- **役割**: 重量最小化と剛性最大化を単一目的化して探索する最適化器
- **主メソッド**:
  - `_apply_design(x)` : 設計変数を要素断面へ反映
  - `evaluate(x)` : 質量とねじり剛性を返す
  - `evaluate_with_compliance(x)` : さらに `fsae_regulation_checker` を呼んで規則適合を付加
  - `objective(x)` : 重み付き目的関数＋剛性不足/規則違反ペナルティ
  - `optimize(method, max_iter, seed)` : `differential_evolution` または `Nelder-Mead` を実行
- **戻り値**:
  - `optimize()` は `x_opt`, `mass`, `K_torsion`, `history`, `compliance_pass` などを含む辞書を返します。

### `create_demo_fsae_frame()` / `create_demo_fsae_frame_with_map()`
- デモ用 16 節点フレームを生成します。
- 後者は `FrameSemanticMap` も同時に返し、規則チェッカーと直結できます。

## データフロー

1. `TubeMaterial` / `TubeSection` / `Node` / `Element` でフレームを定義
2. `BeamFEMSolver.assemble()` が要素剛性を全体行列へ組立
3. `solve()` が変位を解き、`total_mass()` と `torsional_rigidity()` が性能値を計算
4. `FSAEFrameOptimizer._apply_design()` がグループ断面を更新
5. `evaluate()` / `objective()` が探索用指標へ変換
6. 必要に応じて `evaluate_with_compliance()` が規則適合結果を統合
7. `optimize()` が最適設計と履歴を返す

## システム内での位置づけ

- `frame_optimize` 配下の **Python 側解析カーネル** です。
- MATLAB 主体のフレーム最適化群とは別に、軽量な FEM 実験・規則適合付き探索・単体テスト可能な形で機能します。
- `fsae_regulation_checker.py` と `tests/test_regulation_checker.py` は本ファイルのデモモデルと API に依存しています。
