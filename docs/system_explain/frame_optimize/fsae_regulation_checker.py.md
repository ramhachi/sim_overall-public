# `frame_optimize/fsae_regulation_checker.py`

## 役割

- Python 版フレームモデルに対し、**FSAE 2026 の管材・フープ角度・ブレース・寸法・インパクトアッテネータ・材料**を一括検証する適合チェッカーです。
- `fsae_frame_optimizer.py` の `BeamFEMSolver` と `FrameSemanticMap` を受け取り、最適化結果の規則適合判定にも使われます。

## 運用上の入口

### `FSAERegulationChecker.check_all()`
- 実運用の主入口です。
- 個別チェックをすべて実行し、`RegulationCheckResult` の配列を返します。

### スタンドアロン実行
- `__main__` では `create_demo_fsae_frame()` と `FrameSemanticMap()` を使って全チェックを回し、テキストレポートを表示します。

## 主要データ構造

### `TUBE_SPECS`
- `Size_A`〜`Size_D` の最小 `I`, `A`, `OD`, `t` を定義するルールテーブルです。

### `DEFAULT_GROUP_SPEC_MAP`
- 要素グループ名を要求サイズへ結び付ける既定マッピングです。
- 例: `main_hoop -> Size_A`, `upper_rail -> Size_B`

### `RegulationCheckResult`
- 1 項目の判定結果を表す dataclass です。
- 主属性: `rule_id`, `category`, `description`, `passed`, `value`, `required`, `severity`

### `FrameSemanticMap`
- 節点 ID と意味的役割を結ぶ dataclass です。
- フープ四隅、フロントバルクヘッド、リア、ブレース接続点、ホイールベース、トレッド、サイドインパクト高さ、IA 寸法、グループ仕様対応を保持します。

## 主要クラス

### `class FSAERegulationChecker`
- **入力**: `solver`, `semantic_map`
- **役割**: 幾何・材料・寸法ルールを個別に判定し、結果を集約する

#### ユーティリティ
- `_get_node(node_id)` : 節点取得
- `_calc_angle_from_vertical(bot_node_id, top_node_id)` : 鉛直との角度算出
- `_calc_angle_between_vectors(v1, v2)` : 2 ベクトル角度算出
- `_lowest_frame_z()` : 最低点高さ取得

#### 代表的な個別チェック
- `check_tube_specs()` : グループごとに `OD`, `t`, `A`, `I` を `TUBE_SPECS` と比較
- `check_main_hoop_angle()` / `check_front_hoop_angle()` : フープ傾斜角の上限判定
- `check_brace_angle()` / `check_brace_attachment_height()` : ブレース角度・接続高さ判定
- `check_main_hoop_width()` : メインフープ底部内寸幅判定
- `check_wheelbase()` / `check_track_ratio()` / `check_side_impact_height()` : 車両寸法判定
- `check_impact_attenuator_dimensions()` / `check_impact_attenuator_energy()` : IA 寸法と簡易エネルギ推定
- `check_main_hoop_material()` / `check_bracing_material()` : 鉄鋼/アルミ条件の判定

#### レポート生成
- `generate_report(results)` : 人間向けテキストレポート
- `generate_report_dict(results)` : ダッシュボード連携向け辞書

## 入出力・副作用

- **入力**: `BeamFEMSolver` が持つ節点・要素・材料情報、`FrameSemanticMap` の寸法情報
- **出力**: `RegulationCheckResult` 配列、テキストレポート、JSON 互換辞書
- **副作用**: なし

## データフロー

1. `solver.elements` をグループ単位に整理
2. `FrameSemanticMap` から意味付き節点と寸法基準を取得
3. 各ルール判定メソッドが `RegulationCheckResult` を生成
4. `check_all()` が結果を集約
5. レポート関数が人間可読形式または API 形式へ変換

## システム内での位置づけ

- `FSAEFrameOptimizer.evaluate_with_compliance()` から呼ばれる**規則適合レイヤ**です。
- 軽量 FEM の性能評価だけでは不足する「FSAE ルールを満たしているか」を補い、最適化の目的関数にも違反ペナルティを与えるための橋渡しを担います。
