# test_regulation_checker.py
## ファイル目的
- Python 実装の FSAE 2026 レギュレーションチェッカーと最適化器のコンプライアンス連携を広範囲に検証する。

## 実行フロー
- `create_demo_fsae_frame` 系 fixture でデモフレームを作る。
- 管材仕様、角度、形状、IA、材料、統合チェック、レポート、最適化連携を `pytest` クラスごとに検証する。

## フィクスチャ詳細
### `demo_frame()`
- シグネチャ: `def demo_frame()`
- 入力: なし
- 出力: `(solver, front_nodes, rear_nodes)`
- 責務: デモフレーム供給

### `default_smap()`
- シグネチャ: `def default_smap()`
- 出力: `FrameSemanticMap()`
- 責務: 既定意味マップ供給

### `checker(demo_frame, default_smap)`
- シグネチャ: `def checker(demo_frame, default_smap)`
- 出力: `FSAERegulationChecker`
- 責務: 共通チェッカー構築

## テストクラス詳細
### `class TestTubeSpecs`
- 責務: F.3.2 管材仕様
- 個別テスト:
  - `test_tube_spec_returns_results`: 結果配列と型確認
  - `test_tube_spec_all_have_rule_id`: 全結果 `rule_id=="F.3.2"`
  - `test_main_hoop_roll_tube_od_pass`: main hoop 外径 pass
  - `test_brace_tube_od_pass`: brace 外径 pass
  - `test_tube_spec_fail_thin_wall`: 薄肉 main hoop fail
  - `test_tube_spec_fail_small_od`: 小外径 main hoop fail

### `class TestAngleCalculations`
- 責務: 鉛直基準角度とフープ/ブレース角度
- 個別テスト:
  - `test_vertical_vector_angle_zero`: 鉛直 0°
  - `test_tilted_vector_angle`: 45° ベクトル
  - `test_horizontal_vector_angle_90`: 水平 90°
  - `test_main_hoop_angle_pass`: F.5.8.3 pass
  - `test_front_hoop_angle_pass`: F.5.7.6 pass
  - `test_main_hoop_angle_fail`: 20° 傾斜 fail
  - `test_brace_angle_pass`: ブレース角度 pass

### `class TestGeometryChecks`
- 責務: 寸法・幅・ホイールベース・トレッド比・Side Impact 高さ
- 個別テスト:
  - `test_brace_attachment_pass`
  - `test_main_hoop_width_pass`
  - `test_main_hoop_width_fail`
  - `test_wheelbase_pass`
  - `test_wheelbase_fail`
  - `test_track_ratio_pass`
  - `test_track_ratio_fail`
  - `test_side_impact_height_pass`
  - `test_side_impact_height_fail_low`
  - `test_side_impact_height_fail_high`

### `class TestImpactAttenuator`
- 責務: IA 寸法・吸収エネルギー
- 個別テスト:
  - `test_ia_dimensions_pass`
  - `test_ia_dimensions_fail_short`
  - `test_ia_energy_pass`
  - `test_ia_energy_fail_small_volume`

### `class TestMaterialChecks`
- 責務: main hoop と bracing 材料適合
- 個別テスト:
  - `test_main_hoop_steel_pass`
  - `test_main_hoop_aluminum_fail`
  - `test_bracing_material_pass`（コメント上は demo brace が実際には fail することを確認）

### `class TestCheckAll`
- 責務: `check_all()` 統合結果
- 個別テスト:
  - `test_check_all_returns_list`
  - `test_check_all_result_types`
  - `test_check_all_categories`
  - `test_demo_frame_compliance`

### `class TestReportGeneration`
- 責務: 文字列/辞書レポート生成
- 個別テスト:
  - `test_generate_report_string`
  - `test_generate_report_contains_results`
  - `test_generate_report_dict`
  - `test_report_dict_check_format`

### `class TestEvaluateWithCompliance`
- 責務: 最適化器の compliance 連携
- 個別テスト:
  - `test_evaluate_with_compliance`
  - `test_evaluate_without_semantic_map`
  - `test_objective_penalizes_noncompliance_in_loop`
  - `test_optimize_returns_compliance_summary`

### `class TestCreateDemoWithMap`
- 責務: `create_demo_fsae_frame_with_map` 補助 API
- 個別テスト:
  - `test_returns_four_items`
  - `test_semantic_map_type`
  - `test_integration_with_checker`

## 依存関係
- `pytest`, `numpy`
- `fsae_frame_optimizer`
- `fsae_regulation_checker`

## 前提条件・注意事項
- MATLAB 版フレーム最適化とは別の Python 実装を対象にしている。
- 個別テスト名とコメントが実際の期待値を説明しているため、仕様把握に有用。
