# MATLAB ソースリファレンス

このディレクトリは、リポジトリ直下の MATLAB `.m` ファイルを対象にした日本語のファイル別リファレンスです。最適化、車両パラメータ構築、アーティファクト入出力、シミュレーション実行、可視化、検証スクリプトまでを、役割ごとに参照できるよう整理しています。

## 概念設計・設計変数適用

- [`ConceptTradeOffs.m.md`](./ConceptTradeOffs.m.md) — 概念設計段階で使う離散選択肢と連続トレードオフ設定。
- [`compute_aero_cd_a.m.md`](./compute_aero_cd_a.m.md) — 揚力係数と抗力係数の簡易トレードオフモデル。
- [`compute_track_width_mass_distribution.m.md`](./compute_track_width_mass_distribution.m.md) — トレッド拡大量に応じた質量増加推定。
- [`apply_tradeoff_design.m.md`](./apply_tradeoff_design.m.md) — ConceptTradeOffs の選択肢を実車パラメータへ反映。
- [`apply_full_design.m.md`](./apply_full_design.m.md) — 大規模最適化ベクトルを完全な車両構造体へ変換。
- [`apply_vehicle_extra_tunables.m.md`](./apply_vehicle_extra_tunables.m.md) — 拡張チューナブル項目を車両パラメータへ加算。
- [`build_extended_vehicle_tunable_specs.m.md`](./build_extended_vehicle_tunable_specs.m.md) — 共有チューナブル仕様の定義レジストリ。
- [`create_vehicle_baseline_template.m.md`](./create_vehicle_baseline_template.m.md) — C++ デフォルトに依存しない基準車両テンプレート生成。
- [`apply_named_tire_model_selection.m.md`](./apply_named_tire_model_selection.m.md) — タイヤモデルのカタログ選択と適用。

## シミュレーション実行・パイプライン

- [`VehicleSimRunner.m.md`](./VehicleSimRunner.m.md) — MATLAB から Docker 内 C++ シミュレータを呼び出す実行ラッパー。
- [`run_full_pipeline.m.md`](./run_full_pipeline.m.md) — PID 最適化、実行、可視化をつなぐ統合入口。
- [`find_vehicle_json_candidates.m.md`](./find_vehicle_json_candidates.m.md) — 車両 artifact 候補の探索・正規化・優先度付け。
- [`select_vehicle_json_candidate.m.md`](./select_vehicle_json_candidate.m.md) — 候補一覧から人手で 1 件選ぶ共通 UI。
- [`resolve_default_vehicle_json.m.md`](./resolve_default_vehicle_json.m.md) — 既定 artifact を shared selector で自動解決。
- [`grid_search_pid.m.md`](./grid_search_pid.m.md) — PID ゲインの高速グリッド探索。
- [`optimize_pid_gains.m.md`](./optimize_pid_gains.m.md) — PID ゲインのベイズ最適化。
- [`optimize_concept_design.m.md`](./optimize_concept_design.m.md) — 階層最適化第1段階の概念設計最適化。
- `run_full_vehicle_ga.m` — LPV-LQRとPINNで評価する再開可能な車両諸元GA。詳細は`docs/vehicle_ga_platform.md`。
- [`optimize_multi_event.m.md`](./optimize_multi_event.m.md) — 複数イベントを同時に満たす折衷最適化。

## アーティファクト・データ入出力

- [`build_vehicle_artifact.m.md`](./build_vehicle_artifact.m.md) — 車両仕様と評価結果を統合 JSON 用構造体へ集約。
- [`save_vehicle_artifact.m.md`](./save_vehicle_artifact.m.md) — 統合アーティファクトの JSON 保存。
- [`load_vehicle_artifact.m.md`](./load_vehicle_artifact.m.md) — JSON アーティファクトの読み込みと正規化。
- [`save_suspension_geometry_artifact.m.md`](./save_suspension_geometry_artifact.m.md) — サスペンション結果を同一 artifact へ追記保存。
- [`extract_vehicle_params_from_artifact.m.md`](./extract_vehicle_params_from_artifact.m.md) — 統合形式と旧形式の差を吸収して車両仕様を抽出。
- [`import_engine_csv.m.md`](./import_engine_csv.m.md) — エンジントルク CSV の MATLAB 向け変換。
- [`merge_nested_structs.m.md`](./merge_nested_structs.m.md) — ネスト構造体の再帰マージ。
- [`set_nested_struct_field.m.md`](./set_nested_struct_field.m.md) — パス文字列による動的フィールド設定。

## 可視化・比較・トラック作成

- [`TrackDesigner.m.md`](./TrackDesigner.m.md) — クリック操作でコース中心線を作る GUI ツール。
- [`compare_results.m.md`](./compare_results.m.md) — 2 つのシミュレーション結果 CSV の比較可視化。
- [`run_comparison.m.md`](./run_comparison.m.md) — 比較可視化の簡易起動スクリプト。
- [`run_visualization.m.md`](./run_visualization.m.md) — 保存済み CSV を対象にした可視化起動スクリプト。
- [`visualize_laptime_sim.m.md`](./visualize_laptime_sim.m.md) — ラップタイム系シミュレーションの主可視化関数。
- [`visualize_14dof_cpp_result.m.md`](./visualize_14dof_cpp_result.m.md) — 14DOF C++ 結果の多面表示。
- [`visualize_debug.m.md`](./visualize_debug.m.md) — 自動車線上の軌跡デバッグ表示。

## 検証・デバッグ・開発補助

- `tests/test_vehicle_ga_platform.m` — 車両GAの設定、schema、seed、gate、checkpoint契約テスト。
- [`test_figure8_generation.m.md`](./test_figure8_generation.m.md) — 8 の字コース生成の形状確認。
- [`test_skidpad_track.m.md`](./test_skidpad_track.m.md) — SAE スキッドパッド幾何の検証。
- [`verify_skidpad_time.m.md`](./verify_skidpad_time.m.md) — 最適化済み概念設計のスキッドパッド再評価。
- [`verify_steering_param.m.md`](./verify_steering_param.m.md) — 操舵関連パラメータの受け渡し検証。

## 補足

- ここで扱うのは**リポジトリ直下の MATLAB ファイル**です。
- `rl_driver/` などのサブディレクトリは個別ディレクトリで管理されます。
- `parameter_presets/` は EasyStart が選択する固定 JSON プリセット置き場です。
- 各ページは「概要」「主要関数・クラス」「理論・背景」「外部ライブラリ・ツールボックス」「入出力」「システム上の役割」の共通見出しで統一しています。
