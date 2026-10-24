# Implementation Plan: Tire Model Fitting

## Overview

本実装計画は、FSAE Tire Test Consortium (TTC)の実験データからTMeasyタイヤモデルのパラメータをフィッティングするシステムを段階的に構築します。実装は、データ読み込み→前処理→モデル実装→最適化→評価→出力の順に進め、各段階でテストを実施して正確性を確保します。

## Tasks

- [x] 1. プロジェクト構造とデータローダーの実装
  - プロジェクトディレクトリ構造を作成（tire_fitting/, tests/, output/）
  - TireDataLoaderクラスを実装してDriveBrakeとCorneringのCSVファイルを読み込む
  - FZ符号変換（負値→正値）を実装
  - 必須カラムの存在チェックとエラーハンドリングを実装
  - _Requirements: 1.1, 1.2, 1.3, 1.4, 1.5_

- [x] 1.1 Write property test for CSV data loading
  - **Property 1: CSV Data Loading Preserves All Required Columns**
  - **Validates: Requirements 1.1**

- [x] 1.2 Write property test for FZ sign conversion
  - **Property 2: FZ Sign Conversion Round Trip**
  - **Validates: Requirements 1.3**

- [x] 1.3 Write property test for missing column detection
  - **Property 3: Missing Column Error Detection**
  - **Validates: Requirements 1.5**

- [x] 2. データ前処理とフィルタリングの実装
  - [x] 2.1 TireDataPreprocessorクラスを実装
    - filter_data()メソッドで車速、垂直荷重、スリップ角/比、温度のフィルタリングを実装
    - 各フィルタ条件をboolean indexingで適用
    - _Requirements: 2.1, 2.2, 2.3, 2.4, 2.5_

  - [x] 2.2 Write property test for data filtering
    - **Property 4: Data Filtering Preserves Only Valid Rows**
    - **Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5**

  - [x] 2.3 荷重グループ化機能を実装
    - group_by_load()メソッドでLow/Medium/Highの3グループに分類
    - 荷重範囲: Low (600-900N), Medium (900-1100N), High (1100-1400N)
    - _Requirements: 3.1, 3.2, 3.3, 3.4_

  - [x] 2.4 Write property test for load grouping
    - **Property 5: Load Grouping Classification Correctness**
    - **Validates: Requirements 3.1, 3.2, 3.3, 3.4**

  - [x] 2.5 IQR法による外れ値除去を実装
    - remove_outliers_iqr()メソッドでQ1, Q3, IQRを計算
    - Q1 - 1.5*IQR と Q3 + 1.5*IQR の範囲外を除去
    - _Requirements: 4.1, 4.2, 4.3, 4.4_

  - [x] 2.6 Write property test for outlier removal
    - **Property 6: IQR Outlier Removal Completeness**
    - **Validates: Requirements 4.1, 4.2, 4.3, 4.4**

- [x] 3. Checkpoint - データ処理の検証
  - すべてのデータ処理テストが成功することを確認
  - 実際のTTCデータファイルで読み込みと前処理を実行し、結果を確認
  - 質問があればユーザーに確認

- [x] 4. TMeasyモデルエンジンの実装
  - [x] 4.1 TMeasyModelクラスの基本構造を実装
    - __init__()で基準荷重fz_nominalを設定
    - 荷重依存摩擦係数の計算メソッドを実装
    - _Requirements: 5.5, 6.3_

  - [x] 4.2 Write property test for load-dependent friction
    - **Property 10: Load-Dependent Friction Coefficient Formula**
    - **Validates: Requirements 5.5, 6.3**

  - [x] 4.3 縦力計算メソッドを実装
    - compute_longitudinal_force()で正規化変形量xi_barを計算
    - 飽和関数mu_effを実装（s < 1: s*(2-s), s >= 1: 1）
    - 縦力Fxの計算式を実装
    - _Requirements: 5.1, 5.2, 5.3, 5.4_

  - [x] 4.4 Write property test for longitudinal normalized deformation
    - **Property 7: Longitudinal Normalized Deformation Formula**
    - **Validates: Requirements 5.1**

  - [x] 4.5 Write property test for saturation function
    - **Property 8: Saturation Function Piecewise Correctness**
    - **Validates: Requirements 5.2, 5.3**

  - [x] 4.6 Write property test for longitudinal force calculation
    - **Property 9: Longitudinal Force Calculation Formula**
    - **Validates: Requirements 5.4**

  - [x] 4.7 横力計算メソッドを実装
    - compute_lateral_force()で正規化変形量eta_barを計算
    - キャンバースラストの追加を実装
    - 横力Fyの計算式を実装
    - _Requirements: 6.1, 6.2, 6.4_

  - [x] 4.8 Write property test for lateral normalized deformation
    - **Property 11: Lateral Normalized Deformation Formula**
    - **Validates: Requirements 6.1**

  - [x] 4.9 Write property test for camber thrust
    - **Property 12: Camber Thrust Addition**
    - **Validates: Requirements 6.2**

  - [x] 4.10 Write property test for saturation consistency
    - **Property 13: Saturation Function Consistency**
    - **Validates: Requirements 6.4**

- [x] 5. Checkpoint - モデル実装の検証
  - すべてのモデルテストが成功することを確認
  - 既知のパラメータで力の計算が正しいことを手動で確認
  - 質問があればユーザーに確認

- [x] 6. パラメータ最適化の実装
  - [x] 6.1 TireParameterOptimizerクラスを実装
    - __init__()でTMeasyModelインスタンスを受け取る
    - パラメータ境界の定義を実装
    - _Requirements: 7.2, 7.3, 7.4, 8.2, 8.3, 8.4_

  - [x] 6.2 縦力フィッティングメソッドを実装
    - fit_longitudinal()でmu_x_peak, k_mu_x, Kxを最適化
    - 差分進化法（scipy.optimize.differential_evolution）を使用
    - Levenberg-Marquardt法（scipy.optimize.least_squares）で局所最適化
    - 正規化残差二乗和を目的関数として使用
    - _Requirements: 7.1, 7.2, 7.3, 7.4_

  - [x] 6.3 Write property test for longitudinal optimizer bounds
    - **Property 14: Longitudinal Optimizer Returns Valid Parameters**
    - **Validates: Requirements 7.1, 7.2, 7.3, 7.4**

  - [x] 6.4 横力フィッティングメソッドを実装
    - fit_lateral()でmu_y_peak, k_mu_y, Ky, C_gammaを最適化
    - 縦力と同じ最適化戦略を使用
    - _Requirements: 8.1, 8.2, 8.3, 8.4_

  - [x] 6.5 Write property test for lateral optimizer bounds
    - **Property 15: Lateral Optimizer Returns Valid Parameters**
    - **Validates: Requirements 8.1, 8.2, 8.3, 8.4**

- [x] 7. 評価メトリクスと可視化の実装
  - [x] 7.1 FitEvaluatorクラスを実装
    - calculate_metrics()でRMSE, NRMSE, R², TICを計算
    - calculate_peak_friction_error()でピーク摩擦係数誤差を計算
    - _Requirements: 9.1, 9.2, 9.3, 9.4, 9.5_

  - [x] 7.2 Write property test for RMSE calculation
    - **Property 16: RMSE Calculation Formula**
    - **Validates: Requirements 9.1**

  - [x] 7.3 Write property test for NRMSE calculation
    - **Property 17: NRMSE Calculation Formula**
    - **Validates: Requirements 9.2**

  - [x] 7.4 Write property test for R-squared calculation
    - **Property 18: R-Squared Calculation Formula**
    - **Validates: Requirements 9.3**

  - [x] 7.5 Write property test for TIC calculation
    - **Property 19: Theil Inequality Coefficient Formula**
    - **Validates: Requirements 9.4**

  - [x] 7.6 Write property test for peak friction error
    - **Property 20: Peak Friction Error Calculation**
    - **Validates: Requirements 9.5**

  - [x] 7.7 プロット生成メソッドを実装
    - plot_force_vs_slip()でFx vs SR、Fy vs SAのプロットを生成
    - plot_residuals()で残差プロットを生成
    - matplotlibを使用してPNG形式で保存
    - _Requirements: 10.1, 10.2, 10.3, 10.4, 10.5_

- [x] 8. パラメータエクスポートの実装
  - [x] 8.1 ParameterExporterクラスを実装
    - export_yaml()でYAMLファイルを出力
    - tire_model, tire_name, effective_radius, nominal_loadフィールドを含む
    - longitudinal, lateral, camberセクションを含む
    - fitting_infoセクションにメトリクスを含む
    - _Requirements: 11.1, 11.2, 11.3, 11.4, 11.5_

  - [x] 8.2 Write property test for YAML serialization
    - **Property 21: YAML Parameter Serialization Round Trip**
    - **Validates: Requirements 11.1, 11.2, 11.3, 11.4, 11.5**

  - [x] 8.3 C++ヘッダファイル出力を実装
    - export_cpp_header()でC++ヘッダファイルを生成
    - include guardsを追加
    - fsae::tire名前空間内にstructを定義
    - すべてのパラメータをstatic constexpr doubleメンバーとして定義
    - コメントでタイヤ名、生成日、単位を記載
    - _Requirements: 12.1, 12.2, 12.3, 12.4, 12.5_

  - [x] 8.4 Write property test for C++ header structure
    - **Property 22: C++ Header Structure Correctness**
    - **Validates: Requirements 12.2, 12.3, 12.4**

- [x] 9. Checkpoint - エクスポート機能の検証
  - YAMLファイルが正しく生成され、パースできることを確認
  - C++ヘッダファイルが有効な構文を持つことを確認
  - 質問があればユーザーに確認

- [x] 10. メインスクリプトと統合
  - [x] 10.1 メインスクリプトを作成
    - コマンドライン引数でデータファイルパスを受け取る
    - データ読み込み→前処理→フィッティング→評価→出力の完全なパイプラインを実装
    - 進捗状況をログ出力
    - _Requirements: 1.1-13.5_

  - [x] 10.2 エラーハンドリングを追加
    - ファイル読み込みエラー、最適化失敗、出力エラーを適切に処理
    - エラーメッセージをユーザーフレンドリーに表示
    - _Requirements: 1.4, 1.5, 13.5_

  - [x] 10.3 目標精度の検証を実装
    - フィッティング完了後にRMSE < 100N, TIC < 0.15, ピーク誤差 < 5%を確認
    - 目標未達成の場合は警告を表示
    - _Requirements: 13.1, 13.2, 13.3, 13.4, 13.5_

- [x] 11. End-to-End統合テスト
  - 実際のTTCデータファイル（DriveBrake-Hoosier18.0×6.0-10-R20.csv, Cornering-Hoosier18.0×6.0-10-R20.csv）を使用
  - 完全なフィッティングパイプラインを実行
  - 目標精度が達成されることを確認
  - YAMLとC++ヘッダファイルが正しく生成されることを確認
  - プロットが生成されることを確認
  - _Requirements: 1.1-13.5_

- [x] 12. Final Checkpoint - 全体検証
  - すべてのテストが成功することを確認
  - 生成されたパラメータファイルを確認
  - プロットを目視で確認
  - ドキュメントを更新（README、使用方法）
  - 質問があればユーザーに確認

## Notes

- Each task references specific requirements for traceability
- Checkpoints ensure incremental validation at key milestones
- Property tests validate universal correctness properties across random inputs
- Unit tests validate specific examples and edge cases
- The implementation follows a bottom-up approach: data → model → optimization → evaluation → export
- Python 3.x with pandas, numpy, scipy, matplotlib, PyYAML is required
