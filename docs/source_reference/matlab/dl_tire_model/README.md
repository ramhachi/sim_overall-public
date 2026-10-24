# DNN タイヤモデル学習・推論パイプライン

## 概要
`matlab/dl_tire_model` は、TTC 由来のタイヤ試験データを前処理し、深層学習モデルを学習・評価し、C++ 実装向けに書き出すための MATLAB パイプラインです。加えて、ハイパーパラメータ探索、可視化、最終検証、TMeasy や Magic Formula との比較用スクリプトも含みます。

## 主要ファイル群
- **データ層**: `loadTireData`, `extractFeatures`, `extractTargets`, `augment_tire_data_for_nn`
- **前処理**: `preprocessData`, `preprocessDataWrapper`, `removeMissingData`, `normalizeData`, `splitData`
- **モデル生成**: `createTireModel`, `createDeepTireModel`, `createVeryDeepTireModel`, `createLSTMTireModel`, `createModelFromConfig`
- **学習**: `trainTireModel`, `createTrainingOptions`, `checkGPUAvailability`
- **評価**: `evaluateModel`, `computeRMSE`, `computeR2`, `computeTIC`
- **可視化**: `generateAllVisualizations` と各種プロット生成関数
- **エクスポート**: `exportModel`, `exportForCpp`, `exportModelForCpp`, `generateLookupTable`
- **探索・実験**: `main_hyperparameter_tuning`, `runHyperparameterSearch`, `compareExperiments`, `logExperiment`
- **メインワークフロー**: `main_training_workflow`, `main_prediction_workflow`
- **検証・診断**: `final_system_validation`, `run_*_validation`, `check_*`, `verify_fx_fy_swap`
- **比較モデル**: `magic_formula_fitting`, `tmeasy_*`
- **クイックリファレンス**: `*_quick_reference`

## 文書一覧

### データ・前処理
- [augment_tire_data_for_nn.m.md](./augment_tire_data_for_nn.m.md)
- [check_fz_dist.m.md](./check_fz_dist.m.md)
- [check_symmetry.m.md](./check_symmetry.m.md)
- [computeInputRanges.m.md](./computeInputRanges.m.md)
- [denormalizeData.m.md](./denormalizeData.m.md)
- [detectOutOfRange.m.md](./detectOutOfRange.m.md)
- [extractFeatures.m.md](./extractFeatures.m.md)
- [extractTargets.m.md](./extractTargets.m.md)
- [loadTireData.m.md](./loadTireData.m.md)
- [normalizeData.m.md](./normalizeData.m.md)
- [preprocessData.m.md](./preprocessData.m.md)
- [preprocessDataWrapper.m.md](./preprocessDataWrapper.m.md)
- [removeMissingData.m.md](./removeMissingData.m.md)
- [splitData.m.md](./splitData.m.md)
- [validateTireData.m.md](./validateTireData.m.md)
- [validate_zero_slip_behavior.m.md](./validate_zero_slip_behavior.m.md)
- [verify_fx_fy_swap.m.md](./verify_fx_fy_swap.m.md)

### モデル生成・学習
- [checkGPUAvailability.m.md](./checkGPUAvailability.m.md)
- [check_validation_readiness.m.md](./check_validation_readiness.m.md)
- [clean_and_retrain.m.md](./clean_and_retrain.m.md)
- [createDeepTireModel.m.md](./createDeepTireModel.m.md)
- [createHyperparameterConfig.m.md](./createHyperparameterConfig.m.md)
- [createLSTMTireModel.m.md](./createLSTMTireModel.m.md)
- [createModelFromConfig.m.md](./createModelFromConfig.m.md)
- [createTireModel.m.md](./createTireModel.m.md)
- [createTrainingOptions.m.md](./createTrainingOptions.m.md)
- [createVeryDeepTireModel.m.md](./createVeryDeepTireModel.m.md)
- [trainTireModel.m.md](./trainTireModel.m.md)

### 評価・可視化
- [computeR2.m.md](./computeR2.m.md)
- [computeRMSE.m.md](./computeRMSE.m.md)
- [computeTIC.m.md](./computeTIC.m.md)
- [diagnose_drivebrake_data.m.md](./diagnose_drivebrake_data.m.md)
- [evaluateModel.m.md](./evaluateModel.m.md)
- [final_system_validation.m.md](./final_system_validation.m.md)
- [generateAllVisualizations.m.md](./generateAllVisualizations.m.md)
- [generateLookupTable.m.md](./generateLookupTable.m.md)
- [generateOperatingConditionPlots.m.md](./generateOperatingConditionPlots.m.md)
- [generateResidualPlots.m.md](./generateResidualPlots.m.md)
- [generateScatterPlots.m.md](./generateScatterPlots.m.md)
- [generateTimeSeriesPlots.m.md](./generateTimeSeriesPlots.m.md)

### エクスポート・推論
- [exportForCpp.m.md](./exportForCpp.m.md)
- [exportMagicFormulaForCpp.m.md](./exportMagicFormulaForCpp.m.md)
- [exportModel.m.md](./exportModel.m.md)
- [exportModelForCpp.m.md](./exportModelForCpp.m.md)
- [exportModelWithValidation.m.md](./exportModelWithValidation.m.md)
- [exportNetworkWeights.m.md](./exportNetworkWeights.m.md)
- [exportNormalizationParams.m.md](./exportNormalizationParams.m.md)
- [loadAndPredict.m.md](./loadAndPredict.m.md)
- [predictTireForces.m.md](./predictTireForces.m.md)
- [run_cpp_export.m.md](./run_cpp_export.m.md)
- [run_export_for_cpp.m.md](./run_export_for_cpp.m.md)

### 実験・ワークフロー
- [compareExperiments.m.md](./compareExperiments.m.md)
- [hyperparameter_tuning_quick_reference.m.md](./hyperparameter_tuning_quick_reference.m.md)
- [loadExperimentLog.m.md](./loadExperimentLog.m.md)
- [loadExperimentModel.m.md](./loadExperimentModel.m.md)
- [logExperiment.m.md](./logExperiment.m.md)
- [main_hyperparameter_tuning.m.md](./main_hyperparameter_tuning.m.md)
- [main_prediction_workflow.m.md](./main_prediction_workflow.m.md)
- [main_training_workflow.m.md](./main_training_workflow.m.md)
- [runHyperparameterSearch.m.md](./runHyperparameterSearch.m.md)
- [run_final_validation.m.md](./run_final_validation.m.md)
- [run_improved_validation.m.md](./run_improved_validation.m.md)
- [run_optimized_validation.m.md](./run_optimized_validation.m.md)
- [run_quick_validation.m.md](./run_quick_validation.m.md)
- [run_separated_training.m.md](./run_separated_training.m.md)
- [run_simple_validation.m.md](./run_simple_validation.m.md)
- [run_ultra_deep_validation.m.md](./run_ultra_deep_validation.m.md)
- [saveExperimentModel.m.md](./saveExperimentModel.m.md)
- [workflow_quick_reference.m.md](./workflow_quick_reference.m.md)

### 比較モデル・資料
- [export_quick_reference.m.md](./export_quick_reference.m.md)
- [magic_formula_fitting.m.md](./magic_formula_fitting.m.md)
- [tmeasy_optimized.m.md](./tmeasy_optimized.m.md)
- [tmeasy_optimized_v2.m.md](./tmeasy_optimized_v2.m.md)
- [tmeasy_parameter_fitting.m.md](./tmeasy_parameter_fitting.m.md)
- [tmeasy_realistic_fitting.m.md](./tmeasy_realistic_fitting.m.md)
- [validation_quick_reference.m.md](./validation_quick_reference.m.md)
- [visualization_quick_reference.m.md](./visualization_quick_reference.m.md)

## システム上の位置づけ
このディレクトリは、MATLAB 側で学習したタイヤ力モデルを C++ シミュレータへ橋渡しする中核層です。生データの健全性確認から、学習、性能評価、可視化、C++ 用ルックアップテーブル生成までを一貫して担います。
