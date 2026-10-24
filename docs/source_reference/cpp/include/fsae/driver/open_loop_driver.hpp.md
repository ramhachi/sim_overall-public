# open_loop_driver.hpp

- **目的**: 手動試験用の時系列入力ドライバー群を提供します。
- **主要要素**: `OpenLoopDriver`、`StepSteerDriver`、`RampSteerDriver`、`SineSteerDriver`、`DoubleLaneChangeDriver`、`AccelBrakeDriver`、`TimeSeriesDriver`。
- **理論背景**: 制御器ではなく規定マニューバを線形補間付きで再生します。
- **依存**: `vehicle_state.hpp`、`vector`、`algorithm`。
- **システム上の位置づけ**: サブシステム検証や ISO ライクな手動試験の入力生成源です。

