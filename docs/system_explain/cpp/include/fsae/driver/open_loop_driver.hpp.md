# open_loop_driver.hpp

## 1. ファイルの目的
時間ベースで決め打ち入力を返すオープンループドライバー群をまとめたヘッダです。

## 2. 実行上の役割
閉ループ制御なしで基準入力を生成し、ステップ応答やレーンチェンジなどの試験マニューバを提供します。

## 3. 依存関係
- `vehicle/vehicle_state.hpp`

## 4. 入力/出力
- 入力: 時刻 `t`
- 出力: `DriverInput`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct TimeSeriesPoint`
`time`, `steering`, `throttle`, `brake`, `gear` を持つ時系列 1 点です。

### `class OpenLoopDriver`
抽象基底です。`~OpenLoopDriver()`, `get_input(double)`, `duration()` を宣言します。

### `lerp(...)`
線形補間 helper です。

### `class StepSteerDriver`
ステップ舵角入力。コンストラクタ、`get_input(...)`, `duration()` を持ちます。

### `class RampSteerDriver`
ランプ操舵入力。コンストラクタ、`get_input(...)`, `duration()` を持ちます。

### `class SineSteerDriver`
正弦波操舵入力。コンストラクタ、`get_input(...)`, `duration()` を持ちます。

### `class DoubleLaneChangeDriver`
ダブルレーンチェンジ入力。コンストラクタ、`get_input(...)`, `duration()` を持ちます。

### `class AccelBrakeDriver`
加速→惰行→制動入力。コンストラクタ、`get_input(...)`, `duration()` を持ちます。

### `class TimeSeriesDriver`
任意時系列を補間するドライバーです。コンストラクタ、`get_input(...)`, `duration()` を持ちます。
