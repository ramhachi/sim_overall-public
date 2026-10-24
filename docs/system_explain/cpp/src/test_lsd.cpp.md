# `cpp/src/test_lsd.cpp`

## 1. ファイルの目的
Salisbury 型 LSD モデルの基本パラメータ、トルク配分、制限トルク、トルクバイアス比、実走行想定シナリオを段階的に確認する説明付きテストプログラムです。

## 2. 実行上の役割
- LSD モデル単体の理解と仕様確認を兼ねたサンプルです。
- モデル変更時に、代表条件で配分傾向が変わっていないかを見る回帰確認にも使えます。
- パワートレイン API の利用例としても役立ちます。

## 3. 依存関係
- `fsae/physics/powertrain/lsd.hpp`
- `fsae/physics/powertrain/engine.hpp`
- `SalisburyLSD`, `SalisburyLSD::Params`
- `SalisburyLSD::Params::fsae_traction_focused()`
- `SalisburyLSD::Params::fsae_balanced()`
- `SalisburyLSD::Params::fsae_handling_focused()`
- `lsd.distribute_torque()`, `lsd.compute_locking_torque()`, `lsd.compute_bias_ratio()`
- `create_cbr600rr_engine()`, `GearRatios::cbr600rr_default()`
- 標準ライブラリ `iostream`, `iomanip`, `cmath`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 主に `fsae_traction_focused()` を使用
  - トルク配分テスト: 3 速, 10000 rpm, スロットル 80%
  - 差動制限トルク: 入力トルク `100`〜`1000 Nm`
  - バイアス比較: 3 種プリセット
  - 実走行シナリオ: 2 速, 9000 rpm, スロットル 60%〜100%
- 出力:
  - 標準出力に各セクションの説明と数表
  - 外部ファイルなし
  - 例外発生時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `print_section(const std::string& title)`
- セクション見出しを標準出力へ整形表示する補助関数です。
- 60 文字の `=` 線で区切り、後続テストの読みやすさを上げます。

### `test_lsd_basic()`
- LSD 基本設定の表示用関数です。
- `fsae_traction_focused()` で LSD を生成し、以下を表示します。
  - 加速側ランプ角
  - 減速側ランプ角
  - プリロード
  - 有効半径
  - 摩擦係数
  - プレート数
- 確認内容:
  - プリセット値が想定通りに入っているか

### `test_torque_distribution()`
- LSD のトルク配分を 3 シナリオで確認します。
- セットアップ:
  - LSD: トラクション重視
  - エンジン: CBR600RR
  - ギア比: デフォルト
  - 条件: 3 速, 10000 rpm, 80% スロットル
- シナリオ:
  1. 直進加速: 左右同速で 50:50 相当になるか
  2. 旋回加速: 内輪が 10% 遅いとき、外輪にもトルクが乗るか
  3. 片輪スリップ: 片輪が 2 倍速のとき、グリップ輪へ多く配分されるか

### `test_locking_torque()`
- 入力トルクと差動制限トルクの関係を表示します。
- `omega_diff=5 rad/s` を固定し、`100`〜`1000 Nm` の入力で
  - 加速時 `compute_locking_torque(+torque, ...)`
  - 減速時 `compute_locking_torque(-torque, ...)`
  を比較します。
- 確認内容:
  - ランプ角差により加速側の制限が強くなること

### `test_bias_ratio()`
- LSD セッティング違いでトルクバイアス比を比較します。

#### `test_bias_ratio()` 内 `struct TestSetup`
- メンバ:
  - `std::string name`
  - `SalisburyLSD::Params params`
- 役割:
  - 比較名とパラメータをペアで保持するローカル構造体

#### `test_bias_ratio()` の処理
- 比較対象:
  - トラクション重視
  - バランス型
  - ハンドリング重視
- 条件:
  - `input_torque=500 Nm`
  - `omega_inner=45`
  - `omega_outer=50`
- `compute_bias_ratio()` の結果を表示します。

### `test_real_world_scenario()`
- 低速コーナー立ち上がりを模した簡易シナリオです。
- 条件:
  - 2 速
  - 9000 rpm
  - 内輪 5% 遅い、外輪 5% 速い
  - スロットル 60%→100%
- 各スロットルで
  - エンジントルク
  - 総駆動トルク
  - 内外輪トルク
  を表示します。
- 確認内容:
  - スロットル増加に対して LSD がどのように配分するか

### `main()`
- このファイルの入口です。
- `try` ブロック内で以下を順に実行します。
  1. `test_lsd_basic()`
  2. `test_torque_distribution()`
  3. `test_locking_torque()`
  4. `test_bias_ratio()`
  5. `test_real_world_scenario()`
- 例外時は内容を表示して `1` を返します。

### このファイル内で新規定義される型・補助関数
- 補助関数:
  - `print_section`
- 関数:
  - `test_lsd_basic`
  - `test_torque_distribution`
  - `test_locking_torque`
  - `test_bias_ratio`
  - `test_real_world_scenario`
  - `main`
- ローカル構造体:
  - `test_bias_ratio()` 内 `TestSetup`
