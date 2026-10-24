# `cpp/src/test_14dof_rosenbrock.cpp`

## 1. ファイルの目的
14DOF 車両モデルに対して、標準時間刻み `1 ms` と高精度比較用 `0.1 ms` の結果差・計算時間差を測り、剛性系に対する Rosenbrock 法導入余地を評価する比較テストです。

## 2. 実行上の役割
- 現行 14DOF モデルが `1 ms` 刻みで十分かを確認するベンチマークです。
- 名前に Rosenbrock が入っていますが、このファイル自身では Rosenbrock ソルバ API を呼んでいません。
- より細かい時間刻みとの比較で、積分精度と計算コストを把握します。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/solver/rosenbrock.hpp`
- `VehicleParams`, `Vehicle14DOF_New`, `DriverInput`
- `model.step()`, `model.state()`, `model.set_initial_velocity()`
- `spdlog`
- `<chrono>`
- `<cmath>` の `std::abs`, `std::sqrt`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - 初速 `15 m/s`
  - `throttle=0.1`
  - `steering=0.1 rad`
  - 総時間 `3.0 s`
  - ケース1: `dt=1e-3`
  - ケース2: `dt=1e-4`
- 出力:
  - 標準ログに各ケースの計算時間、最終状態、誤差比較、コメント
  - 外部ファイルなし
  - 失敗時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. 14DOF 用パラメータを明示設定
  2. `DriverInput` を `throttle=0.1`, `steering=0.1` で準備
  3. `model_rk4` を `dt=1e-3` で 3 秒積分
  4. `model_rk4_fine` を `dt=1e-4` で 3 秒積分
  5. 両ケースの実行時間を `chrono` で計測
  6. 最終 `x, y, vx, phi` を表示
  7. 位置差 `err_pos`、速度差 `err_vx` を算出
  8. `err_pos < 0.1 m` を目安に妥当性を説明
  9. どのような剛性条件で Rosenbrock が有利になりそうかをログ出力

#### `main()` が比較しているもの
- ケース A: RK4 相当の通常刻み `1 ms`
- ケース B: より細かい `0.1 ms`
- 比較指標:
  - ステップ数
  - 実時間
  - 最終位置差
  - 最終速度差

#### 注意点
- `rosenbrock.hpp` は include されていますが、Rosenbrock 法の実呼び出しはありません。
- 実体は「1 ms で十分か」の精度・性能比較テストです。

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - RK4 `1 ms` ベース走行
  - RK4 `0.1 ms` 比較走行
  - 両者比較
