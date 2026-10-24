# `cpp/src/main.cpp`

## 1. ファイルの目的
クォーターカー縦振動デモを実行し、Bouc-Wen ダンパを含む簡易サスペンション応答を CSV に出力する、最も小さな C++ 実行入口です。

## 2. 実行上の役割
- `cpp/fsae_sim` の主入口として動作します。
- フル車両ではなく 1 輪相当の上下系だけを高速に確認するためのサニティチェック用途を担います。

## 3. 依存関係
- `fsae/physics/suspension/bouc_wen.hpp`
- `Eigen`
- `spdlog`
- `std::filesystem`, `std::chrono`, `std::ofstream`
- `tmeasy.hpp`, `rosenbrock.hpp`, `rotation.hpp`, `nlohmann/json.hpp` を include していますが、このファイル内での主使用は Bouc-Wen と基本ユーティリティです。

## 4. 入力/出力
- 入力: CLI 引数は未使用。
- 出力:
  - `output/quarter_car/quarter_car_result.csv`
  - 標準ログに計算時間、最終状態、保存先を表示

## 5. 関数・メソッド・クラスの網羅的な説明

### `ensure_output_path(const std::string& filename)`
- `output/quarter_car` を必要に応じて作成し、保存先パス文字列を返します。
- 入力: 出力ファイル名
- 出力: `output/quarter_car/<filename>`

### `struct TimeSeriesData`
- クォーターカー応答の時系列格納構造体です。
- 保持メンバ:
  - `time`
  - `z_s`, `z_u`: ばね上/ばね下変位
  - `dz_s`, `dz_u`: ばね上/ばね下速度
  - `F_spring`, `F_damper`, `F_tire`
  - `z_road`

#### `TimeSeriesData::reserve(size_t n)`
- すべてのベクタに事前容量を確保します。

#### `TimeSeriesData::save_csv(const std::string& filename)`
- 現在保持している全時系列を CSV 化します。
- 出力列: `time,z_s,z_u,dz_s,dz_u,F_spring,F_damper,F_tire,z_road`

### `run_quarter_car_demo()`
- ファイルの本体処理です。
- 主な流れ:
  1. ばね上/ばね下質量、ばね、タイヤ、Bouc-Wen パラメータを設定
  2. 状態 `x=[z_s,z_u,dz_s,dz_u,z_bw]` を初期化
  3. 0.2 秒で 20 mm のステップ路面入力を与える
  4. 力を計算しながら 1 ms 刻みで記録
  5. ローカルラムダ `dynamics` で状態微分を定義
  6. その `dynamics` を用いて RK4 積分
  7. CSV 保存

#### ローカルラムダ `dynamics`
- 入力: 任意の 5 次元状態ベクトル
- 出力: その時点の状態微分
- 役割: RK4 の `k1`〜`k4` 計算に使う局所 ODE 評価器です。

### `main(int argc, char** argv)`
- ログレベル設定後に `run_quarter_car_demo()` を呼ぶだけの薄い入口です。
- `argc`, `argv` は使いません。

