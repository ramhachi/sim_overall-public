# `cpp/src/test_14dof_new.cpp`

## 1. ファイルの目的
`Vehicle14DOF_New` の総合動作確認を行うテスト実行ファイルです。Bouc-Wen ダンパ、LuGre 摩擦、ARB を含む 14DOF モデルを 5 秒間シミュレーションし、ロール角・旋回半径・荷重移動が概ね妥当かを確認します。

## 2. 実行上の役割
- 新 14DOF モデルの代表的なデモ兼スモークテストです。
- パラメータを明示設定したうえで、ステップ操舵時の応答を CSV とログで残します。
- モデル変更時の「全体として動いているか」の確認に使います。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `VehicleParams`, `Vehicle14DOF_New`, `DriverInput`
- `model.set_initial_velocity()`, `model.step()`, `model.state()`
- `model.get_tire_Fz()`, `model.get_tire_Fy()`
- `model.get_bw_forces()`, `model.get_lugre_forces()`
- `spdlog`
- `<fstream>`
- `<cmath>` の `std::abs`, `M_PI`

## 4. 入力/出力
- 入力:
  - CLI 引数なし
  - パラメータはファイル内で明示設定
  - 初速 `15 m/s`
  - `throttle=0.1`
  - `t>=1.0 s` で `steering=0.1 rad`
  - `dt=0.001 s`
  - 総時間 `5.0 s`
- 出力:
  - `14dof_result.csv`
  - 標準ログに静的荷重、初期荷重、最終状態、最終ダンパ力、妥当性判定
  - 判定失敗時は `return 1`

## 5. 関数・メソッド・クラスの網羅的な説明

### `main()`
- このファイル唯一の関数です。
- 主処理:
  1. `VehicleParams` に質量、Bouc-Wen、LuGre、ARB の値を設定
  2. `Vehicle14DOF_New model(params)` を生成
  3. `set_initial_velocity(15.0)` を実行
  4. `14dof_result.csv` を開き列名を書き込み
  5. 静的荷重と初期 `Fz` を表示
  6. 5 秒間積分し、10 ステップごとに状態と各輪垂直荷重を CSV 保存
  7. `max_phi`, 横加速度近似 `ay`, 旋回半径 `final_radius` を更新
  8. 最終状態と最終荷重、Bouc-Wen/LuGre 力を表示
  9. 妥当性判定を実施

#### `main()` が確認するシナリオ
- 初速付き定常加速に、1 秒後のステップ操舵を重ねた試験です。
- 14DOF モデル全体が統合状態で走り切るかを見ます。

#### `main()` の判定内容
- ロール角:
  - `0.0001° < max_phi_deg < 10.0°`
- 旋回半径:
  - `10 m < final_radius < 200 m`
- 荷重移動:
  - `(Fz右側合計 - Fz左側合計)` の絶対値が `0.1 N` を超えるか
  - この項目は warning 扱いで、失敗には直結させていません

#### CSV 列
- `time,x,y,psi,vx,vy,phi,theta,ax,ay,Fz_FL,Fz_FR,Fz_RL,Fz_RR`

### このファイル内で新規定義される型・補助関数
- 独自 `class` / `struct`: なし
- 補助関数: なし
- `TEST` 相当シナリオ:
  - `main()` 内の「初速付きステップ操舵 14DOF 総合試験」1 本
