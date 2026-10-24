# `cpp/src/main_laptime_sim.cpp`

> **要約**: トラック、速度プロファイル、ドライバー、14DOF 車両を統合してフル過渡ラップタイムシミュレーションを行う入口。
> **役割**: 単体の車両応答ではなく、コース全体の走行性能を評価するための中心実行ファイル。

## このファイルが担うこと

シミュレーション条件を `SimulationConfig` にまとめ、  
トラック生成、PID/RL ドライバー選択、車両初期化、周回結果の集計までを一貫して処理する。

## 理論的背景

ラップタイム評価では、車両単体の DOF だけではなく、

- コース中心線
- Frenet 座標
- 曲率ベースの速度計画
- ドライバー制御器

が同時に必要になる。  
このファイルは、それらを 1 つの実験として接続する**統合実験ハーネス**である。

## 主要シンボル

| 種別 | 名前 | 説明 |
|:---|:---|:---|
| `struct` | `SimulationConfig` | トラック、時間刻み、ドライバー、ゲイン、パラメータ注入条件を持つ。 |
| `struct` | `LapResult` | 周回時間や横偏差などの要約結果を持つ。 |
| `function` | `resolve_autocross_csv` | オートクロス CSV の実パスを決める。 |
| `function` | `create_track` | トラック種別や CSV からコースを生成する。 |
| `function` | `select_gains` | PID ゲインプリセットまたはカスタム値を選ぶ。 |
| `function` | `run_laptime_simulation` | ラップタイムシミュレーション本体を実行する。 |

## 依存関係と使用ライブラリ

- `fsae/driver/pid_driver.hpp`
- `fsae/driver/rl_driver.hpp`
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/track/track_model.hpp`
- `fsae/track/waypoint_loader.hpp`
- `fsae/laptime/velocity_profile.hpp`
- `<filesystem>`

## 入出力と使われ方

- **入力**: トラック種別、車両パラメータ、ドライバーモード、PID ゲイン、JSON 出力指定
- **出力**: ラップタイム、偏差、速度統計、必要に応じた JSON / CSV

## 初学者向けの読み方

1. `SimulationConfig` を読んで切り替え可能な項目を把握する  
2. `create_track()` を読んでコース定義の流れを理解する  
3. `run_laptime_simulation()` で統合フローを追う
