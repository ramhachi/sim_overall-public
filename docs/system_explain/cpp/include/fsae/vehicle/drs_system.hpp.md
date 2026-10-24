# drs_system.hpp

## 1. ファイルの目的
DRS の状態遷移と安全判定を定義するヘッダです。

## 2. 実行上の役割
14DOF 車両モデルから呼ばれ、速度・横加速度・舵角に基づいて DRS の起動/停止を管理します。

## 3. 依存関係
- `vehicle_14dof_new.hpp`

## 4. 入力/出力
- 入力: DRS ボタン、速度、`vx`、ヨーレート、舵角、`dt`
- 出力: DRS が起動中かどうかの `bool`

## 5. 関数・メソッド・クラスの網羅的な説明
### `struct DRSConfig`
最低速度、最大横加速度、最大舵角、起動/停止遅れ、空力係数倍率を保持します。

### `class DRSSystem`
- `DRSSystem(const DRSConfig&)`
- `update(...)`: 状態機械を更新
- `is_active()`
- `struct Stats`: `total_active_time`, `activation_count`, `avg_speed_when_active`
- `get_stats()`
- `config()`
- `reset()`
- `enum class State`: `INACTIVE`, `ACTIVATING`, `ACTIVE`, `DEACTIVATING`
- `can_activate(...)`: 3 層安全判定 helper
