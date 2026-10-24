# `tire_pinn/losses/physics_losses.py`

## 1. 役割

PINN に課す物理拘束群をまとめたモジュールです。  
ゼロスリップ、対称性、摩擦円、荷重単調性、`SA=0` での Fy リーク抑制を個別損失として定義します。

## 2. 主要クラス

### `ZeroSlipLoss(force_threshold=2.0)`
- 入力: `SR=SA=0` の collocation 点
- 目的: `Fx=Fy=Mz=0` を近づける
- 出力: 損失と `max_fx/max_fy/max_mz` など

### `SymmetryLoss()`
- 入力: `SR=0, IA=0` の SA 正負ペア
- 目的: `Fy(SA) = -Fy(-SA)` を促す
- 指標: 最大/平均非対称率

### `FrictionCircleLoss(mu_cap=2.6, ...)`
- 入力: 複合スリップ領域の collocation 点
- 目的: `sqrt(Fx² + Fy²) <= μFz`
- 指標: violation rate, max violation
- `set_normalization_params()` で統計を保持更新できます。

### `MonotonicityLoss(epsilon=1e-6)`
- 入力: `requires_grad=True` の collocation 点
- 目的: `∂|Fx|/∂Fz`, `∂|Fy|/∂Fz` の負側を罰する
- 特徴: `autograd.grad()` を使うため高価です

### `FySaZeroLoss(force_threshold=20.0)`
- 入力: `SA=0` だが SR は任意の collocation 点
- 目的: DriveBrake 条件などで `Fy ≈ 0` を保つ

### `PhysicsLossManager`
- 上記損失を束ね、重み付き合計を返します。
- `compute_all()` は各 collocation バッチと重み辞書を受け、`physics_loss_total` を生成します。

## 3. 重要パラメータ

- `weights` 辞書キー:
  - `zero`
  - `sym`
  - `fric`
  - `mono`
  - `fy_sa_zero`
- `MonotonicityLoss` は collocation が無い場合スキップ可能です。

## 4. データフローとシステム適合

`CollocationSampler` が物理拘束用入力を作り、  
`PINNTrainer` が epoch ごとの curriculum 重みを付けて `PhysicsLossManager` へ渡します。  
つまり「実データに無い物理常識」を学習へ注入する中心部です。

## 5. 副作用と注意点

- 副作用はありません。
- 一部損失は raw 物理単位の入力/出力を前提にしています。
- `MonotonicityLoss` は勾配計算コストが高く、検証時は通常省略されます。
