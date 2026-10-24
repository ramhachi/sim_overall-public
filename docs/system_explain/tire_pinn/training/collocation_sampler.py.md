# `tire_pinn/training/collocation_sampler.py`

## 1. 役割

物理損失用の **synthetic collocation points** を生成するサンプラです。  
実測 TTC データとは別に、ゼロスリップや複合スリップなどの拘束点を作ります。

## 2. 入出力

- 入力:
  - `sr/sa/fz/ia/tstc` の物理範囲
  - サンプル数
- 出力:
  - raw 物理単位の `torch.Tensor`

## 3. メソッド

### `sample_zero_slip(n_samples=500)`
- `SR=SA=0`、他はランダム
- `ZeroSlipLoss` 用

### `sample_symmetry(n_pairs=1000)`
- `SR=0, IA=0`、`SA` はランダム
- 呼び出し側が SA 符号反転して対を作る前提です

### `sample_friction_circle(n_samples=500, bias_combined=0.7)`
- 複合スリップ領域を厚めに取り、摩擦円違反が起きやすい点を多めに作ります

### `sample_monotonicity(n_samples=300)`
- 全入力空間に散らし、`requires_grad=True` で返します

### `sample_sa_zero_fy(n_samples=500)`
- `SA=0` 固定、SR は自由
- `FySaZeroLoss` 用です

### `sample_batch(...)`
- 上記をまとめて辞書で返します。
- キー: `zero`, `symmetry`, `friction`, `monotonicity`, `sa_zero_fy`

## 4. システム内の位置付け

`PINNTrainer` が各 epoch/batch でこのサンプラを呼び、  
`PhysicsLossManager` へ渡す物理拘束点を供給します。  
PINN らしさを支える学習補助器です。

## 5. 注意点

- 返す値は raw 物理単位です。
- `input_mean/std` を保持していますが、サンプリング自体は物理範囲ベースで行われます。
