# tire_fitting/tmeasy_model.py

## 役割
旧来の TMeasy 定常力モデル実装です。縦力と横力をベクトル化して計算します。

## 主なクラス
- `TMeasyModel`

## 主なメソッド
- `_compute_load_dependent_friction()`
- `_compute_saturation_function()`
- `compute_longitudinal_force()`
- `compute_lateral_force()`

## 背景・理論
TMeasy は Pacejka より少数パラメータで、荷重感度と飽和関数を用いてタイヤ力を近似する半物理モデルです。縦方向は `xi_bar = Kx*SR/(mu_x*Fz)`、横方向は `eta_bar = Ky*tan(SA)/(mu_y*Fz)` を用いて飽和を表現します。

## 使用ライブラリ
- `numpy`

## 入出力
- 入力: SR/SA、`Fz`、キャンバー、TMeasy パラメータ
- 出力: `Fx` または `Fy`

## パイプライン上の位置
旧 API と一部テストが依存するベースモデルです。
