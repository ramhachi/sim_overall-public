# `matlab/pou_hardpoint_design/optimization/objective_polynomial_fit.m`

## 1. 目的

幾何学から得たジャッキアップ曲線と目標多項式との差を 1 つのスカラーコストへ変換する目的関数です。現実装ではジャッキアップ誤差のみを最適化対象にしています。

## 2. 入出力と依存関係

- 入力:
  - `design_vars`
  - `geom`
  - `target_poly`（`jack_c1/c2`, `camber_c1/c2`）
  - `delta_range`
- 出力:
  - `cost`: スカラー
- 依存関係:
  - `forward_kinematics`
  - `rad2deg`

## 3. 全体処理フロー

1. 重み `w_jack=1.0`, `w_camber=0.0` を設定します。
2. `forward_kinematics` で幾何学曲線を得ます。
3. 目標多項式から `jack_target`, `camber_target` を作ります。
4. ジャッキアップ誤差を mm、キャンバー誤差を deg に変換します。
5. ジャッキアップ RMS と最大誤差、キャンバー RMS を求めます。
6. `0.7 * rms_jack + 0.3 * max_jack` を主コストとして返します。

## 4. 関数詳細

### `objective_polynomial_fit(design_vars, geom, target_poly, delta_range)`

- **キャンバー重みが 0**  
  コメントどおり、固定 A アームでは POU がキャンバーを変えられない前提のため、`w_camber = 0.0` に固定されています。`camber_target` と `camber_error_deg` は計算するものの、コストへ寄与しません。

- **最大誤差を混ぜる理由**  
  RMS だけだと端部が崩れても平均化されやすいため、30% 分を最大ジャッキアップ誤差で補っています。

- **ローカル関数**  
  このファイルにローカル関数はありません。
