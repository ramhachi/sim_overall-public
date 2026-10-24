# rotation.hpp

## ファイルの目的
- SAE J670e 座標系で使うオイラー角と回転行列の最小ユーティリティを提供します。

## 実行上の役割
- 車体姿勢から座標変換行列を作り、重力投影や角速度→オイラー角速度変換に使われます。
- すべてヘッダ内テンプレート実装なので、その場でインライン評価されます。

## 依存関係
- 外部: `Eigen/Dense`, `<cmath>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: `EulerAngles<Scalar>`、角速度、重力加速度
- 出力: `Eigen::Matrix<Scalar,3,3>` または `Eigen::Matrix<Scalar,3,1>`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> struct EulerAngles`
  - `phi`, `theta`, `psi` を保持する姿勢表現です。
  - 並びは Z-Y-X（Yaw-Pitch-Roll）で、`phi` がロール、`theta` がピッチ、`psi` がヨーです。
- `euler_to_rotation_matrix(const EulerAngles<Scalar>& euler)`
  - オイラー角から「慣性座標系→車両座標系」の 3x3 回転行列を構築します。
  - 各軸の `sin/cos` を事前計算し、SAE 座標規約に合わせた行列を返します。
- `angular_velocity_to_euler_rate(const EulerAngles<Scalar>& euler)`
  - 車体角速度 `[p,q,r]^T` をオイラー角速度 `[phi_dot, theta_dot, psi_dot]^T` に変換する行列 `E` を返します。
  - `tan(theta)` と `1/cos(theta)` を使うため、ピッチが ±90° に近いと特異になりやすい点に注意が必要です。
- `gravity_in_body(const EulerAngles<Scalar>& euler, Scalar g = 9.81)`
  - 慣性系で `[0,0,g]` と置いた重力を車体系へ回転して返します。
  - この実装では SAE 規約に合わせて「+Z が下向き」です。
