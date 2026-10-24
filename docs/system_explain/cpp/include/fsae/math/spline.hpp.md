# spline.hpp

## ファイルの目的
- 1 次元 3 次スプラインと 2 次元バイキュービック相当補間を提供します。

## 実行上の役割
- テーブル値の滑らかな補間に使う数学ユーティリティです。
- 実装はヘッダ完結で、補間係数を内部保持して再利用します。

## 依存関係
- 外部: `<vector>`, `<stdexcept>`, `<cmath>`
- リポジトリ内依存: なし

## 入力/出力
- 入力: 単調増加 `x` 軸、対応する `y` または 2 次元グリッド `z`
- 出力: 補間値、1 階微分値

## 関数・メソッド・クラスの網羅的な説明
- `template<typename Scalar> class CubicSpline`
  - 1 次元自然 3 次スプラインです。
  - `CubicSpline()` はデフォルト構築のみです。
  - `build(const std::vector<Scalar>& x, const std::vector<Scalar>& y)`
    - 節点列から係数 `b_, c_, d_` を計算します。
    - 点数不一致や 2 点未満では `std::invalid_argument` を送出します。
    - 内部では `h`, `alpha`, `l`, `mu`, `z` を使うトーマス法で二階微分を解きます。
  - `operator()(Scalar x) const`
    - 補間値評価です。
    - `build()` 前なら `std::runtime_error` を送出します。
  - `derivative(Scalar x) const`
    - 同じ区間係数を用いて 1 階微分を返します。
  - `find_interval(Scalar x) const`
    - private 補助関数です。
    - 範囲外入力は端区間にクランプし、範囲内は二分探索で区間を返します。
  - private メンバ `x_`, `y_`, `b_`, `c_`, `d_`, `n_`, `built_`
    - 節点、係数、点数、構築済みフラグを保持します。
- `template<typename Scalar> class BicubicSpline`
  - 各 `y` 行で `CubicSpline` を作り、問い合わせ時に列方向でもう一度 `CubicSpline` を組む実装です。
  - `build(const std::vector<Scalar>& x, const std::vector<Scalar>& y, const std::vector<std::vector<Scalar>>& z)`
    - `z[j]` を各 `y[j]` 行データとして行方向スプラインを構築します。
  - `operator()(Scalar x, Scalar y) const`
    - まず各行で `x` 補間し、その結果を使って `y` 方向スプラインを都度生成して値を返します。
    - `build()` 前なら `std::runtime_error` を送出します。
  - private メンバ `x_`, `y_`, `row_splines_`, `nx_`, `ny_`, `built_`
    - グリッド軸、行スプライン群、サイズ、構築状態を持ちます。
