# mat_file_reader.hpp

## ファイルの目的
- MAT ファイルから NN タイヤ用配列を読むユーティリティを提供します。

## 実行上の役割
- `matio` を使って 1 次元配列、5 次元配列、スカラーを読み込み、C++ 側で扱いやすい形に変換します。
- RAII ラッパ `MatFile` によりクローズ漏れも防ぎます。

## 依存関係
- 外部: `<string>`, `<vector>`, `<array>`, `<stdexcept>`, `matio.h`
- リポジトリ内依存: NN タイヤ LUT 系から利用される想定

## 入力/出力
- 入力: MAT ファイルパス、変数名、期待次元
- 出力: `mat_t*`, `std::vector<double>`, `Array5D<double>`, `double`

## 関数・メソッド・クラスの網羅的な説明
- `template<typename T> class Array5D`
  - `Array5D()`
    - 空配列です。
  - `Array5D(size_t d0, size_t d1, size_t d2, size_t d3, size_t d4)`
    - 次元を保存し、全要素数で `data_` を確保します。
  - `operator()(size_t i0, size_t i1, size_t i2, size_t i3, size_t i4)`
    - 5 次元添字で可変参照を返します。
  - `operator() const`
    - const 参照版です。
  - `dimensions() const`
    - 5 次元サイズ配列を返します。
  - `size() const`
    - 総要素数です。
  - `data() const` / `data()`
    - 内部一次元ベクタ参照を返します。
  - private `compute_index(...) const`
    - MATLAB の列優先並びを前提にした一次元添字計算です。
- `class MatFileReader`
  - `MatFileReader()` / `~MatFileReader()`
    - 特別な処理は持たない静的ユーティリティ用の形です。
  - `static mat_t* open(const std::string& filepath)`
    - 読み取り専用で MAT ファイルを開き、失敗時は例外です。
  - `static void close(mat_t* matfp)`
    - `nullptr` を許容しつつ `Mat_Close` します。
  - `static std::vector<double> read_array_1d(mat_t* matfp, const std::string& varname)`
    - 実数 double 配列を読み、総要素数ぶんコピーして返します。
  - `static Array5D<double> read_array_5d(mat_t* matfp, const std::string& varname, const std::array<size_t,5>& expected_dims = {})`
    - 実数 double の 5 次元配列を読みます。
    - `expected_dims[0] != 0` のときは次元一致を検証します。
    - MATLAB のフラット列優先添字から各次元添字へ展開し、`Array5D` に再配置します。
  - `static double read_scalar(mat_t* matfp, const std::string& varname)`
    - 1 要素配列を読み、スカラーとして返します。
- `class MatFile`
  - `explicit MatFile(const std::string& filepath)`
    - `MatFileReader::open()` でファイルを開きます。
  - `~MatFile()`
    - 自動で `close()` します。
  - `MatFile(const MatFile&) = delete`, `operator=(const MatFile&) = delete`
    - コピー禁止です。
  - `MatFile(MatFile&& other) noexcept`
    - ムーブ時に所有権を移し、元の `matfp_` を `nullptr` にします。
  - `get() const`
    - 生の `mat_t*` を返します。
  - private `matfp_`
    - 所有する MAT ハンドルです。
