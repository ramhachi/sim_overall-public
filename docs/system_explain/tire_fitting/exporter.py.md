# `tire_fitting/exporter.py`

## 役割

旧来の TMeasy 単独フィット結果を YAML と C++ ヘッダへ出力するエクスポータです。  
人間可読な設定ファイルと、シミュレータへ組み込みやすい定数ヘッダの両方を生成します。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - フィット済みパラメータ辞書
  - 評価指標辞書
  - 出力ファイルパス
  - タイヤ名、有効半径、公称荷重
- 主な出力:
  - YAML ファイル
  - `.h` / `.hpp` 形式の C++ ヘッダ
- 副作用:
  - 親ディレクトリ自動作成
- 例外:
  - `ValueError`: NaN / Inf を含む値
  - `PermissionError`: 書込不可

## 主要クラス

### `ParameterExporter`

#### `export_yaml(params, metrics, filepath, tire_name, effective_radius, nominal_load, data_source)`
- TMeasy 専用の YAML 構造を生成します。
- `longitudinal` / `lateral` / `camber` / `fitting_info` を持ちます。
- `numpy` 型はネイティブ Python 型へ変換してから書き出します。
- `Kx`, `Ky` から参考用の緩和長 `Lx`, `Ly` も計算します。

#### `export_cpp_header(params, tire_name, filepath, effective_radius, nominal_load)`
- タイヤ名から struct 名と include guard を生成します。
- `static constexpr double` で TMeasy パラメータ一式を埋め込みます。

## データフロー

1. パラメータの有限値チェックを行います。
2. YAML 向けは辞書構造へ整形し、C++ 向けは文字列行配列へ整形します。
3. 出力先ディレクトリを作成し、ファイルを書き出します。

## システム内での位置づけ

`main.py` の最終段を担う **旧形式エクスポート層** です。  
`exporters/` 配下の hybrid 形式とは別に、TMeasy 単独パイプライン互換の成果物を維持しています。
