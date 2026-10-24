# `scripts/analysis/fix_yaml.py`

## 1. 役割

`output/tire_params/hoosier_18x6_r20.yaml` を読み直し、**NumPy 型を通常の Python 型へ変換して再保存** するワンショット補修スクリプトです。

## 2. 入出力

- 実行例: `python3 scripts/analysis/fix_yaml.py`
- 入力: `output/tire_params/hoosier_18x6_r20.yaml`
- 出力: 同じファイルへの上書き保存

## 3. トップレベルフロー

1. `yaml.unsafe_load()` で既存 YAML を読みます。
2. `convert_numpy()` で ndarray, integer, floating を再帰変換します。
3. 元ファイルへ `yaml.dump()` で上書きします。
4. 完了メッセージを表示します。

## 4. 関数

### `convert_numpy(obj)`
- 役割: NumPy 依存オブジェクトを YAML 化しやすい型へ落とします。
- 変換規則:
  - `np.ndarray -> list`
  - `np.integer`, `np.floating -> float`
  - `dict/list/tuple` は再帰変換
- 戻り値: 変換後オブジェクト

## 5. 注意点

- ファイルパスが固定で、引数は取りません。
- `unsafe_load` を使うため、信頼済みファイルを扱う前提です。
