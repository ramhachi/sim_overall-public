# frame_optimize/cpp/beam_fem_standalone.cpp

## 役割

JSON ファイル入出力だけで FEM を解く、単発実行用の CLI 版です。TCP サーバーを立てたくないときや、入力を固定して再現実験したいときに向いています。

## 入出力

- 入力: `input.json`
- 出力: `output.json`

どちらも `nodes`、`elements`、`bcs`、`loads` を持つ単純な JSON 形式です。

## サーバー版との違い

- 通信層がなく、1 回解いて終了する
- MATLAB EasyStart からの通常運用では使われない
- ただし FEM 本体の考え方はサーバー版とほぼ同じなので、デバッグ時の比較基準として読みやすい

## 実装メモ

到達不能節点を BFS で検出して拘束する点、Fz カップルからねじり剛性を推定できる点はサーバー版と共通です。
