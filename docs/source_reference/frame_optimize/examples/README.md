# frame_optimize/examples/

EasyStart から直接辿る実行スクリプト群です。現行運用で最も重要なのは `run_frame_optimization.m` ですが、これは今は **interactive wrapper** で、実処理本体は `../matlab/run_canonical_frame_optimization.m` にあります。

## ファイル

- [run_frame_optimization.m.md](./run_frame_optimization.m.md)

## このディレクトリの位置づけ

ここは「ユーザー操作に近い層」です。現在の `run_frame_optimization.m` は、目標重量や SolidWorks マクロ出力の有無を対話入力で受け取り、結果保存先だけ base workspace へ戻しつつ canonical API を呼び出します。
