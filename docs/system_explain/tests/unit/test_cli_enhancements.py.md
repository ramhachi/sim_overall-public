# tests/unit/test_cli_enhancements.py

## 役割
CLI の設定ファイル読み込みと引数マージ処理が、設定値の妥当性と優先順位を正しく扱うかを確認します。

## 守っている振る舞い
- 有効 YAML を `load_config_file()` が辞書として読み込めること。
- 存在しない設定ファイルで `FileNotFoundError`、壊れた YAML や不正な `model_long` / `model_lat` で `ValueError` になること。
- 空ファイルが空辞書として扱われること。
- `merge_config_with_args()` が CLI デフォルト値のときのみ設定ファイル値を採用し、明示 CLI 値を優先すること。
- `--model-long`、`--model-lat`、`--config` を持つ引数解析関数が存在すること。

## 主なフィクスチャ/ヘルパー
- 一時 YAML を各テストで直接生成します。
- `argparse.Namespace` を使って実 CLI 実行なしに引数状態を再現します。
- `/workspace` を `sys.path` へ追加して import を安定化しています。

## アサーションの意味
例外系アサーションは「設定ミスを静かに飲み込まない」ことを保証します。値優先順位の確認は、再現性のあるバッチ実行と手動上書きの両立を守るために重要です。
