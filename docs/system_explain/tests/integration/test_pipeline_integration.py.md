# tests/integration/test_pipeline_integration.py

## 役割
Docker コンテナ内で `./cpp/laptime_sim` を起動し、MATLAB 側が組み立てるラップタイム実行コマンドが通るかだけを確認する薄い統合テストです。

## 守っている振る舞い
- `ASE_autocross` 用トラック、短時間 `sim_time=5.0`、固定 PID ゲインを付けたコマンドを `docker compose run --rm fsae-sim` で実行できること。
- 出力先を `/workspace/opt_temp_test.csv` に固定してもプロセスが `CalledProcessError` なしで終了すること。

## 主なフィクスチャ/ヘルパー
- fixture はなく、テスト関数内で Docker 実行コマンド文字列を組み立てます。
- `cwd="/Users/sota/projects/FomulaTMU/sim_overall"` を明示してリポジトリ直下から実行します。

## 重要なアサーション・生成物
- 明示 `assert` は無く、`subprocess.run(..., check=True)` が唯一の成否判定です。
- 生成物はコンテナ内 `/workspace/opt_temp_test.csv` です。
- skip は無いため、Docker / build / track ファイルが無い環境ではそのまま失敗します。
