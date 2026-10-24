# `correlation/populate_correlation_config.py`

## 1. 役割

実車相関用 JSON 設定を読み、**入力完了率の診断** と **テンプレートベースの simulator params 生成** を行うツールです。

## 2. 入出力

- 実行例:
  - `python3 correlation/populate_correlation_config.py correlation/real_vehicle_correlation_config.json --check`
  - `python3 correlation/populate_correlation_config.py correlation/real_vehicle_correlation_config.json --export -o correlation/params_correlated.json`
- 入力:
  - 実車相関設定 JSON
  - `params_correlated_template.json`
- 出力:
  - readiness レポート
  - 相関用パラメータ JSON

## 3. 主要制御フロー

1. `main()` が `--check` と `--export` を解釈します。
2. `check_config()` で required/optional の充足率を集計します。
3. `export_params_run()` でテンプレート JSON に埋め戻します。
4. `--check` のみ、または `--check` 省略時も既定で readiness を表示します。

## 4. 関数

### `is_filled(value: Any) -> bool`
- 役割: 値が実質的に埋まっているか判定します。
- 判定:
  - `None` と空文字は未入力
  - list/dict は中身も再帰チェック
- 戻り値: 真偽値。

### `count_fields(obj: Any, prefix: str = "") -> list[dict]`
- 役割: `{"required": ..., "value": ...}` 形式の葉を全探索して一覧化します。
- 戻り値: `path`, `required`, `filled`, `method` を持つ辞書配列。

### `check_config(config: dict) -> dict`
- 役割: required/optional の総数、充足数、未入力パス一覧、readiness % を計算します。

### `field(config: dict, *keys: str) -> Any`
- 役割: ネスト辞書から値を取り出します。
- 特徴: 葉が `{"value": ...}` 形式なら中の `value` を返します。

### `assign_if_filled(container: dict, key: str, value: Any) -> None`
- 役割: 値が埋まっている時だけ出力辞書へ代入します。
- 副作用: `container` を破壊的更新します。

### `deg_list_to_rad(values: Any) -> Any`
- 役割: 度単位リストをラジアンへ変換します。
- 未入力値なら `None` を返します。

### `export_params_run(config: dict) -> dict`
- 役割: テンプレート JSON を読み、相関設定から simulator 用構造へ値を移植します。
- 主な変換:
  - コーナーウェイトから `m_total`, `a`, `b` を算出
  - `static_camber_deg`, `static_toe_deg` をラジアン化
  - タイヤ緩和長を `Lx/Ly` と `relaxation` の両方へ反映
  - 空力の `reference_density` を `rho` にも複写
- 副作用: テンプレート辞書を上書き更新します。

### `main() -> None`
- 役割: CLI 入口です。
- 挙動:
  - `--check` または `--export` なしで readiness を表示
  - `--export` 指定時は JSON を保存

## 5. 注意点

- export は「埋まっている項目だけをテンプレートへ反映」する方式です。
- 不足項目があっても `--export` 自体は進み得るため、通常は `--check` を先に見る前提です。
