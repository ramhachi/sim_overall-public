# `dashboard/app.py`

## 1. 役割

`dashboard/app.py` は FSAE ダッシュボードの **FastAPI バックエンド本体** です。  
エンジン情報表示、車両パラメータの取得・更新、簡易レギュレーション判定、簡易タイヤカーブ生成、C++ シミュレーション起動、各種エクスポート API を 1 つのサーバーに集約しています。

## 2. 実行位置と入出力

- 起動例: `python3 -m uvicorn dashboard.app:app --host 0.0.0.0 --port 8050 --reload`
- 主な入力:
  - `dashboard/i18n/*.json`
  - `data/vehicle/engine_cbr600rr.json`
  - ブラウザからの JSON リクエスト
  - `build/cpp/fsae_14dof` バイナリ
- 主な出力:
  - HTML (`/`)
  - JSON API 応答
  - C++ バイナリ標準出力の要約
  - JSON/YAML/C++ ヘッダ文字列

## 3. 主要制御フロー

1. 起動時にプロジェクト基準パス、静的ファイル、Jinja テンプレートを初期化します。
2. `DEFAULT_VEHICLE_PARAMS` を深いコピーして `_current_params` に保持し、UI からの編集状態をメモリ上で持ちます。
3. `/api/engine` ではエンジン JSON 読み込みとパワーカーブ計算を行います。
4. `/api/params` 系では車両パラメータを取得・更新・初期化します。
5. `/api/regulation` では `FSAE_2026_RULES` を順番に評価して合否一覧を返します。
6. `/api/tire` では簡易 TMeasy/Pacejka 風のタイヤ力カーブを生成します。
7. `/api/simulate` では C++ 実行ファイルを起動し、長い標準出力を先頭 10 行 + 末尾 30 行へ要約して返します。
8. `/api/export` では現在パラメータを JSON/YAML/C++ ヘッダへ変換します。

## 4. 定数・状態

- `BASE_DIR`, `PROJECT_ROOT`, `DATA_DIR`, `VEHICLE_DATA`, `ENGINE_JSON`, `I18N_DIR`  
  ファイル探索の基準です。
- `DEFAULT_VEHICLE_PARAMS`  
  UI の編集初期値です。質量、慣性、寸法、サスペンション、タイヤ、空力、パワートレイン、フレームをネスト辞書で保持します。
- `_current_params`  
  現在セッションで編集中の値です。永続化はせず、API プロセスのメモリ上にのみ存在します。
- `FSAE_2026_RULES`  
  ルール ID、カテゴリ、説明、判定関数、表示用値関数、要求値を持つルール配列です。
- `SIM_BINARY`  
  `build/cpp/fsae_14dof` を指し、シミュレーション API が参照します。

## 5. 主要関数

### `load_i18n(lang: str = "en") -> dict`

- 目的: 指定言語の翻訳 JSON を読み込みます。
- 引数: `lang` は言語コードです。
- 戻り値: 翻訳辞書。
- 副作用: `_i18n_cache` にキャッシュを書き込みます。指定言語が無い場合は `en.json` にフォールバックします。

### `load_engine_data() -> dict`

- 目的: `engine_cbr600rr.json` を読み込みます。
- 戻り値: エンジン辞書。
- 副作用: ファイルが無い場合は `_default_engine_data()` を使います。

### `_default_engine_data() -> dict`

- 目的: エンジン JSON 不在時のフォールバック値を返します。
- 戻り値: CBR600RR 想定の固定データ。
- 副作用: なし。

### `run_regulation_check(params: dict) -> List[dict]`

- 目的: `FSAE_2026_RULES` を全件評価します。
- 引数: 車両パラメータ辞書。
- 戻り値: 各ルールの `id/category/rule/passed/value/required` を持つ配列。
- 副作用: ルール関数内で例外が出た場合、該当ルールは fail 扱いになり、エラー文が `value` に入ります。

### `compute_tire_forces(mu_max: float = 1.6, Fz: float = 1000.0, n_points: int = 100) -> dict`

- 目的: ダッシュボード表示用の簡易タイヤ曲線を生成します。
- 入力: 最大摩擦係数、鉛直荷重、サンプル点数。
- 戻り値: `slip_ratios`, `fx_values`, `slip_angles_deg`, `fy_values`, `Fz` を含む辞書。
- 副作用: なし。物理モデルというより可視化用近似です。

### `compute_power_curve(engine_data: dict) -> dict`

- 目的: トルクカーブからパワーカーブを計算します。
- 入力: `engine.torque_curve` を含む辞書。
- 戻り値: 回転数、トルク、出力 kW の辞書。
- 副作用: なし。

### `run_simulation(scenario: str, params: dict) -> dict`

- 目的: C++ 14DOF 実行バイナリを起動します。
- 入力: `scenario` と現在パラメータ。
- 戻り値: `success`, `output`, `stderr`, `total_lines` などを含む辞書。
- 副作用:
  - `subprocess.run()` で `build/cpp/fsae_14dof` を実行します。
  - タイムアウトは 120 秒です。
  - 実際には `params` をバイナリへ渡しておらず、現状はシナリオ指定のみです。

### `export_params_json(params: dict, include_engine: bool, include_tire: bool) -> str`

- 目的: パラメータを JSON 文字列へ変換します。
- 挙動: `include_engine=True` ならエンジン情報を追加し、`include_tire=False` なら `tire` セクションを削除します。

### `export_params_yaml(params: dict, include_engine: bool, include_tire: bool) -> str`

- 目的: パラメータを YAML 文字列へ変換します。
- 戻り値: YAML 文字列。`PyYAML` が未導入なら案内コメント文字列を返します。
- 副作用: 実ファイル保存はしません。

### `export_params_cpp(params: dict) -> str`

- 目的: ネスト辞書を C++ `constexpr` 群へ平坦化してヘッダ文字列を生成します。
- 挙動:
  - 辞書は再帰的にフラット化
  - 配列は `constexpr double name[] = {...};`
  - 数値は `constexpr double name = ...;`
- 副作用: 生成時刻をコメントに埋め込みます。

## 6. API エンドポイント

### `GET /`
- `index()` が担当。
- `lang` クエリで翻訳辞書を読み、`index.html` を返します。

### `GET /api/engine`
- `api_engine()` が担当。
- エンジン基本情報と `compute_power_curve()` の結果を返します。

### `GET /api/params`
- `_current_params` をそのまま返します。

### `POST /api/params`
- リクエスト JSON 全体で `_current_params` を置き換えます。

### `POST /api/params/reset`
- `DEFAULT_VEHICLE_PARAMS` に戻します。

### `GET /api/regulation`
- レギュレーション一覧と全体合格フラグを返します。

### `GET /api/tire`
- `Fz` クエリを受け取り、簡易タイヤ曲線を返します。

### `POST /api/simulate`
- `scenario` を受け取り、C++ シミュレーション要約を返します。

### `GET /api/export`
- `fmt=json|yaml|cpp_header` と include フラグでエクスポート内容を返します。

### `GET /api/i18n/{lang}`
- 翻訳辞書そのものを返します。

## 7. 注意点

- パラメータ更新はメモリ上のみで、サーバー再起動で失われます。
- シミュレーション API は現在 `params` を C++ 側へ反映しません。
- レギュレーションの一部は「設計仕様上 true を返すダミー評価」で、幾何詳細の実計算ではありません。
