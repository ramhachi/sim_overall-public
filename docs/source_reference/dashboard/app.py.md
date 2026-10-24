# `dashboard/app.py`

> **要約**: FastAPI ベースのダッシュボードバックエンド。車両パラメータ管理、シミュレーション実行、レギュレーション確認、タイヤ力計算、パラメータエクスポートの API を提供する。
> **役割**: ブラウザからこのシステムに触るための運用入口であり、研究者向けの CLI 群を人向け UI に変換する。

## ファイルの目的

アプリケーション初期化、静的ファイル配信、Jinja2 テンプレート描画、i18n 読み込み、デフォルト車両パラメータ保持、FSAE 2026 レギュレーションチェック、C++ シミュレーションバイナリ呼び出し、パラメータエクスポート（JSON/YAML/C++ ヘッダー）をまとめたメインモジュール。

## 理論的背景

### タイヤ力計算
`compute_tire_forces()` は簡易 Pacejka（Magic Formula）近似でタイヤ力カーブを生成する：
- **縦力**: `Fx = Dx · sin(Cx · arctan(Bx · SR))`（B=12, C=1.5）
- **横力**: `Fy = -Dy · sin(Cy · arctan(By · SA))`（B=10, C=1.3）

これはダッシュボード表示用の簡易モデルであり、実際のシミュレーションでは C++ 側の TMeasy/PINN モデルを使用する。

### レギュレーションチェック
FSAE 2026 ルールに基づく 18 項目のパラメータ検証を実装。ホイールベース最小値、トラック比率、サスペンショントラベル、エンジン排気量、ロールオーバー安定性、フレーム管材規格等をチェックする。

## 主要シンボル

| 種別 | 名前 | 説明 |
|:---|:---|:---|
| `function` | `load_i18n(lang)` | 言語辞書を読み込み（キャッシュ付き） |
| `function` | `load_engine_data()` | `engine_cbr600rr.json` からエンジンデータ読み込み |
| `function` | `_default_engine_data()` | JSON 不在時の CBR600RR フォールバック値 |
| `variable` | `DEFAULT_VEHICLE_PARAMS` | 14-DOF モデルの初期パラメータ（C++ `vehicle_params.hpp` と同期） |
| `variable` | `FSAE_2026_RULES` | レギュレーションチェックルール 18 項目のリスト |
| `function` | `run_regulation_check(params)` | 全ルールをチェックし結果リストを返す |
| `function` | `compute_tire_forces(mu_max, Fz, n_points)` | 簡易タイヤ力カーブ計算 |
| `function` | `compute_power_curve(engine_data)` | トルク→パワー変換（P = T·ω） |
| `function` | `run_simulation(scenario, params)` | C++ `fsae_14dof` バイナリの subprocess 実行 |
| `function` | `export_params_json(...)` | JSON 形式エクスポート |
| `function` | `export_params_yaml(...)` | YAML 形式エクスポート |
| `function` | `export_params_cpp(params)` | C++ `constexpr` ヘッダー形式エクスポート |

## API エンドポイント

| メソッド | パス | 説明 |
|:---|:---|:---|
| `GET` | `/` | メインページ（HTML テンプレート） |
| `GET` | `/api/engine` | エンジンデータ + パワーカーブ |
| `GET` | `/api/params` | 現在の車両パラメータ |
| `POST` | `/api/params` | パラメータ更新 |
| `POST` | `/api/params/reset` | デフォルトに復元 |
| `GET` | `/api/regulation` | レギュレーションチェック実行 |
| `GET` | `/api/tire?Fz=` | タイヤ力カーブデータ |
| `POST` | `/api/simulate` | C++ シミュレーション実行 |
| `GET` | `/api/export?fmt=` | パラメータエクスポート |
| `GET` | `/api/i18n/{lang}` | 翻訳辞書取得 |

## 依存関係と使用ライブラリ

| ライブラリ | このファイルでの目的 |
|:---|:---|
| `FastAPI` | REST API と HTML 応答を簡潔に定義 |
| `Jinja2Templates` | HTML テンプレートにサーバ側データを埋め込み |
| `StaticFiles` | CSS/JS/画像の静的ファイル配信 |
| `subprocess` | C++ シミュレーションバイナリの外部プロセス実行 |
| `json` / `copy` | パラメータの読み込み・深いコピー |
| `math` | パワーカーブ計算（P = T·RPM·π/30000） |
| `yaml`（オプション） | YAML エクスポート（未インストール時はフォールバック） |
| `time` | C++ ヘッダーのタイムスタンプ生成 |

## 入出力

- **入力**: HTTP リクエスト、`data/vehicle/engine_cbr600rr.json`、UI 上のパラメータ、`build/cpp/fsae_14dof` バイナリ
- **出力**: HTML ページ、JSON API レスポンス、パラメータファイル

## 起動方法

```bash
python3 -m uvicorn dashboard.app:app --host 0.0.0.0 --port 8050 --reload
```

## 初学者向けの読み方

1. `app = FastAPI(...)` 周辺とパス定義を見る
2. `DEFAULT_VEHICLE_PARAMS` で車両モデルの構造を把握する
3. i18n とエンジンデータのローダーを確認する
4. `FSAE_2026_RULES` でレギュレーションチェックの仕組みを理解する
5. 最後に API エンドポイント定義へ進む
