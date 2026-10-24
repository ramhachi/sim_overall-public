# `dashboard/static/js/main.js`

> **要約**: FSAE ダッシュボードのクライアント側 JavaScript。API 通信、Plotly グラフ描画、SPA ナビゲーション、パラメータ編集を実装する。

## ファイルの目的

`app.py` が提供する REST API と通信し、エンジン特性・車両パラメータ・タイヤ特性・シミュレーション結果・レギュレーションチェック結果をブラウザ上で動的に表示・操作するクライアントロジック。

## 理論的背景

シミュレーション設定 UI では、ページ遷移よりも**その場で値を変えて結果を確認する操作感**が重要になる。このファイルは、その即時性を実現する層である。

## 主要関数

| 関数 | 説明 |
|:---|:---|
| `goPage(name)` | SPA ページ遷移。CSS クラスで表示/非表示を切り替え、遅延データロード |
| `switchLang(lang)` | 言語切り替え（ページリロード） |
| `loadEngineData()` | `GET /api/engine` → Plotly でトルク/パワーの二軸グラフ描画 |
| `loadVehicleParams()` | `GET /api/params` → 動的フォーム生成 |
| `renderParamForm(params)` | パラメータセクションごとに入力フィールドを生成 |
| `saveParams()` | `POST /api/params` でパラメータ更新 |
| `resetParams()` | `POST /api/params/reset` でデフォルト復元 |
| `loadTireData()` | `GET /api/tire` → Fx vs SR, Fy vs SA のグラフ描画 |
| `runSim()` | `POST /api/simulate` → シミュレーション結果表示 |
| `runRegCheck()` | `GET /api/regulation` → チェック結果テーブル描画 |
| `previewExport()` / `downloadExport()` | `GET /api/export` → プレビュー表示 or ファイルダウンロード |

## Plotly 設定

グラフはダークテーマ統一で、以下の共通レイアウトを使用：
- 背景色: `#1a1d27`（カード背景と同一）
- フォント色: `#e0e0e0`
- グリッド: `#2a2d3a`

## 使用ライブラリ

| ライブラリ | 目的 |
|:---|:---|
| Plotly.js | インタラクティブなグラフ描画（CDN 読み込み） |
| ブラウザ Fetch API | バックエンド REST API との非同期通信 |

## テンプレート連携

`index.html` のレンダリング時に以下のグローバル変数が埋め込まれる：
- `T`: i18n 翻訳辞書オブジェクト
- `LANG`: 現在の言語コード

## 入出力

- **入力**: ユーザー操作（クリック、フォーム入力）、バックエンド API レスポンス
- **出力**: DOM 更新（グラフ、テーブル、フォーム）

## システム上の役割

ダッシュボードの SPA フロントエンドロジック。`app.py` の全 API エンドポイントのクライアント側実装を提供する。
