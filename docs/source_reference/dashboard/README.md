# `dashboard/`

FastAPI バックエンド、Jinja2 テンプレート、静的アセット（CSS/JS）、i18n 辞書を含むダッシュボード系の索引。

## 概要

FSAE 車両シミュレーションシステムのブラウザベース GUI。車両パラメータの閲覧・編集、エンジン特性可視化、シミュレーション実行、FSAE 2026 レギュレーション適合性チェック、タイヤモデル表示、パラメータエクスポートを提供する。

## ディレクトリ構造

```
dashboard/
├── __init__.py          # パッケージ初期化
├── app.py               # FastAPI バックエンド（メインモジュール）
├── templates/
│   └── index.html       # Jinja2 HTML テンプレート（SPA）
├── static/
│   ├── css/
│   │   └── style.css    # ダークテーマ CSS
│   └── js/
│       └── main.js      # クライアント側 JavaScript
└── i18n/
    ├── ja.json           # 日本語翻訳辞書
    └── en.json           # 英語翻訳辞書
```

## ファイル索引

| ファイル | 説明 |
|:---|:---|
| [`__init__.py.md`](__init__.py.md) | パッケージ初期化マーカー |
| [`app.py.md`](app.py.md) | FastAPI バックエンド。REST API・テンプレート描画・レギュレーションチェック |
| [`templates/index.html.md`](templates/index.html.md) | メイン HTML テンプレート（7 ページ SPA） |
| [`static/css/style.css.md`](static/css/style.css.md) | ダークテーマ CSS デザインシステム |
| [`static/js/main.js.md`](static/js/main.js.md) | Plotly グラフ描画・API 通信・SPA ナビゲーション |
| [`i18n/ja.json.md`](i18n/ja.json.md) | 日本語 UI 翻訳辞書 |
| [`i18n/en.json.md`](i18n/en.json.md) | 英語 UI 翻訳辞書（フォールバック） |

## 起動方法

```bash
python3 -m uvicorn dashboard.app:app --host 0.0.0.0 --port 8050 --reload
```

## 技術スタック

- **バックエンド**: FastAPI + Jinja2
- **フロントエンド**: Vanilla JS + Plotly.js (CDN)
- **スタイル**: カスタム CSS（ダークテーマ）
- **i18n**: JSON 辞書ファイル（日本語/英語）
