# `dashboard/i18n/en.json`

> **要約**: ダッシュボード UI の英語翻訳辞書。日本語版 (`ja.json`) のフォールバックとしても機能する。

## ファイルの目的

ダッシュボードの英語表示に使用する翻訳テキストを JSON 形式で管理する。`app.py` の `load_i18n("en")` で読み込まれ、指定言語の辞書が存在しない場合のデフォルトとしても使われる。

## 構造

`ja.json` と同一のキー構造を持ち、値のみ英語で記述されている。

| キー | 内容例 |
|:---|:---|
| `app_title` | `"FSAE Vehicle Simulation Dashboard"` |
| `nav.engine` | `"Engine Curve"` |
| `simulation.scenarios.step_steer` | `"Step Steer (stability test)"` |
| `regulation.status_pass` | `"PASS"` |

## フォールバック動作

`load_i18n()` で指定言語の JSON が存在しない場合、`en.json` が代わりに読み込まれる。

## 入出力

- **入力**: `app.py` の `load_i18n()` で読み込み
- **出力**: Jinja2 テンプレートおよびクライアント JS 経由で UI に反映

## システム上の役割

i18n システムの英語リソースファイルおよびフォールバックデフォルト。
