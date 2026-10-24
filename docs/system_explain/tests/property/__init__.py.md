# tests/property/__init__.py

## 役割
`tests.property` を property-based test 用パッケージとして束ねるだけの初期化ファイルです。

## 守っていること
- テスト収集時に `tests/property` が独立したテスト集合として扱えること。
- property 系テストの意図がパッケージ docstring で明示されていること。

## 主なフィクスチャ/ヘルパー
特別なフィクスチャや補助関数はありません。短い docstring のみです。

## アサーションの意味
このファイル自身にアサーションはありませんが、property テスト群の入口として配置されることで、配下の Hypothesis テストを安定して発見できる前提を支えています。
