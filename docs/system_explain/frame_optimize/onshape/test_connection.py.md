# `frame_optimize/onshape/test_connection.py`

## 役割

- Onshape OAuth2 認証と基本 REST API への到達性を確認する**接続テストスクリプト**です。
- 認証成功後にセッション情報、ドキュメント一覧、API v10 到達性を確認します。

## 運用上の入口

### `main()`
- 実行時に `.env` または環境変数からクライアント資格情報を読み、ブラウザ認証→API テストを順に実行します。

## 主要関数・クラス

### `load_env()`
- `.env` と環境変数から `ONSHAPE_CLIENT_ID` / `ONSHAPE_CLIENT_SECRET` を解決します。

### `OAuthCallbackHandler`
- OAuth リダイレクトを受け取るローカル HTTP ハンドラです。
- `/callback` で認可コードを保存し、簡易 HTML を返します。

### `oauth_browser_login(client_id, client_secret)`
- **入力**: OAuth 資格情報
- **出力**: トークン応答辞書
- **副作用**: `localhost:8080` サーバ起動、ブラウザ起動、Onshape OAuth フロー実行

### `test_api_calls(access_token)`
- **入力**: アクセストークン
- **出力**: 標準出力ログ
- **検証内容**:
  - `/api/users/sessioninfo`
  - `/api/v10/documents`（最新 5 件）
  - API v10 エンドポイント応答

### `create_fishmouth(...)`
- ブール演算 API によるフィッシュマウス加工関数です。
- このファイルでは `main()` から未使用ですが、接続確認から加工 API 実験へ発展させるための補助関数として同居しています。

## データフロー

1. `.env` / 環境変数から資格情報を取得
2. ブラウザ OAuth でアクセストークン取得
3. セッション情報 API を叩いてログイン確認
4. ドキュメント一覧 API を叩いて読み取り権限確認
5. API バージョン応答を確認

## 入出力・副作用

- **入力**: OAuth 資格情報
- **出力**: 標準出力ログ
- **副作用**:
  - ブラウザ起動
  - ローカル HTTP サーバ起動
  - Onshape API への GET/POST 通信

## システム内での位置づけ

- Onshape 連携の**最初の疎通確認ツール**です。
- `create_test_model.py` や `test_cope_cut.py` を実行する前に、認証設定と API 到達性が正しいかを切り分ける用途で使います。
