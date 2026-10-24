# `frame_optimize/onshape/test_cope_cut.py`

## 役割

- Onshape 上の既存パイプ同士に対し、ブール演算で**cope cut（端面えぐり / fishmouth）**を掛ける API 実験スクリプトです。
- FeatureScript ではなく Onshape 組み込み `boolean` feature を直接叩き、加工 API の成立性を確認します。

## 運用上の入口

### `main()`
- `.env` の OAuth 資格情報を読み、ブラウザ認証後に対象ドキュメントへ cope cut を試行します。
- ただし本ファイルは、`main_pipe_id` と `branch_pipe_id` を手元の ID に差し替えて使う前提です。

## 主要関数・クラス

### `load_env()`
- `.env` から Onshape OAuth 資格情報を読み込みます。

### `OAuthCallbackHandler`
- `localhost:8080/callback` を待ち受ける OAuth コールバックハンドラです。

### `oauth_browser_login(client_id, client_secret)`
- **出力**: トークン応答辞書
- **副作用**: ローカル HTTP サーバ起動、ブラウザ認証

### `get_document_versions(access_token, did, wid, eid)`
- **役割**: `serializationVersion` と `sourceMicroversion` を取得し、feature POST の JSON バージョン不整合を避けます。
- **戻り値**: `(serialization_version, source_microversion)`

### `create_boolean_cope_cut(...)`
- **入力**: アクセストークン、対象 did/wid/eid、主管 ID、支管 ID
- **出力**: 戻り値なし
- **副作用**: Onshape ドキュメントへ `boolean` feature を追加
- **処理**:
  - ドキュメントの現在版情報を取得
  - `operationType = SUBTRACTION` と `keepTools = True` を持つ payload を構築
  - `tools` と `targets` に deterministic ID を設定して POST

## データフロー

1. OAuth 資格情報読込
2. ブラウザログインでアクセストークン取得
3. 既存ドキュメントの version / microversion 取得
4. 手指定のパイプ ID を使って boolean subtract 実行
5. 結果 URL を表示

## 入出力・副作用

- **入力**: `.env`、固定の `TARGET_DID/WID/EID`、手指定のパイプ ID
- **出力**: 標準出力ログ
- **副作用**:
  - ブラウザ起動
  - ローカル HTTP サーバ起動
  - Onshape ドキュメントへ加工 feature 追加

## システム内での位置づけ

- フレーム CAD 自動生成の中でも、**接合端面の加工 API を成立させるための実験段階コード**です。
- 完全自動化パイプラインではなく、Onshape feature payload とバージョン整合の確認を担うプロトタイプとして使われます。
