# `frame_optimize/onshape/create_test_model.py`

## 役割

- Onshape OAuth2 でログインし、**新規ドキュメント作成→Part Studio 特定→円スケッチ→押し出し**までを API 経由で自動実行するテストモデル生成スクリプトです。
- Onshape 連携の最小成功パスを確認するための実験コードです。

## 運用上の入口

### `main()`
- スクリプト実行時の標準入口です。
- `.env` から `ONSHAPE_CLIENT_ID` / `ONSHAPE_CLIENT_SECRET` を読み、ブラウザ認証を経て一連の API 呼び出しを進めます。

## 主要関数

### `load_env()`
- **入力**: 同ディレクトリの `.env`
- **出力**: Client ID / Secret
- **副作用**: 未設定環境変数を `.env` 値で補完

### `oauth_login(cid, csec)`
- **入力**: OAuth クライアント ID / Secret
- **出力**: アクセストークン
- **副作用**: `localhost:8080` に一時 HTTP サーバを立て、ブラウザを起動

### `create_document(token, name)`
- 新規 Onshape ドキュメントと既定ワークスペース ID を返します。

### `get_partstudio(token, did, wid)`
- ドキュメント内の Part Studio element ID を取得します。
- 型が揺れるケースでは最初の要素へフォールバックします。

### `eval_featurescript(...)` / `extract_transient_ids(result)`
- FeatureScript 実行と transient ID 取り出しを担当します。
- 平面・スケッチ領域の内部参照 ID 解決に使われます。

### `get_top_plane_id(...)`
- Top plane の transient ID を 3 通りの手段で探索します。
- **戻り値**: 見つかれば plane ID、失敗時は `None`

### `add_sketch(...)`
- 指定平面に円スケッチを追加します。
- **入力**: plane transient ID、半径 `radius_m`
- **出力**: sketch feature ID

### `get_sketch_region_id(...)`
- スケッチ領域の transient ID を取得します。

### `add_extrude(...)`
- スケッチ領域を `depth_mm` だけ押し出します。
- **出力**: extrude feature ID

## データフロー

1. `.env` 読込
2. OAuth ブラウザ認証でアクセストークン取得
3. 新規ドキュメント作成
4. Part Studio element ID 解決
5. Top plane transient ID 解決
6. 円スケッチ作成
7. スケッチ領域 ID 解決
8. 500 mm 押し出し
9. 完成ドキュメント URL を表示してブラウザで開く

## 入出力・副作用

- **入力**: `.env` の Onshape OAuth 資格情報
- **出力**: 標準出力ログ、Onshape 上の新規ドキュメント
- **副作用**:
  - ローカル HTTP サーバ起動
  - ブラウザ起動
  - Onshape 上へ新規モデル作成

## システム内での位置づけ

- `frame_optimize/onshape` 配下の API 実験群のうち、**作成系 API が一通り通るか**を確認する最小自動化スクリプトです。
- 後続のフレーム自動生成や cope cut 実験の基礎になります。
