#!/usr/bin/env python3
"""
Onshape API 接続テスト — OAuth2 ブラウザログイン方式

使い方:
  1. https://cad.onshape.com/appstore/dev-portal で OAuth Application を作成
     - Redirect URL: http://localhost:8080/callback
     - Permissions: Read your documents にチェック
  2. 環境変数 or .env を設定:
       ONSHAPE_CLIENT_ID=<your client id>
       ONSHAPE_CLIENT_SECRET=<your client secret>
  3. 実行:
       python test_connection.py
  4. ブラウザが開くのでOnshapeにログイン → 自動でトークン取得 → API呼び出し
"""

import http.server
import json
import os
import sys
import threading
import urllib.parse
import webbrowser
from pathlib import Path

try:
    import requests
except ImportError:
    print("ERROR: requests ライブラリが必要です")
    print("  pip install requests")
    sys.exit(1)

# --- 設定 ---
ONSHAPE_BASE_URL = "https://cad.onshape.com"
OAUTH_AUTHORIZE_URL = "https://oauth.onshape.com/oauth/authorize"
OAUTH_TOKEN_URL = "https://oauth.onshape.com/oauth/token"
REDIRECT_PORT = 8080
REDIRECT_URI = f"http://localhost:{REDIRECT_PORT}/callback"
SCOPES = "OAuth2Read"  # 読み取りのみ（接続テスト用）


def load_env():
    """環境変数 or .env ファイルから Client ID/Secret を取得"""
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, val = line.split("=", 1)
                os.environ.setdefault(key.strip(), val.strip())

    client_id = os.environ.get("ONSHAPE_CLIENT_ID", "")
    client_secret = os.environ.get("ONSHAPE_CLIENT_SECRET", "")
    return client_id, client_secret


class OAuthCallbackHandler(http.server.BaseHTTPRequestHandler):
    """OAuth2 リダイレクトを受け取るローカルHTTPサーバー"""

    auth_code = None

    def do_GET(self):
        parsed = urllib.parse.urlparse(self.path)
        params = urllib.parse.parse_qs(parsed.query)

        if parsed.path == "/callback" and "code" in params:
            OAuthCallbackHandler.auth_code = params["code"][0]
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(
                b"<html><body><h2>OK! Onshape auth complete.</h2>"
                b"<p>You can close this tab.</p></body></html>"
            )
        elif "error" in params:
            self.send_response(400)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            error_msg = params.get("error_description", params["error"])[0]
            self.wfile.write(f"<html><body><h2>Error: {error_msg}</h2></body></html>".encode())
        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # suppress log


def oauth_browser_login(client_id: str, client_secret: str) -> dict:
    """ブラウザでOnshapeログイン → トークン取得"""

    # 1. 認可URLを構築してブラウザを開く
    auth_params = urllib.parse.urlencode({
        "response_type": "code",
        "client_id": client_id,
        "redirect_uri": REDIRECT_URI,
        "scope": SCOPES,
    })
    auth_url = f"{OAUTH_AUTHORIZE_URL}?{auth_params}"

    print(f"  Opening browser for Onshape login...")
    print(f"  URL: {auth_url[:80]}...")

    # 2. ローカルサーバーを起動してリダイレクトを待つ
    server = http.server.HTTPServer(("localhost", REDIRECT_PORT), OAuthCallbackHandler)
    server.timeout = 120  # 2分タイムアウト

    webbrowser.open(auth_url)
    print(f"  Waiting for login callback on localhost:{REDIRECT_PORT}...")

    # コールバックを1回だけ処理
    OAuthCallbackHandler.auth_code = None
    server.handle_request()
    server.server_close()

    if not OAuthCallbackHandler.auth_code:
        raise RuntimeError("OAuth callback did not receive auth code (timeout or error)")

    auth_code = OAuthCallbackHandler.auth_code
    print(f"  Auth code received: {auth_code[:10]}...")

    # 3. auth code → access token に交換
    token_resp = requests.post(
        OAUTH_TOKEN_URL,
        data={
            "grant_type": "authorization_code",
            "code": auth_code,
            "client_id": client_id,
            "client_secret": client_secret,
            "redirect_uri": REDIRECT_URI,
        },
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )

    if token_resp.status_code != 200:
        raise RuntimeError(
            f"Token exchange failed ({token_resp.status_code}): {token_resp.text}"
        )

    token_data = token_resp.json()
    print(f"  Access token acquired (expires_in={token_data.get('expires_in', '?')}s)")
    return token_data


def test_api_calls(access_token: str):
    """Onshape APIの接続テスト"""
    headers = {
        "Authorization": f"Bearer {access_token}",
        "Accept": "application/json",
    }

    # Test 1: セッション情報（ユーザー確認）
    print("\n[Test 1] Session Info (GET /api/users/sessioninfo)")
    resp = requests.get(f"{ONSHAPE_BASE_URL}/api/users/sessioninfo", headers=headers)
    if resp.status_code == 200:
        info = resp.json()
        name = info.get("name", "?")
        email = info.get("email", "?")
        print(f"  OK: Logged in as {name} ({email})")
    else:
        print(f"  FAILED: {resp.status_code} {resp.text[:200]}")

    # Test 2: ドキュメント一覧（最新5件）
    print("\n[Test 2] Documents (GET /api/v10/documents)")
    resp = requests.get(
        f"{ONSHAPE_BASE_URL}/api/v10/documents",
        headers=headers,
        params={"limit": 5, "sortColumn": "modifiedAt", "sortOrder": "desc"},
    )
    if resp.status_code == 200:
        data = resp.json()
        items = data.get("items", [])
        print(f"  OK: {len(items)} documents found (showing up to 5)")
        for doc in items:
            print(f"    - {doc['name']} (id={doc['id'][:12]}...)")
    else:
        print(f"  FAILED: {resp.status_code} {resp.text[:200]}")

    # Test 3: API バージョン確認
    print("\n[Test 3] API Version Check")
    resp = requests.get(
        f"{ONSHAPE_BASE_URL}/api/v10/documents",
        headers=headers,
        params={"limit": 1},
    )
    print(f"  API endpoint: {ONSHAPE_BASE_URL}/api/v10/")
    print(f"  Response status: {resp.status_code}")
    if resp.status_code == 200:
        print(f"  API v10 is accessible")

def create_fishmouth(access_token: str, did: str, wid: str, eid: str, main_pipe_id: str, branch_pipe_id: str):
    """
    Onshape APIでフィッシュマウス（引き算）を作成する
    main_pipe_id: 刃（ツール）となる主軸パイプのID
    branch_pipe_id: 削られる（ターゲット）側の斜めパイプのID
    """
    url = f"{ONSHAPE_BASE_URL}/api/partstudios/d/{did}/w/{wid}/e/{eid}/features"
    headers = {
        "Authorization": f"Bearer {access_token}",
        "Content-Type": "application/json",
        "Accept": "application/vnd.onshape.v1+json"
    }

    # Onshape独特のフィーチャー定義JSON (ブール演算: 引き算)
    payload = {
        "feature": {
            "type": 119,
            "typeName": "BTMFeature134",
            "message": {
                "featureType": "boolean",
                "name": "Fishmouth Cut (API)",
                "parameters": [
                    {
                        # 演算タイプ: SUBTRACT (引き算)
                        "type": 145, "typeName": "BTMParameterEnum145",
                        "message": { "parameterId": "booleanType", "value": "SUBTRACT" }
                    },
                    {
                        # ツールを維持する（主軸パイプが消えないようにする絶対必須の設定）
                        "type": 144, "typeName": "BTMParameterBoolean144",
                        "message": { "parameterId": "keepTools", "value": True }
                    },
                    {
                        # 刃（ツール）となるパイプ
                        "type": 148, "typeName": "BTMParameterQueryList148",
                        "message": {
                            "parameterId": "tools",
                            "queries": [{"type": 138, "typeName": "BTMIndividualQuery138", "message": {"deterministicIds": [main_pipe_id]}}]
                        }
                    },
                    {
                        # 削られる（ターゲット）パイプ
                        "type": 148, "typeName": "BTMParameterQueryList148",
                        "message": {
                            "parameterId": "targets",
                            "queries": [{"type": 138, "typeName": "BTMIndividualQuery138", "message": {"deterministicIds": [branch_pipe_id]}}]
                        }
                    }
                ]
            }
        }
    }

    print("\n[API] フィッシュマウス加工を実行中...")
    resp = requests.post(url, headers=headers, json=payload)
    
    if resp.status_code == 200:
        print("  OK: フィッシュマウス（えぐり）の生成に成功しました！")
    else:
        print(f"  FAILED: {resp.status_code}\n{resp.text}")
        
def main():
    print("=" * 50)
    print(" Onshape API Connection Test")
    print("=" * 50)
    print()

    # 1. 認証情報の読み込み
    load_env()
    client_id, client_secret = load_env()

    if not client_id or not client_secret:
        print("ERROR: ONSHAPE_CLIENT_ID / ONSHAPE_CLIENT_SECRET が未設定です")
        print()
        print("設定方法:")
        print("  1. https://cad.onshape.com/appstore/dev-portal でOAuth Appを作成")
        print(f"     Redirect URL: {REDIRECT_URI}")
        print("  2. 以下のいずれかで設定:")
        print(f"     a) .env ファイル ({Path(__file__).parent / '.env'}):")
        print("        ONSHAPE_CLIENT_ID=xxxxx")
        print("        ONSHAPE_CLIENT_SECRET=xxxxx")
        print("     b) 環境変数:")
        print("        export ONSHAPE_CLIENT_ID=xxxxx")
        print("        export ONSHAPE_CLIENT_SECRET=xxxxx")
        sys.exit(1)

    print(f"Client ID: {client_id[:8]}...")
    print()

    # 2. OAuth ブラウザログイン
    print("[Auth] OAuth2 browser login")
    try:
        token_data = oauth_browser_login(client_id, client_secret)
    except Exception as e:
        print(f"  ERROR: {e}")
        sys.exit(1)

    access_token = token_data["access_token"]

    # 3. API テスト
    print("\n" + "=" * 50)
    print(" API Connection Tests")
    print("=" * 50)
    test_api_calls(access_token)

    print("\n" + "=" * 50)
    print(" All tests complete!")
    print("=" * 50)





if __name__ == "__main__":
    main()
