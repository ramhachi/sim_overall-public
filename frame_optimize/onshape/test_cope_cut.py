#!/usr/bin/env python3
"""
Onshape API — FSAE 端面加工 (Cope Cut) 自動生成 完全版
"""

import http.server
import os
import sys
import urllib.parse
import warnings
import webbrowser
from pathlib import Path

# macOS環境での urllib3 v2 の SSL警告をミュート
warnings.filterwarnings("ignore", category=UserWarning, module="urllib3")

try:
    import requests
except ImportError:
    print("ERROR: requests ライブラリが必要です (pip install requests)")
    sys.exit(1)

# --- 設定 ---
ONSHAPE_BASE_URL = "https://cad.onshape.com"
OAUTH_AUTHORIZE_URL = "https://oauth.onshape.com/oauth/authorize"
OAUTH_TOKEN_URL = "https://oauth.onshape.com/oauth/token"
REDIRECT_PORT = 8080
REDIRECT_URI = f"http://localhost:{REDIRECT_PORT}/callback"
SCOPES = "OAuth2Read OAuth2Write"

# ★あなたのドキュメントIDに書き換えてください★
TARGET_DID = "efe84e2ad8e0fa61c5e503df"
TARGET_WID = "ec7efd95fea8e860cc04e740"
TARGET_EID = "e2ebcfa919c62f9b4a830983"


def load_env():
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                key, val = line.split("=", 1)
                os.environ.setdefault(key.strip(), val.strip())
    return os.environ.get("ONSHAPE_CLIENT_ID", ""), os.environ.get(
        "ONSHAPE_CLIENT_SECRET", ""
    )


class OAuthCallbackHandler(http.server.BaseHTTPRequestHandler):
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
                b"<html><body><h2>OK! Onshape auth complete.</h2><p>You can close this tab.</p></body></html>"
            )
        else:
            self.send_response(400)
            self.end_headers()

    def log_message(self, format, *args):
        pass


def oauth_browser_login(client_id: str, client_secret: str) -> dict:
    auth_params = urllib.parse.urlencode(
        {
            "response_type": "code",
            "client_id": client_id,
            "redirect_uri": REDIRECT_URI,
            "scope": SCOPES,
        }
    )
    auth_url = f"{OAUTH_AUTHORIZE_URL}?{auth_params}"

    print("  Opening browser for Onshape login...")
    server = http.server.HTTPServer(("localhost", REDIRECT_PORT), OAuthCallbackHandler)
    server.timeout = 120
    webbrowser.open(auth_url)

    OAuthCallbackHandler.auth_code = None
    server.handle_request()
    server.server_close()

    if not OAuthCallbackHandler.auth_code:
        raise RuntimeError("認証コードが取得できませんでした。")

    token_resp = requests.post(
        OAUTH_TOKEN_URL,
        data={
            "grant_type": "authorization_code",
            "code": OAuthCallbackHandler.auth_code,
            "client_id": client_id,
            "client_secret": client_secret,
            "redirect_uri": REDIRECT_URI,
        },
        headers={"Content-Type": "application/x-www-form-urlencoded"},
    )
    if token_resp.status_code != 200:
        raise RuntimeError(f"トークン取得失敗: {token_resp.text}")
    print("  Token acquired\n")
    return token_resp.json()


def get_document_versions(access_token: str, did: str, wid: str, eid: str):
    """【最重要】現在のドキュメントのバージョン情報を取得する（JSONエラー回避の特効薬）"""
    url = f"{ONSHAPE_BASE_URL}/api/partstudios/d/{did}/w/{wid}/e/{eid}/features"
    headers = {
        "Authorization": f"Bearer {access_token}",
        "Accept": "application/vnd.onshape.v1+json",
    }
    resp = requests.get(url, headers=headers)
    if resp.status_code == 200:
        data = resp.json()
        return data.get("serializationVersion", "1.1.23"), data.get(
            "sourceMicroversion", ""
        )
    return "1.1.23", ""


def create_boolean_cope_cut(
    access_token: str,
    did: str,
    wid: str,
    eid: str,
    main_pipe_id: str,
    branch_pipe_id: str,
):
    """
    Onshape APIのビルトイン機能（boolean）を使用して端面加工を行う。
    FeatureScriptのnamespaceエラーを避けるため、最も堅牢なブール演算を使用します。
    """
    # 1. バージョン情報を動的に取得してエラーを防ぐ
    serialization_version, source_microversion = get_document_versions(
        access_token, did, wid, eid
    )

    url = f"{ONSHAPE_BASE_URL}/api/partstudios/d/{did}/w/{wid}/e/{eid}/features"
    headers = {
        "Authorization": f"Bearer {access_token}",
        "Content-Type": "application/json",
        "Accept": "application/vnd.onshape.v1+json",
    }

    # 2. 厳密なパラメータ定義（SUBTRACTION）
    payload = {
        "feature": {
            "type": 119,
            "typeName": "BTMFeature134",
            "message": {
                "featureType": "boolean",
                "name": "Auto Cope Cut (Boolean)",
                "parameters": [
                    {
                        "type": 145,
                        "typeName": "BTMParameterEnum145",
                        "message": {
                            "parameterId": "operationType",
                            "value": "SUBTRACTION",  # ← SUBTRACTではなくSUBTRACTIONが正解
                            "enumName": "BooleanOperationType",
                        },
                    },
                    {
                        "type": 144,
                        "typeName": "BTMParameterBoolean144",
                        "message": {"parameterId": "keepTools", "value": True},
                    },
                    {
                        "type": 148,
                        "typeName": "BTMParameterQueryList148",
                        "message": {
                            "parameterId": "tools",
                            "queries": [
                                {
                                    "type": 138,
                                    "typeName": "BTMIndividualQuery138",
                                    "message": {"deterministicIds": [main_pipe_id]},
                                }
                            ],
                        },
                    },
                    {
                        "type": 148,
                        "typeName": "BTMParameterQueryList148",
                        "message": {
                            "parameterId": "targets",
                            "queries": [
                                {
                                    "type": 138,
                                    "typeName": "BTMIndividualQuery138",
                                    "message": {"deterministicIds": [branch_pipe_id]},
                                }
                            ],
                        },
                    },
                ],
            },
        },
        "serializationVersion": serialization_version,
        "sourceMicroversion": source_microversion,
    }

    print("\n[8] Boolean Subtract (Cope Cut) を実行中...")
    resp = requests.post(url, headers=headers, json=payload)

    if resp.status_code == 200:
        print(
            "  OK: 完璧なフィッシュマウス加工が生成されました！画面を確認してください。"
        )
    else:
        print(f"  FAILED: Error {resp.status_code}")
        print(f"  {resp.text}")


def main():
    print("=======================================================")
    print(" Onshape: Cope Cut (端面加工) API Test")
    print("=======================================================\n")

    client_id, client_secret = load_env()
    if not client_id or not client_secret:
        print("ERROR: .env に Client ID / Secret が設定されていません。")
        sys.exit(1)

    try:
        print("[1] OAuth login")
        token_data = oauth_browser_login(client_id, client_secret)
        access_token = token_data["access_token"]

        # ---------------------------------------------------------
        # 【ここに あなたが成功させた Step 2 〜 7 のパイプ生成コードを貼り付けてください】
        # ---------------------------------------------------------
        print("[2-7] パイプ生成処理 (お手元のコードを実行)...")

        # ※テスト用ハードコード（すでにOnshape上にパイプが存在する場合）
        # ご自身のスクリプトで取得した変数名に書き換えてください。
        main_pipe_id = "JHD"  # ← 実際のログにあったツールパイプ(刃)のID
        branch_pipe_id = "JKD"  # ← 実際のログにあったターゲットパイプ(削られる側)のID
        # ---------------------------------------------------------

        # 組み込み完了後、ブール演算によるフィッシュマウス加工を実行
        if main_pipe_id and branch_pipe_id:
            create_boolean_cope_cut(
                access_token,
                TARGET_DID,
                TARGET_WID,
                TARGET_EID,
                main_pipe_id,
                branch_pipe_id,
            )
        else:
            print("エラー: パイプのIDが取得できていません。")

        print("\n=======================================================")
        print(" 処理完了")
        print(
            f" URL: https://cad.onshape.com/documents/{TARGET_DID}/w/{TARGET_WID}/e/{TARGET_EID}"
        )
        print("=======================================================")

    except Exception as e:
        print(f"実行エラー: {e}")


if __name__ == "__main__":
    main()
