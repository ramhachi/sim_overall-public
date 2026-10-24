#!/usr/bin/env python3
"""
Onshape API テスト: 簡単なパイプ（円柱）モデルを作成

動作:
  1. OAuth2 ブラウザログイン
  2. 新規ドキュメント "FSAE_Pipe_Test" を作成
  3. Part Studio にスケッチ（円）を追加
  4. 押し出し（Extrude）で円柱を作成
  5. ブラウザでドキュメントを開く
"""

import http.server
import json
import os
import sys
import urllib.parse
import webbrowser
from pathlib import Path

try:
    import requests
except ImportError:
    print("ERROR: pip install requests")
    sys.exit(1)

# --- 設定 ---
BASE = "https://cad.onshape.com"
OAUTH_AUTH_URL = "https://oauth.onshape.com/oauth/authorize"
OAUTH_TOKEN_URL = "https://oauth.onshape.com/oauth/token"
PORT = 8080
REDIRECT_URI = f"http://localhost:{PORT}/callback"


def load_env():
    env_file = Path(__file__).parent / ".env"
    if env_file.exists():
        for line in env_file.read_text().splitlines():
            line = line.strip()
            if line and not line.startswith("#") and "=" in line:
                k, v = line.split("=", 1)
                os.environ.setdefault(k.strip(), v.strip())
    return os.environ.get("ONSHAPE_CLIENT_ID", ""), os.environ.get("ONSHAPE_CLIENT_SECRET", "")


# --- OAuth ---
class _Handler(http.server.BaseHTTPRequestHandler):
    auth_code = None

    def do_GET(self):
        qs = urllib.parse.parse_qs(urllib.parse.urlparse(self.path).query)
        if "code" in qs:
            _Handler.auth_code = qs["code"][0]
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.end_headers()
            self.wfile.write(b"<h2>OK! Close this tab.</h2>")
        else:
            self.send_response(400)
            self.end_headers()

    def log_message(self, *a):
        pass


def oauth_login(cid, csec):
    url = f"{OAUTH_AUTH_URL}?" + urllib.parse.urlencode({
        "response_type": "code", "client_id": cid,
        "redirect_uri": REDIRECT_URI, "scope": "OAuth2ReadPII OAuth2Read OAuth2Write",
    })
    srv = http.server.HTTPServer(("localhost", PORT), _Handler)
    srv.timeout = 120
    _Handler.auth_code = None
    print(f"  Opening browser...")
    webbrowser.open(url)
    srv.handle_request()
    srv.server_close()
    if not _Handler.auth_code:
        raise RuntimeError("No auth code received")
    r = requests.post(OAUTH_TOKEN_URL, data={
        "grant_type": "authorization_code", "code": _Handler.auth_code,
        "client_id": cid, "client_secret": csec, "redirect_uri": REDIRECT_URI,
    })
    r.raise_for_status()
    return r.json()["access_token"]


# --- API helpers ---
def api_get(token, path, **kwargs):
    r = requests.get(f"{BASE}{path}", headers={
        "Authorization": f"Bearer {token}", "Accept": "application/json",
    }, **kwargs)
    return r


def api_post(token, path, body):
    r = requests.post(f"{BASE}{path}", headers={
        "Authorization": f"Bearer {token}",
        "Accept": "application/json",
        "Content-Type": "application/json",
    }, json=body)
    return r


# --- Onshape operations ---
def create_document(token, name="FSAE_Pipe_Test"):
    """新規ドキュメント作成"""
    r = api_post(token, "/api/v10/documents", {"name": name, "isPublic": False})
    if r.status_code not in (200, 201):
        print(f"  FAILED: {r.status_code} {r.text[:300]}")
        sys.exit(1)
    doc = r.json()
    did = doc["id"]
    wid = doc["defaultWorkspace"]["id"]
    print(f"  Document: {name} (did={did[:12]}...)")
    return did, wid


def get_partstudio(token, did, wid):
    """Part Studio の element ID を取得"""
    r = api_get(token, f"/api/v10/documents/d/{did}/w/{wid}/elements")
    r.raise_for_status()
    for elem in r.json():
        if elem.get("elementType") == "PARTSTUDIO" or elem.get("type") == "Part Studio":
            eid = elem["id"]
            print(f"  Part Studio: eid={eid[:12]}...")
            return eid
    # type フィールドが異なる場合、最初の要素を返す
    elems = r.json()
    if elems:
        eid = elems[0]["id"]
        print(f"  Using first element as Part Studio: eid={eid[:12]}... (type={elems[0].get('elementType', elems[0].get('type', '?'))})")
        return eid
    raise RuntimeError("No Part Studio found")


def eval_featurescript(token, did, wid, eid, script):
    """FeatureScript を実行して result を返す"""
    r = api_post(token, f"/api/v6/partstudios/d/{did}/w/{wid}/e/{eid}/featurescript", {
        "script": script,
    })
    if r.status_code != 200:
        print(f"  FeatureScript failed: {r.status_code} {r.text[:300]}")
        return None
    data = r.json()
    for n in data.get("notices", []):
        if isinstance(n, str):
            print(f"  FS notice: {n}")
        elif isinstance(n, dict):
            m = n.get("message", "")
            print(f"  FS notice: {m.get('value', m) if isinstance(m, dict) else m}")
    return data.get("result")


def extract_transient_ids(result):
    """FeatureScript の transientQueriesToStrings 結果からIDリストを抽出"""
    if not result or not isinstance(result, dict):
        return []
    bt = result.get("btType", "")
    # 配列の場合 (BTFSValueArray-1499 or com.belmonttech...BTFSValueArray)
    if "BTFSValueArray" in bt:
        values = result.get("value", [])
        if isinstance(values, list):
            return [v.get("value", "") for v in values if isinstance(v, dict) and v.get("value")]
        return []
    # Map の場合 — 中に "ids" キーがあればそれを再帰的に処理
    if "BTFSValueMap" in bt:
        entries = result.get("value", [])
        if isinstance(entries, list):
            for entry in entries:
                if isinstance(entry, dict) and entry.get("key") == "ids":
                    return extract_transient_ids(entry.get("value", {}))
        return []
    # 単一文字列の場合
    if "String" in bt:
        val = result.get("value", "")
        return [val] if val else []
    # btType 不明の場合、value が文字列ならそれを返す
    if isinstance(result.get("value"), str) and result["value"]:
        return [result["value"]]
    return []


def get_top_plane_id(token, did, wid, eid):
    """Top plane の transient ID を取得（複数手法を試行）"""

    # 方法1: qCreatedBy で Top plane を取得
    result = eval_featurescript(token, did, wid, eid, """
        function(context is Context, queries is map) {
            var topFace = evaluateQuery(context, qCreatedBy(makeId("Top"), EntityType.FACE));
            return transientQueriesToStrings(topFace);
        }
    """)
    ids = extract_transient_ids(result)
    if ids:
        print(f"  Top plane ID (method 1): {ids[0][:30]}...")
        return ids[0]
    print(f"  Method 1 failed. Result: {json.dumps(result, indent=2)[:200] if result else 'None'}")

    # 方法2: 全ての面を取得して最初の3つ（デフォルト平面）を表示
    result = eval_featurescript(token, did, wid, eid, """
        function(context is Context, queries is map) {
            var allFaces = evaluateQuery(context, qEverything(EntityType.FACE));
            return { "count" : size(allFaces), "ids" : transientQueriesToStrings(allFaces) };
        }
    """)
    if result:
        print(f"  Method 2 - all faces: {json.dumps(result, indent=2)[:400]}")
        # extract_transient_ids は Map も再帰的に処理する
        ids = extract_transient_ids(result)
        if ids:
            # 最初の面が Top plane の可能性が高い
            print(f"  Found {len(ids)} faces, using first: {ids[0][:30]}...")
            return ids[0]

    # 方法3: デフォルト平面の名前で直接検索
    result = eval_featurescript(token, did, wid, eid, """
        function(context is Context, queries is map) {
            var planes = evaluateQuery(context, qBodyType(qEverything(EntityType.BODY), BodyType.MATE_CONNECTOR));
            return transientQueriesToStrings(evaluateQuery(context, qEverything(EntityType.FACE)));
        }
    """)
    ids = extract_transient_ids(result)
    if ids:
        print(f"  Method 3 found {len(ids)} faces, using first: {ids[0][:30]}...")
        return ids[0]

    print("  All methods failed!")
    return None


def add_sketch(token, did, wid, eid, plane_tid, radius_m=0.0127):
    """円スケッチを追加 (デフォルト: OD=25.4mm のパイプ断面)"""
    body = {
        "btType": "BTFeatureDefinitionCall-1406",
        "feature": {
            "btType": "BTMSketch-151",
            "featureType": "newSketch",
            "name": "Pipe Profile",
            "suppressed": False,
            "parameters": [
                {
                    "btType": "BTMParameterQueryList-148",
                    "parameterId": "sketchPlane",
                    "queries": [{
                        "btType": "BTMIndividualQuery-138",
                        "deterministicIds": [plane_tid],
                    }],
                },
            ],
            "entities": [
                {
                    "btType": "BTMSketchCurve-4",
                    "entityId": "circle-outer",
                    "centerId": "circle-outer.center",
                    "geometry": {
                        "btType": "BTCurveGeometryCircle-115",
                        "radius": radius_m,
                        "xCenter": 0.0,
                        "yCenter": 0.0,
                        "xDir": 1.0,
                        "yDir": 0.0,
                        "clockwise": False,
                    },
                },
            ],
            "constraints": [],
        },
    }
    r = api_post(token, f"/api/v6/partstudios/d/{did}/w/{wid}/e/{eid}/features", body)
    if r.status_code != 200:
        print(f"  Sketch FAILED: {r.status_code}")
        print(f"  Response: {r.text[:500]}")
        return None
    data = r.json()
    feat = data.get("feature", {})
    fid = feat.get("featureId", "")
    status = feat.get("featureStatus", "?")
    print(f"  Sketch added: featureId={fid}, status={status}")
    if status != "OK":
        notices = data.get("featureState", {}).get("notices", [])
        for n in notices:
            msg = n.get("message", "") if isinstance(n, dict) else str(n)
            if isinstance(msg, dict):
                msg = msg.get("value", str(msg))
            print(f"    Notice: {msg}")
    return fid


def get_sketch_region_id(token, did, wid, eid, sketch_fid):
    """FeatureScript で sketch region の transient ID を取得"""
    result = eval_featurescript(token, did, wid, eid, f"""
    function(context is Context, queries is map) {{
        var regions = evaluateQuery(context, qSketchRegion(makeId("{sketch_fid}")));
        return transientQueriesToStrings(regions);
    }}
    """)
    ids = extract_transient_ids(result)
    if ids:
        print(f"  Sketch region transient ID: {ids[0][:30]}...")
        return ids[0]
    print(f"  No sketch region found. Result: {json.dumps(result, indent=2)[:300] if result else 'None'}")
    return None


def add_extrude(token, did, wid, eid, region_tid, depth_mm=500):
    """押し出し（Extrude）を追加"""
    body = {
        "btType": "BTFeatureDefinitionCall-1406",
        "feature": {
            "btType": "BTMFeature-134",
            "featureType": "extrude",
            "name": "Pipe Extrude",
            "suppressed": False,
            "parameters": [
                {
                    "btType": "BTMParameterEnum-145",
                    "parameterId": "bodyType",
                    "enumName": "ExtendedToolBodyType",
                    "value": "SOLID",
                },
                {
                    "btType": "BTMParameterEnum-145",
                    "parameterId": "operationType",
                    "enumName": "NewBodyOperationType",
                    "value": "NEW",
                },
                {
                    "btType": "BTMParameterQueryList-148",
                    "parameterId": "entities",
                    "queries": [{
                        "btType": "BTMIndividualQuery-138",
                        "deterministicIds": [region_tid],
                    }],
                },
                {
                    "btType": "BTMParameterEnum-145",
                    "parameterId": "endBound",
                    "enumName": "BoundingType",
                    "value": "BLIND",
                },
                {
                    "btType": "BTMParameterQuantity-147",
                    "parameterId": "depth",
                    "expression": f"{depth_mm} mm",
                },
            ],
        },
    }
    r = api_post(token, f"/api/v6/partstudios/d/{did}/w/{wid}/e/{eid}/features", body)
    if r.status_code != 200:
        print(f"  Extrude FAILED: {r.status_code}")
        print(f"  Response: {r.text[:500]}")
        return None
    data = r.json()
    feat = data.get("feature", {})
    fid = feat.get("featureId", "")
    status = feat.get("featureStatus", "?")
    print(f"  Extrude added: featureId={fid}, status={status}")
    if status != "OK":
        notices = data.get("featureState", {}).get("notices", [])
        for n in notices:
            msg = n.get("message", "") if isinstance(n, dict) else str(n)
            if isinstance(msg, dict):
                msg = msg.get("value", str(msg))
            print(f"    Notice: {msg}")
    return fid


# --- Main ---
def main():
    print("=" * 50)
    print(" Onshape: Create Test Pipe Model")
    print("=" * 50)

    cid, csec = load_env()
    if not cid or not csec:
        print("ERROR: .env に ONSHAPE_CLIENT_ID / SECRET を設定してください")
        sys.exit(1)

    # 1. OAuth login
    print("\n[1] OAuth login")
    token = oauth_login(cid, csec)
    print("  Token acquired")

    # 2. Create document
    print("\n[2] Create document")
    did, wid = create_document(token)

    # 3. Get Part Studio
    print("\n[3] Get Part Studio")
    eid = get_partstudio(token, did, wid)

    # 4. Get Top plane transient ID
    print("\n[4] Get Top plane ID")
    plane_tid = get_top_plane_id(token, did, wid, eid)
    if not plane_tid:
        print("  ABORT: Could not resolve Top plane")
        print(f"\n  Document URL: {BASE}/documents/{did}")
        sys.exit(1)

    # 5. Add sketch (circle, OD=25.4mm)
    print("\n[5] Add circle sketch (R=12.7mm)")
    sketch_fid = add_sketch(token, did, wid, eid, plane_tid, radius_m=0.0127)
    if not sketch_fid:
        print("  ABORT: Sketch creation failed")
        print(f"\n  Document URL: {BASE}/documents/{did}")
        sys.exit(1)

    # 6. Get sketch region transient ID
    print("\n[6] Get sketch region ID")
    region_tid = get_sketch_region_id(token, did, wid, eid, sketch_fid)
    if not region_tid:
        print("  ABORT: Could not find sketch region")
        print(f"\n  Document URL: {BASE}/documents/{did}")
        sys.exit(1)

    # 7. Add extrude (500mm)
    print("\n[7] Extrude 500mm")
    extrude_fid = add_extrude(token, did, wid, eid, region_tid, depth_mm=500)

    # 8. Done
    doc_url = f"{BASE}/documents/{did}/w/{wid}/e/{eid}"
    print("\n" + "=" * 50)
    if extrude_fid:
        print(f" SUCCESS! Pipe model created")
    else:
        print(f" Partial: sketch created but extrude may have failed")
    print(f" Open in browser: {doc_url}")
    print("=" * 50)
    webbrowser.open(doc_url)


if __name__ == "__main__":
    main()
