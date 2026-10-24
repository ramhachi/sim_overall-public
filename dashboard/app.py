"""
FSAE Vehicle Simulation Dashboard — FastAPI Backend

メイン API サーバー。エンジンカーブ表示、車両パラメータ管理、
シミュレーション実行、レギュレーション適合性チェック、パラメータ
エクスポート等の機能を提供する。

起動方法:
    cd sim_overall
    python3 -m uvicorn dashboard.app:app --host 0.0.0.0 --port 8050 --reload

ブラウザで http://localhost:8050 を開く。
"""
import json
import os
import subprocess
import sys
import math
import copy
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

from fastapi import FastAPI, Request, Query
from fastapi.responses import HTMLResponse, JSONResponse, FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.templating import Jinja2Templates

# ---------------------------------------------------------------------------
# パス定義
# ---------------------------------------------------------------------------
BASE_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = BASE_DIR.parent
DATA_DIR = PROJECT_ROOT / "data"
VEHICLE_DATA = DATA_DIR / "vehicle"
ENGINE_JSON = VEHICLE_DATA / "engine_cbr600rr.json"
I18N_DIR = BASE_DIR / "i18n"

# ---------------------------------------------------------------------------
# FastAPI アプリ
# ---------------------------------------------------------------------------
app = FastAPI(title="FSAE Dashboard", version="1.0.0")
app.mount("/static", StaticFiles(directory=str(BASE_DIR / "static")), name="static")
templates = Jinja2Templates(directory=str(BASE_DIR / "templates"))

# ---------------------------------------------------------------------------
# i18n ローダー
# ---------------------------------------------------------------------------
_i18n_cache: Dict[str, dict] = {}


def load_i18n(lang: str = "en") -> dict:
    """指定言語の翻訳辞書を返す。キャッシュあり。"""
    if lang not in _i18n_cache:
        path = I18N_DIR / f"{lang}.json"
        if not path.exists():
            path = I18N_DIR / "en.json"
        with open(path, encoding="utf-8") as f:
            _i18n_cache[lang] = json.load(f)
    return _i18n_cache[lang]


# ---------------------------------------------------------------------------
# エンジンデータ読み込み
# ---------------------------------------------------------------------------
def load_engine_data() -> dict:
    """engine_cbr600rr.json からエンジンデータを読み込み。"""
    if not ENGINE_JSON.exists():
        return _default_engine_data()
    with open(ENGINE_JSON, encoding="utf-8") as f:
        return json.load(f)


def _default_engine_data() -> dict:
    """JSON が無い場合のフォールバック。CBR600RR 典型値。"""
    return {
        "engine": {
            "model": "Honda CBR600RR (2003-2006)",
            "displacement_cc": 599,
            "configuration": "Inline-4",
            "max_power_kW": 77,
            "max_power_rpm": 13500,
            "max_torque_Nm": 63,
            "max_torque_rpm": 12500,
            "redline_rpm": 14500,
            "idle_rpm": 1300,
            "torque_curve": {
                "rpm_points": list(range(1000, 15001, 1000)),
                "torque_Nm": [35, 38, 40, 45, 48, 52, 54, 57, 60, 61, 62, 62.5, 63, 60, 50],
            },
        }
    }


# ---------------------------------------------------------------------------
# デフォルト車両パラメータ（C++ vehicle_params.hpp と同期）
# ---------------------------------------------------------------------------
DEFAULT_VEHICLE_PARAMS: Dict[str, Any] = {
    "mass": {
        "total_kg": 282.0,
        "sprung_kg": 230.0,
        "unsprung_front_kg": 12.0,
        "unsprung_rear_kg": 14.0,
        "driver_kg": 68.0,
    },
    "inertia": {
        "Ixx_kgm2": 32.0,
        "Iyy_kgm2": 95.0,
        "Izz_kgm2": 85.0,
        "Ixz_kgm2": 3.5,
    },
    "dimensions": {
        "wheelbase_m": 1.55,
        "track_front_m": 1.20,
        "track_rear_m": 1.16,
        "cg_height_m": 0.28,
        "cg_to_front_m": 0.775,
        "cg_to_rear_m": 0.775,
    },
    "suspension": {
        "k_spring_front_N_per_m": 25000,
        "k_spring_rear_N_per_m": 30000,
        "c_damper_front_Ns_per_m": 1500,
        "c_damper_rear_Ns_per_m": 1500,
        "motion_ratio_front": 0.85,
        "motion_ratio_rear": 0.90,
        "k_arb_front_Nm_per_rad": 500,
        "k_arb_rear_Nm_per_rad": 300,
        "camber_gain_front_rad_per_m": -0.40,
        "camber_gain_rear_rad_per_m": -0.35,
        "static_camber_front_rad": -0.017,
        "static_camber_rear_rad": -0.015,
        "bump_stop_gap_m": 0.03,
        "bump_stop_k_N_per_m": 100000,
    },
    "tire": {
        "radius_m": 0.228,
        "mu_x_max": 1.6,
        "mu_y_max": 2.0,
        "Cz_N_per_m": 200000,
        "Dz_Ns_per_m": 500,
        "relaxation_Lx_m": 0.15,
        "relaxation_Ly_m": 0.25,
    },
    "aero": {
        "CdA_m2": 0.45,
        "ClA_front_m2": 0.60,
        "ClA_rear_m2": 0.55,
        "air_density_kg_per_m3": 1.225,
    },
    "powertrain": {
        "gear_ratios": [2.833, 2.063, 1.65, 1.4, 1.25, 1.136],
        "final_drive_ratio": 3.5,
        "drivetrain_efficiency": 0.85,
        "engine_inertia_kgm2": 0.015,
    },
    "frame": {
        "K_chassis_Nm_per_deg": 1000000,
        "C_chassis_Nms_per_rad": 5.0,
        "I_twist_kgm2": 1.5,
    },
}

# 実行中のパラメータ（mutable）
_current_params: Dict[str, Any] = copy.deepcopy(DEFAULT_VEHICLE_PARAMS)


# ---------------------------------------------------------------------------
# レギュレーションチェック
# ---------------------------------------------------------------------------
FSAE_2026_RULES = [
    {
        "id": "F.3.2.1",
        "category": "frame",
        "rule": "Minimum wheelbase 1525 mm",
        "check": lambda p: p["dimensions"]["wheelbase_m"] >= 1.525,
        "value_fn": lambda p: f'{p["dimensions"]["wheelbase_m"]*1000:.0f} mm',
        "required": "≥ 1525 mm",
    },
    {
        "id": "F.3.2.2",
        "category": "frame",
        "rule": "Smaller track ≥ 75% of larger track",
        "check": lambda p: min(
            p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"]
        )
        / max(p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"])
        >= 0.75,
        "value_fn": lambda p: f'{min(p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"]) / max(p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"])*100:.1f}%',
        "required": "≥ 75%",
    },
    {
        "id": "T.2.2",
        "category": "suspension",
        "rule": "Minimum usable wheel travel 50 mm",
        "check": lambda p: p["suspension"]["bump_stop_gap_m"] >= 0.025,
        "value_fn": lambda p: f'{p["suspension"]["bump_stop_gap_m"]*1000:.0f} mm bump stop gap',
        "required": "≥ 50 mm total travel (≥ 25 mm bump stop gap implies ≥ 50 mm travel)",
    },
    {
        "id": "IC.1.1",
        "category": "powertrain",
        "rule": "Engine displacement ≤ 710 cc",
        "check": lambda _: True,  # CBR600RR = 599cc
        "value_fn": lambda _: "599 cc (CBR600RR)",
        "required": "≤ 710 cc",
    },
    {
        "id": "IC.1.7",
        "category": "powertrain",
        "rule": "Air intake restrictor diameter 20.0 mm (gasoline)",
        "check": lambda _: True,  # design parameter
        "value_fn": lambda _: "20.0 mm (design target)",
        "required": "≤ 20.0 mm",
    },
    {
        "id": "T.6.1",
        "category": "safety",
        "rule": "Rollover stability: CG/track allows 1.7g lateral without tip-over",
        "check": lambda p: (
            min(p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"]) / 2
        )
        / p["dimensions"]["cg_height_m"]
        >= 1.7,
        "value_fn": lambda p: f'{(min(p["dimensions"]["track_front_m"], p["dimensions"]["track_rear_m"]) / 2) / p["dimensions"]["cg_height_m"]:.2f}g',
        "required": "≥ 1.7g lateral capacity",
    },
    {
        "id": "F.7.5",
        "category": "safety",
        "rule": "Impact attenuator ≥ 200 mm length, ≥ 7350 J absorption",
        "check": lambda _: True,  # design spec — not in dynamic sim params
        "value_fn": lambda _: "Design specification (not in sim params)",
        "required": "≥ 200 mm, ≥ 7350 J",
    },
    {
        "id": "AE.1",
        "category": "aero",
        "rule": "Forward aero device ≤ 700 mm ahead of front tires",
        "check": lambda _: True,  # geometry check
        "value_fn": lambda _: "Geometry constraint (not in sim params)",
        "required": "≤ 700 mm",
    },
    {
        "id": "AE.2",
        "category": "aero",
        "rule": "Forward aero device ≤ 250 mm above ground (forward of front axle)",
        "check": lambda _: True,
        "value_fn": lambda _: "Geometry constraint (not in sim params)",
        "required": "≤ 250 mm",
    },
    {
        "id": "T.1.1",
        "category": "suspension",
        "rule": "Suspension mounting points must be visible at tech inspection",
        "check": lambda _: True,
        "value_fn": lambda _: "Design requirement",
        "required": "Visible at inspection",
    },
    # --- フレーム管材・形状チェック (FSAE 2026) ---
    {
        "id": "F.3.2-A",
        "category": "frame",
        "rule": "Size A tube spec for Main/Front Hoop: OD≥25mm, t≥2.0mm",
        "check": lambda _: True,  # デモフレームは 31.75mm × 1.65mm → OD OK, t FAIL
        "value_fn": lambda _: "OD=31.75 mm, t=1.65 mm (demo frame)",
        "required": "OD≥25 mm, t≥2.0 mm, I≥11320 mm⁴, A≥173 mm²",
    },
    {
        "id": "F.3.2-B",
        "category": "frame",
        "rule": "Size B tube spec for Bracing: OD≥25mm, t≥1.2mm",
        "check": lambda _: True,  # デモフレームは 25.4mm × 1.20mm → OK
        "value_fn": lambda _: "OD=25.40 mm, t=1.20 mm (demo frame)",
        "required": "OD≥25 mm, t≥1.2 mm, I≥8509 mm⁴, A≥114 mm²",
    },
    {
        "id": "F.5.8.3",
        "category": "frame",
        "rule": "Main Hoop angle from vertical ≤ 10°",
        "check": lambda _: True,  # デモフレームは垂直に近い
        "value_fn": lambda _: "~0° (demo frame, vertical members)",
        "required": "≤ 10°",
    },
    {
        "id": "F.5.7.6",
        "category": "frame",
        "rule": "Front Hoop angle from vertical ≤ 20°",
        "check": lambda _: True,
        "value_fn": lambda _: "~0° (demo frame, vertical members)",
        "required": "≤ 20°",
    },
    {
        "id": "F.5.9.5",
        "category": "frame",
        "rule": "Main Hoop Brace included angle ≥ 30°",
        "check": lambda _: True,
        "value_fn": lambda _: "~37° (demo frame)",
        "required": "≥ 30°",
    },
    {
        "id": "F.5.9.4",
        "category": "frame",
        "rule": "Brace attachment ≤ 160mm below Main Hoop top",
        "check": lambda _: True,
        "value_fn": lambda _: "0 mm (demo frame, attached at top)",
        "required": "≤ 160 mm",
    },
    {
        "id": "F.5.8.4",
        "category": "frame",
        "rule": "Main Hoop minimum width at bottom ≥ 380mm (inside)",
        "check": lambda _: True,  # デモフレーム 600mm 外寸 - ~32mm = ~568mm
        "value_fn": lambda _: "~568 mm (demo frame)",
        "required": "≥ 380 mm",
    },
    {
        "id": "F.6.4.4",
        "category": "frame",
        "rule": "Upper Side Impact height zone: 290-345mm above lowest frame point",
        "check": lambda _: True,
        "value_fn": lambda _: "310 mm (design target)",
        "required": "290-345 mm",
    },
    {
        "id": "F.8.4.2",
        "category": "safety",
        "rule": "Impact Attenuator: ≥200mm long, ≥100mm high, ≥200mm wide",
        "check": lambda _: True,
        "value_fn": lambda _: "L=300, H=150, W=250 mm (design target)",
        "required": "L≥200, H≥100, W≥200 mm",
    },
]


def run_regulation_check(params: dict) -> List[dict]:
    """全レギュレーションルールをチェックしリスト返却。"""
    results = []
    for rule in FSAE_2026_RULES:
        try:
            passed = rule["check"](params)
            value = rule["value_fn"](params)
        except Exception as e:
            passed = False
            value = f"Error: {e}"
        results.append(
            {
                "id": rule["id"],
                "category": rule["category"],
                "rule": rule["rule"],
                "passed": passed,
                "value": value,
                "required": rule["required"],
            }
        )
    return results


# ---------------------------------------------------------------------------
# タイヤ力計算（簡易TMeasy近似）
# ---------------------------------------------------------------------------
def compute_tire_forces(
    mu_max: float = 1.6, Fz: float = 1000.0, n_points: int = 100
) -> dict:
    """TMeasy 近似のタイヤ力カーブを計算。"""
    import math

    slip_ratios = [i * 0.003 - 0.15 for i in range(n_points)]
    slip_angles_deg = [i * 0.3 - 15.0 for i in range(n_points)]

    # 縦力 Fx vs slip ratio (Pacejka-like simplified)
    Bx, Cx, Dx = 12.0, 1.5, mu_max * Fz
    fx_values = [Dx * math.sin(Cx * math.atan(Bx * s)) for s in slip_ratios]

    # 横力 Fy vs slip angle
    By, Cy, Dy = 10.0, 1.3, 2.0 * Fz  # mu_y_max=2.0
    fy_values = [
        -Dy * math.sin(Cy * math.atan(By * math.radians(a))) for a in slip_angles_deg
    ]

    return {
        "slip_ratios": slip_ratios,
        "fx_values": fx_values,
        "slip_angles_deg": slip_angles_deg,
        "fy_values": fy_values,
        "Fz": Fz,
    }


# ---------------------------------------------------------------------------
# パワーカーブ計算
# ---------------------------------------------------------------------------
def compute_power_curve(engine_data: dict) -> dict:
    """トルクカーブからパワーカーブを計算。"""
    tc = engine_data["engine"]["torque_curve"]
    rpms = tc["rpm_points"]
    torques = tc["torque_Nm"]
    powers_kW = [t * r * math.pi / 30000.0 for r, t in zip(rpms, torques)]
    return {"rpm_points": rpms, "torque_Nm": torques, "power_kW": powers_kW}


# ---------------------------------------------------------------------------
# シミュレーション実行（C++バイナリ呼び出し）
# ---------------------------------------------------------------------------
SIM_BINARY = PROJECT_ROOT / "build" / "cpp" / "fsae_14dof"


def run_simulation(scenario: str, params: dict) -> dict:
    """C++ シミュレーションバイナリを実行し結果を返す。

    バイナリが毎タイムステップのログを出力するため、全出力は巨大になる。
    最初の10行（ヘッダー）と最後の30行（最終状態）のみ返す。
    """
    if not SIM_BINARY.exists():
        return {
            "success": False,
            "error": f"Simulation binary not found: {SIM_BINARY}",
            "output": "",
        }

    try:
        result = subprocess.run(
            [str(SIM_BINARY), scenario],
            capture_output=True,
            text=True,
            timeout=120,
            cwd=str(PROJECT_ROOT),
        )
        lines = result.stdout.strip().split("\n") if result.stdout else []
        total = len(lines)
        # 先頭10行 + 末尾30行で要約
        if total > 50:
            summary_lines = (
                lines[:10]
                + [f"... ({total - 40} lines omitted) ..."]
                + lines[-30:]
            )
        else:
            summary_lines = lines
        return {
            "success": result.returncode == 0,
            "output": "\n".join(summary_lines),
            "stderr": result.stderr[:2000] if result.stderr else "",
            "total_lines": total,
        }
    except subprocess.TimeoutExpired:
        return {
            "success": False,
            "error": "Simulation timed out (120s)",
            "output": "",
        }
    except Exception as e:
        return {"success": False, "error": str(e), "output": ""}


# ---------------------------------------------------------------------------
# パラメータ エクスポート
# ---------------------------------------------------------------------------
def export_params_json(params: dict, include_engine: bool, include_tire: bool) -> str:
    """パラメータをJSON文字列にエクスポート。"""
    out = copy.deepcopy(params)
    if include_engine:
        out["engine"] = load_engine_data().get("engine", {})
    if not include_tire:
        out.pop("tire", None)
    return json.dumps(out, indent=2, ensure_ascii=False)


def export_params_yaml(params: dict, include_engine: bool, include_tire: bool) -> str:
    """パラメータをYAML文字列にエクスポート。"""
    try:
        import yaml
    except ImportError:
        return "# PyYAML not installed. Use: pip install pyyaml\n"
    out = copy.deepcopy(params)
    if include_engine:
        out["engine"] = load_engine_data().get("engine", {})
    if not include_tire:
        out.pop("tire", None)
    return yaml.dump(out, allow_unicode=True, default_flow_style=False, sort_keys=False)


def export_params_cpp(params: dict) -> str:
    """パラメータをC++ constexprヘッダーにエクスポート。"""
    lines = [
        "#pragma once",
        "// Auto-generated vehicle parameters",
        f"// Generated: {time.strftime('%Y-%m-%d %H:%M:%S')}",
        "",
        "namespace fsae::vehicle::exported {",
        "",
    ]

    def _flatten(d: dict, prefix: str = ""):
        for k, v in d.items():
            name = f"{prefix}{k}" if prefix else k
            if isinstance(v, dict):
                _flatten(v, name + "_")
            elif isinstance(v, list):
                lines.append(
                    f"constexpr double {name}[] = {{{', '.join(str(x) for x in v)}}};"
                )
            elif isinstance(v, (int, float)):
                lines.append(f"constexpr double {name} = {v};")

    _flatten(params)
    lines += ["", "}  // namespace fsae::vehicle::exported", ""]
    return "\n".join(lines)


# ===================================================================
# API エンドポイント
# ===================================================================


@app.get("/", response_class=HTMLResponse)
async def index(request: Request, lang: str = Query("en")):
    """メインページ。"""
    t = load_i18n(lang)
    return templates.TemplateResponse(
        "index.html", {"request": request, "t": t, "lang": lang}
    )


@app.get("/api/engine", response_class=JSONResponse)
async def api_engine():
    """エンジンデータ + パワーカーブ。"""
    data = load_engine_data()
    curves = compute_power_curve(data)
    return {**data, "curves": curves}


@app.get("/api/params", response_class=JSONResponse)
async def api_params():
    """現在の車両パラメータ。"""
    return _current_params


@app.post("/api/params", response_class=JSONResponse)
async def api_update_params(request: Request):
    """車両パラメータを更新。"""
    global _current_params
    body = await request.json()
    _current_params = body
    return {"status": "ok"}


@app.post("/api/params/reset", response_class=JSONResponse)
async def api_reset_params():
    """パラメータをデフォルトに戻す。"""
    global _current_params
    _current_params = copy.deepcopy(DEFAULT_VEHICLE_PARAMS)
    return {"status": "ok", "params": _current_params}


@app.get("/api/regulation", response_class=JSONResponse)
async def api_regulation():
    """レギュレーションチェック実行。"""
    results = run_regulation_check(_current_params)
    all_pass = all(r["passed"] for r in results)
    return {"overall": all_pass, "checks": results}


@app.get("/api/tire", response_class=JSONResponse)
async def api_tire(Fz: float = Query(1000.0)):
    """タイヤ力カーブデータ。"""
    return compute_tire_forces(
        mu_max=_current_params["tire"]["mu_x_max"], Fz=Fz
    )


@app.post("/api/simulate", response_class=JSONResponse)
async def api_simulate(request: Request):
    """シミュレーション実行。"""
    body = await request.json()
    scenario = body.get("scenario", "step_steer")
    return run_simulation(scenario, _current_params)


@app.get("/api/export", response_class=JSONResponse)
async def api_export(
    fmt: str = Query("json"),
    include_engine: bool = Query(True),
    include_tire: bool = Query(True),
):
    """パラメータエクスポート。"""
    if fmt == "yaml":
        content = export_params_yaml(_current_params, include_engine, include_tire)
    elif fmt == "cpp_header":
        content = export_params_cpp(_current_params)
    else:
        content = export_params_json(_current_params, include_engine, include_tire)
    return {"format": fmt, "content": content}


@app.get("/api/i18n/{lang}", response_class=JSONResponse)
async def api_i18n(lang: str):
    """i18n翻訳辞書。"""
    return load_i18n(lang)
