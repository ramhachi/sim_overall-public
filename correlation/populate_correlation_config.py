#!/usr/bin/env python3
"""
populate_correlation_config.py

Read the real-vehicle correlation config, report readiness, and export a
baseline-backed simulator params JSON.

Usage:
    python3 correlation/populate_correlation_config.py --check correlation/real_vehicle_correlation_config.json
    python3 correlation/populate_correlation_config.py --export correlation/real_vehicle_correlation_config.json -o correlation/params_correlated.json
"""

import argparse
import json
import math
from pathlib import Path
from typing import Any


TEMPLATE_PATH = Path(__file__).with_name("params_correlated_template.json")


def is_filled(value: Any) -> bool:
    if value is None:
        return False
    if isinstance(value, str):
        return value.strip() != ""
    if isinstance(value, list):
        return len(value) > 0 and all(is_filled(v) for v in value)
    if isinstance(value, dict):
        return len(value) > 0 and all(is_filled(v) for v in value.values())
    return True


def count_fields(obj: Any, prefix: str = "") -> list[dict]:
    results: list[dict] = []
    if isinstance(obj, dict):
        if "required" in obj and "value" in obj:
            results.append(
                {
                    "path": prefix,
                    "required": bool(obj.get("required", False)),
                    "filled": is_filled(obj.get("value")),
                    "method": obj.get("method", ""),
                }
            )
        else:
            for key, value in obj.items():
                child_prefix = f"{prefix}.{key}" if prefix else key
                results.extend(count_fields(value, child_prefix))
    return results


def check_config(config: dict) -> dict:
    fields = count_fields(config)
    required = [f for f in fields if f["required"]]
    optional = [f for f in fields if not f["required"]]

    required_filled = sum(1 for f in required if f["filled"])
    optional_filled = sum(1 for f in optional if f["filled"])

    return {
        "required_total": len(required),
        "required_filled": required_filled,
        "optional_total": len(optional),
        "optional_filled": optional_filled,
        "readiness_pct": round(100.0 * required_filled / max(len(required), 1), 1),
        "missing_required": [f["path"] for f in required if not f["filled"]],
        "missing_optional": [f["path"] for f in optional if not f["filled"]],
    }


def field(config: dict, *keys: str) -> Any:
    node: Any = config
    for key in keys:
        if not isinstance(node, dict) or key not in node:
            return None
        node = node[key]
    if isinstance(node, dict) and "value" in node:
        return node["value"]
    return node


def assign_if_filled(container: dict, key: str, value: Any) -> None:
    if is_filled(value):
        container[key] = value


def deg_list_to_rad(values: Any) -> Any:
    if not is_filled(values):
        return None
    return [math.radians(v) if v is not None else None for v in values]


def export_params_run(config: dict) -> dict:
    with TEMPLATE_PATH.open() as f:
        out = json.load(f)

    corner_weights = field(config, "mass_and_inertia", "corner_weights_kg")
    unsprung_mass = field(config, "mass_and_inertia", "unsprung_mass_kg")
    wheelbase = field(config, "mass_and_inertia", "wheelbase_m")

    if is_filled(corner_weights):
        total_mass = float(sum(corner_weights))
        out["mass"]["m_total"] = total_mass
        if is_filled(wheelbase) and total_mass > 1e-9:
            front_ratio = float(corner_weights[0] + corner_weights[1]) / total_mass
            out["mass"]["wheelbase"] = wheelbase
            out["mass"]["b"] = wheelbase * front_ratio
            out["mass"]["a"] = wheelbase - out["mass"]["b"]

    if is_filled(unsprung_mass):
        out["mass"]["m_unsprung"] = unsprung_mass
        if out["mass"].get("m_total") is not None:
            out["mass"]["m_sprung"] = out["mass"]["m_total"] - sum(unsprung_mass)

    assign_if_filled(out["mass"], "h_cg", field(config, "mass_and_inertia", "cg_height_m"))
    assign_if_filled(out["mass"], "wheelbase", wheelbase)
    assign_if_filled(out["mass"], "track_f", field(config, "mass_and_inertia", "track_front_m"))
    assign_if_filled(out["mass"], "track_r", field(config, "mass_and_inertia", "track_rear_m"))
    assign_if_filled(out["mass"], "Ixx", field(config, "mass_and_inertia", "Ixx_kgm2"))
    assign_if_filled(out["mass"], "Iyy", field(config, "mass_and_inertia", "Iyy_kgm2"))
    assign_if_filled(out["mass"], "Izz", field(config, "mass_and_inertia", "Izz_kgm2"))
    assign_if_filled(out["mass"], "Ixz", field(config, "mass_and_inertia", "Ixz_kgm2"))
    assign_if_filled(out["mass"], "Iw", field(config, "mass_and_inertia", "wheel_inertia_kgm2"))

    assign_if_filled(out["suspension"], "k_spring", field(config, "suspension_and_chassis", "spring_rates_Npm"))
    assign_if_filled(out["suspension"], "motion_ratio", field(config, "suspension_and_chassis", "motion_ratio"))
    assign_if_filled(out["suspension"], "c0", field(config, "suspension_and_chassis", "damper_c0_Ns_per_m"))
    assign_if_filled(out["suspension"], "k_arb_f", field(config, "suspension_and_chassis", "antiroll_bar_front_Nm_per_rad"))
    assign_if_filled(out["suspension"], "k_arb_r", field(config, "suspension_and_chassis", "antiroll_bar_rear_Nm_per_rad"))
    assign_if_filled(out["suspension"], "bump_stop_gap", field(config, "suspension_and_chassis", "bump_stop_gap_m"))
    assign_if_filled(out["suspension"], "bump_stop_k", field(config, "suspension_and_chassis", "bump_stop_stiffness_Npm"))
    assign_if_filled(out["suspension"], "K_chassis", field(config, "suspension_and_chassis", "chassis_torsional_stiffness_Nm_per_deg"))
    assign_if_filled(out["suspension"], "C_chassis", field(config, "suspension_and_chassis", "chassis_torsional_damping_Nms_per_rad"))
    assign_if_filled(out["suspension"], "I_twist", field(config, "suspension_and_chassis", "chassis_twist_inertia_kgm2"))
    static_camber = deg_list_to_rad(field(config, "kinematics_and_compliance", "static_camber_deg"))
    static_toe = deg_list_to_rad(field(config, "kinematics_and_compliance", "static_toe_deg"))
    assign_if_filled(out["suspension"], "static_camber", static_camber)
    assign_if_filled(out["suspension"], "static_toe", static_toe)
    assign_if_filled(out["suspension"], "camber_gain", field(config, "kinematics_and_compliance", "camber_gain_rad_per_m"))

    assign_if_filled(out["steering"], "ackermann", field(config, "brake_and_steering", "steering_ackermann"))
    assign_if_filled(out["steering"], "rack_ratio", field(config, "brake_and_steering", "steering_rack_ratio_m_per_rad"))
    assign_if_filled(out["steering"], "R_steer", field(config, "brake_and_steering", "steering_ratio"))
    assign_if_filled(out["steering"], "max_angle", field(config, "brake_and_steering", "steering_max_angle_rad"))
    assign_if_filled(out["steering"], "stiffness", field(config, "brake_and_steering", "steering_stiffness_Nm_per_rad"))
    assign_if_filled(out["steering"], "T_fric", field(config, "brake_and_steering", "steering_friction_Nm"))
    assign_if_filled(out["steering"], "I_steer", field(config, "brake_and_steering", "steering_inertia_kgm2"))

    assign_if_filled(out["tire"], "model_name", field(config, "tire_package", "tire_model_id"))
    assign_if_filled(out["tire"], "radius", field(config, "tire_package", "loaded_radius_m"))
    assign_if_filled(out["tire"], "mu_x_max", field(config, "tire_package", "mu_x_max"))
    assign_if_filled(out["tire"], "mu_y_max", field(config, "tire_package", "mu_y_max"))
    assign_if_filled(out["tire"], "Fz_nom", field(config, "tire_package", "Fz_nom_N"))
    assign_if_filled(out["tire"], "Cx", field(config, "tire_package", "Cx_N_per_m"))
    assign_if_filled(out["tire"], "Cy", field(config, "tire_package", "Cy_N_per_rad"))
    assign_if_filled(out["tire"], "k_tire", field(config, "tire_package", "vertical_stiffness_N_per_m"))
    assign_if_filled(out["tire"], "c_tire", field(config, "tire_package", "vertical_damping_Ns_per_m"))
    assign_if_filled(out["tire"], "dmu_dFz", field(config, "tire_package", "dmu_dFz_per_N"))
    assign_if_filled(out["tire"], "pneumatic_trail", field(config, "tire_package", "pneumatic_trail_m"))
    assign_if_filled(out["tire"], "ambient_temperature_c", field(config, "tire_package", "ambient_temperature_c"))
    relax_lx = field(config, "tire_package", "relaxation_Lx_m")
    relax_ly = field(config, "tire_package", "relaxation_Ly_m")
    assign_if_filled(out["tire"], "Lx", relax_lx)
    assign_if_filled(out["tire"], "Ly", relax_ly)
    assign_if_filled(out["tire"]["relaxation"], "Lx_base", relax_lx)
    assign_if_filled(out["tire"]["relaxation"], "Ly_base", relax_ly)

    assign_if_filled(out["powertrain"], "engine_rpm_points", field(config, "powertrain_and_diff", "engine_rpm_points_rpm"))
    assign_if_filled(out["powertrain"], "engine_torque_curve", field(config, "powertrain_and_diff", "engine_torque_curve_Nm"))
    assign_if_filled(out["powertrain"], "gear_ratios", field(config, "powertrain_and_diff", "gear_ratios"))
    assign_if_filled(out["powertrain"], "primary_ratio", field(config, "powertrain_and_diff", "primary_ratio"))
    assign_if_filled(out["powertrain"], "final_ratio", field(config, "powertrain_and_diff", "final_ratio"))
    assign_if_filled(out["powertrain"], "efficiency", field(config, "powertrain_and_diff", "driveline_efficiency"))
    assign_if_filled(out["powertrain"], "lsd_preload", field(config, "powertrain_and_diff", "lsd_preload_Nm"))
    assign_if_filled(out["powertrain"], "lsd_ramp_accel", field(config, "powertrain_and_diff", "lsd_ramp_accel_deg"))
    assign_if_filled(out["powertrain"], "lsd_ramp_decel", field(config, "powertrain_and_diff", "lsd_ramp_decel_deg"))
    assign_if_filled(out["powertrain"], "lsd_effective_radius", field(config, "powertrain_and_diff", "lsd_effective_radius_m"))
    assign_if_filled(out["powertrain"], "lsd_clutch_friction", field(config, "powertrain_and_diff", "lsd_clutch_friction"))
    assign_if_filled(out["powertrain"], "lsd_num_plates", field(config, "powertrain_and_diff", "lsd_num_plates"))

    assign_if_filled(out["brake"], "balance", field(config, "brake_and_steering", "brake_balance_front"))
    assign_if_filled(out["brake"], "max_pressure", field(config, "brake_and_steering", "brake_max_pressure_Pa"))
    assign_if_filled(out["brake"], "mu_pad", field(config, "brake_and_steering", "brake_mu_pad"))
    assign_if_filled(out["brake"], "caliper_area", field(config, "brake_and_steering", "brake_caliper_area_m2"))
    assign_if_filled(out["brake"], "effective_radius", field(config, "brake_and_steering", "brake_effective_radius_m"))

    assign_if_filled(out["aero"], "CL_A", field(config, "aero", "CL_A_m2"))
    assign_if_filled(out["aero"], "CD_A", field(config, "aero", "CD_A_m2"))
    assign_if_filled(out["aero"], "aero_balance", field(config, "aero", "aero_balance_front"))
    assign_if_filled(out["aero"], "h_aero", field(config, "aero", "aero_center_height_m"))
    assign_if_filled(out["aero"], "nominal_front_ride_height_mm", field(config, "aero", "nominal_front_ride_height_mm"))
    assign_if_filled(out["aero"], "nominal_rear_ride_height_mm", field(config, "aero", "nominal_rear_ride_height_mm"))
    assign_if_filled(out["aero"], "reference_speed", field(config, "aero", "reference_speed_mps"))
    reference_density = field(config, "aero", "reference_density_kg_per_m3")
    assign_if_filled(out["aero"], "reference_density", reference_density)
    assign_if_filled(out["aero"], "rho", reference_density)
    assign_if_filled(out["aero"], "map_mode", field(config, "aero", "map_mode"))

    return out


def main() -> None:
    parser = argparse.ArgumentParser(description="Real-vehicle correlation config tool")
    parser.add_argument("config", help="Path to real_vehicle_correlation_config.json")
    parser.add_argument("--check", action="store_true", help="Print readiness report")
    parser.add_argument("--export", action="store_true", help="Export params_correlated.json")
    parser.add_argument("-o", "--output", default="params_correlated.json", help="Output path")
    args = parser.parse_args()

    with open(args.config) as f:
        config = json.load(f)

    if args.check or not args.export:
        report = check_config(config)
        print("=== Correlation Config Readiness ===")
        print(f"Required fields : {report['required_filled']}/{report['required_total']}")
        print(f"Optional fields : {report['optional_filled']}/{report['optional_total']}")
        print(f"Required readiness: {report['readiness_pct']}%")
        if report["missing_required"]:
            print(f"\n--- Missing required fields ({len(report['missing_required'])}) ---")
            for path in report["missing_required"]:
                print(f"  ✗ {path}")

    if args.export:
        params = export_params_run(config)
        with open(args.output, "w") as f:
            json.dump(params, f, indent=2)
        print(f"\nExported simulator params to: {args.output}")


if __name__ == "__main__":
    main()
