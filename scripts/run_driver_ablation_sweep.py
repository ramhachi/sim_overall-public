#!/usr/bin/env python3
"""Run torque-mode driver A/B and ablation comparisons.

The script exports committed revisions with ``git archive`` into an ignored
runtime directory, builds ``laptime_sim`` for each revision, and runs a fixed
40 Nm torque-mode autocross scenario.  It also runs HEAD-only ablations for the
current driver safety layers and stores one CSV trace per case.
"""

from __future__ import annotations

import argparse
import csv
import datetime as _dt
import json
import os
import shutil
import subprocess
import sys
import tarfile
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any


DEFAULT_COMMITS = [
    "outer_loop_tuned=06c4388",
    "preview_unified=2eecca3",
    "mu_limiter=ba22697",
    "intervention_metrics=e21b1ab",
    "head=HEAD",
]

@dataclass(frozen=True)
class CommitSpec:
    label: str
    rev: str


@dataclass(frozen=True)
class ScenarioSpec:
    track: str
    time_sec: float
    driver_torque_max: float
    params_file: Path | None = None
    params_json: str = ""
    torque_speed_scale: float | None = None


@dataclass(frozen=True)
class CaseSpec:
    case_id: str
    group: str
    commit: CommitSpec
    extra_args: list[str] = field(default_factory=list)
    env: dict[str, str] = field(default_factory=dict)


def run_command(
    args: list[str],
    cwd: Path,
    env: dict[str, str] | None = None,
    timeout: float | None = None,
    check: bool = True,
) -> subprocess.CompletedProcess[str]:
    merged_env = os.environ.copy()
    if env:
        merged_env.update(env)
    result = subprocess.run(
        args,
        cwd=cwd,
        env=merged_env,
        timeout=timeout,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if check and result.returncode != 0:
        raise RuntimeError(
            f"Command failed ({result.returncode}): {' '.join(args)}\n"
            f"cwd={cwd}\nstdout:\n{result.stdout}\nstderr:\n{result.stderr}"
        )
    return result


def repo_root() -> Path:
    result = run_command(["git", "rev-parse", "--show-toplevel"], Path.cwd())
    return Path(result.stdout.strip()).resolve()


def parse_commit_spec(raw: str) -> CommitSpec:
    if "=" in raw:
        label, rev = raw.split("=", 1)
    else:
        rev = raw
        label = rev.replace("/", "_").replace("~", "_").replace("^", "_")
    label = label.strip()
    rev = rev.strip()
    if not label or not rev:
        raise ValueError(f"Invalid commit spec: {raw!r}")
    return CommitSpec(label=label, rev=rev)


def resolve_commit(repo: Path, spec: CommitSpec) -> CommitSpec:
    result = run_command(["git", "rev-parse", "--short=12", spec.rev], repo)
    return CommitSpec(label=spec.label, rev=result.stdout.strip())


def resolve_user_path(repo: Path, raw_path: str) -> Path:
    path = Path(raw_path).expanduser()
    if not path.is_absolute():
        path = repo / path
    return path.resolve()


def export_commit(repo: Path, rev: str, destination: Path) -> None:
    if destination.exists():
        shutil.rmtree(destination)
    destination.mkdir(parents=True, exist_ok=True)

    archive = subprocess.Popen(
        ["git", "archive", "--format=tar", rev],
        cwd=repo,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )
    try:
        assert archive.stdout is not None
        with tarfile.open(fileobj=archive.stdout, mode="r|") as tar:
            tar.extractall(destination)
    finally:
        if archive.stdout is not None:
            archive.stdout.close()
    _, stderr = archive.communicate()
    if archive.returncode != 0:
        raise RuntimeError(f"git archive failed for {rev}: {stderr.decode(errors='replace')}")


def build_laptime_sim(source_dir: Path, jobs: int) -> None:
    build_dir = source_dir / "build"
    run_command(
        ["cmake", "-S", str(source_dir), "-B", str(build_dir), "-G", "Ninja", "-DCMAKE_BUILD_TYPE=Release"],
        source_dir,
    )
    build_cmd = ["cmake", "--build", str(build_dir), "--target", "laptime_sim"]
    if jobs > 0:
        build_cmd.extend(["--parallel", str(jobs)])
    run_command(build_cmd, source_dir)


def extract_json(stdout: str) -> dict[str, Any]:
    begin = stdout.find("%%JSON_BEGIN%%")
    end = stdout.find("%%JSON_END%%")
    if begin == -1 or end == -1 or end <= begin:
        return {}
    payload = stdout[begin + len("%%JSON_BEGIN%%") : end].strip()
    return json.loads(payload)


def run_laptime_case(
    source_dir: Path,
    case: CaseSpec,
    scenario: ScenarioSpec,
    trace_dir: Path,
) -> dict[str, Any]:
    exe = source_dir / "build" / "cpp" / "laptime_sim"
    trace_csv_path = trace_dir / f"{case.case_id}.csv"
    cmd = [
        str(exe),
        "--track",
        scenario.track,
        "--output_json",
        "--output",
        str(trace_csv_path),
        "--time",
        str(int(scenario.time_sec)),
    ]
    if scenario.params_file is not None:
        cmd.extend(["--params_file", str(scenario.params_file)])
        cmd.extend(["--driver-torque-max", f"{scenario.driver_torque_max:.12g}"])
    else:
        cmd.extend(["--params-json", scenario.params_json])
    if scenario.torque_speed_scale is not None:
        cmd.extend(["--torque-speed-scale", f"{scenario.torque_speed_scale:.12g}"])
    cmd.extend(case.extra_args)
    result = run_command(cmd, source_dir, env=case.env, timeout=scenario.time_sec + 30.0, check=False)
    json_result = extract_json(result.stdout)

    log_path = trace_dir / f"{case.case_id}.log"
    log_path.write_text(
        "COMMAND: " + " ".join(cmd) + "\n\nSTDOUT:\n" + result.stdout + "\n\nSTDERR:\n" + result.stderr,
        encoding="utf-8",
    )

    row: dict[str, Any] = {
        "case_id": case.case_id,
        "group": case.group,
        "commit_label": case.commit.label,
        "commit_rev": case.commit.rev,
        "return_code": result.returncode,
        "extra_args": " ".join(case.extra_args),
        "env": json.dumps(case.env, sort_keys=True),
        "log_path": str(log_path),
        "trace_csv_path": str(trace_csv_path),
    }
    row.update(json_result)
    if not json_result:
        row["status"] = "failed"
        row["failure_code"] = "json_missing"
        row["failure_message"] = "laptime_sim did not emit JSON markers"
    return row


def default_ablation_cases(head: CommitSpec) -> list[CaseSpec]:
    huge = "1000000"
    return [
        CaseSpec("head_default", "ablation", head),
        CaseSpec("head_no_understeer_preview", "ablation", head, ["--understeer-gradient", "0"]),
        CaseSpec("head_fixed_preview_legacy", "ablation", head, ["--torque-speed-cap-preview-time", "0"]),
        CaseSpec(
            "head_no_steering_speed_cap",
            "ablation",
            head,
            [
                "--torque-speed-cap-preview-time",
                "0",
                "--torque-speed-cap-preview-distance",
                "0",
            ],
        ),
        CaseSpec(
            "head_steer_only_corner_throttle",
            "ablation",
            head,
            [
                "--corner-throttle-lateral-utilization-start",
                huge,
                "--corner-throttle-lateral-utilization-full",
                str(float(huge) + 1.0),
                "--corner-throttle-drive-slip-start",
                huge,
                "--corner-throttle-drive-slip-full",
                str(float(huge) + 1.0),
            ],
        ),
        CaseSpec(
            "head_no_corner_throttle",
            "ablation",
            head,
            env={"FOPT_DISABLE_DRIVER_CORNER_THROTTLE_LIMIT": "1"},
        ),
    ]


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    preferred = [
        "case_id",
        "group",
        "commit_label",
        "commit_rev",
        "status",
        "failure_code",
        "lap_completed",
        "lap_time",
        "driver_intervention_cost",
        "avg_torque_speed_multiplier",
        "mean_torque_speed_loss",
        "mean_corner_throttle_loss",
        "mean_steering_cap_speed_loss",
        "mean_steering_reach_speed_loss",
        "mean_track_margin_speed_loss",
        "steering_saturation_ratio",
        "mean_steering_lag_ratio",
        "dominant_loss",
        "dominant_loss_steering_lag_ratio",
        "max_lateral_error",
        "max_track_limit_excess",
        "max_speed",
        "avg_speed",
        "return_code",
        "extra_args",
        "env",
        "log_path",
        "trace_csv_path",
    ]
    keys = list(preferred)
    for row in rows:
        for key in row.keys():
            if key not in keys:
                keys.append(key)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def num(row: dict[str, Any], key: str) -> float | None:
    value = row.get(key, "")
    if value in ("", None):
        return None
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def fmt(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value:.3f}"


def write_report(path: Path, rows: list[dict[str, Any]], run_id: str, scenario: ScenarioSpec) -> None:
    commit_rows = [r for r in rows if r.get("group") == "commit"]
    ablation_rows = [r for r in rows if r.get("group") == "ablation"]
    first_commit_lap = num(commit_rows[0], "lap_time") if commit_rows else None
    head_default = next((r for r in ablation_rows if r.get("case_id") == "head_default"), None)
    head_lap = num(head_default or {}, "lap_time")

    lines = [
        "# Driver Ablation Sweep",
        "",
        f"- run_id: `{run_id}`",
        f"- scenario: {scenario.track}, PID torque mode",
        f"- driver_torque_max: {scenario.driver_torque_max:.3f} Nm",
        f"- torque_speed_scale: {fmt(scenario.torque_speed_scale)}",
        f"- params_file: `{scenario.params_file}`" if scenario.params_file is not None else "- params_file: inline JSON",
        "",
        "## Commit A/B",
        "",
        "| case | commit | status | lap_time | delta_vs_first | intervention | torque_loss | throttle_loss | sat_ratio | lag_ratio | dominant_loss |",
        "|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---|",
    ]
    for row in commit_rows:
        lap = num(row, "lap_time")
        delta = lap - first_commit_lap if lap is not None and first_commit_lap is not None else None
        lines.append(
            "| {case} | {commit} | {status} | {lap} | {delta} | {intervention} | {torque_loss} | {throttle_loss} | {sat} | {lag} | {loss} |".format(
                case=row.get("case_id", ""),
                commit=row.get("commit_rev", ""),
                status=row.get("status", ""),
                lap=fmt(lap),
                delta=fmt(delta),
                intervention=fmt(num(row, "driver_intervention_cost")),
                torque_loss=fmt(num(row, "mean_torque_speed_loss")),
                throttle_loss=fmt(num(row, "mean_corner_throttle_loss")),
                sat=fmt(num(row, "steering_saturation_ratio")),
                lag=fmt(num(row, "mean_steering_lag_ratio")),
                loss=row.get("dominant_loss", ""),
            )
        )

    lines.extend(
        [
            "",
            "## HEAD Ablations",
            "",
            "| case | status | lap_time | delta_vs_head | intervention | torque_loss | throttle_loss | steer_cap_loss | reach_cap_loss | sat_ratio | lag_ratio | dominant_loss |",
            "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|",
        ]
    )
    for row in ablation_rows:
        lap = num(row, "lap_time")
        delta = lap - head_lap if lap is not None and head_lap is not None else None
        lines.append(
            "| {case} | {status} | {lap} | {delta} | {intervention} | {torque_loss} | {throttle_loss} | {steer_cap} | {reach_cap} | {sat} | {lag} | {loss} |".format(
                case=row.get("case_id", ""),
                status=row.get("status", ""),
                lap=fmt(lap),
                delta=fmt(delta),
                intervention=fmt(num(row, "driver_intervention_cost")),
                torque_loss=fmt(num(row, "mean_torque_speed_loss")),
                throttle_loss=fmt(num(row, "mean_corner_throttle_loss")),
                steer_cap=fmt(num(row, "mean_steering_cap_speed_loss")),
                reach_cap=fmt(num(row, "mean_steering_reach_speed_loss")),
                sat=fmt(num(row, "steering_saturation_ratio")),
                lag=fmt(num(row, "mean_steering_lag_ratio")),
                loss=row.get("dominant_loss", ""),
            )
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-id", default=_dt.datetime.utcnow().strftime("%Y%m%d_%H%M%S"))
    parser.add_argument("--output-dir", default="")
    parser.add_argument("--commit", action="append", dest="commits", help="label=rev. Can be repeated.")
    parser.add_argument("--skip-commit-sweep", action="store_true")
    parser.add_argument("--skip-ablations", action="store_true")
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--timeout-sec", type=float, default=180.0)
    parser.add_argument("--track", default="autocross")
    parser.add_argument("--params-file", default="", help="Vehicle params JSON path, resolved relative to repo root.")
    parser.add_argument("--params-json", default="", help="Inline vehicle params JSON. Mutually exclusive with --params-file.")
    parser.add_argument("--driver-torque-max", type=float, default=40.0)
    parser.add_argument("--torque-speed-scale", type=float, default=None)
    args = parser.parse_args(argv)

    repo = repo_root()
    if args.params_file and args.params_json:
        raise ValueError("--params-file and --params-json are mutually exclusive")
    params_file = resolve_user_path(repo, args.params_file) if args.params_file else None
    if params_file is not None and not params_file.is_file():
        raise FileNotFoundError(f"params file not found: {params_file}")
    params_json = args.params_json or json.dumps(
        {"steering": {"torque_mode": True, "T_driver_max": args.driver_torque_max}}
    )
    scenario = ScenarioSpec(
        track=args.track,
        time_sec=args.timeout_sec,
        driver_torque_max=args.driver_torque_max,
        params_file=params_file,
        params_json=params_json,
        torque_speed_scale=args.torque_speed_scale,
    )
    output_dir = Path(args.output_dir) if args.output_dir else repo / "output" / "runtime" / "driver_ablation" / args.run_id
    output_dir = output_dir.resolve()
    sources_dir = output_dir / "sources"
    traces_dir = output_dir / "traces"
    output_dir.mkdir(parents=True, exist_ok=True)
    sources_dir.mkdir(parents=True, exist_ok=True)
    traces_dir.mkdir(parents=True, exist_ok=True)

    raw_commits = args.commits if args.commits else DEFAULT_COMMITS
    commit_specs = [resolve_commit(repo, parse_commit_spec(raw)) for raw in raw_commits]
    source_by_rev: dict[str, Path] = {}
    rows: list[dict[str, Any]] = []

    def ensure_source(spec: CommitSpec) -> Path:
        if spec.rev in source_by_rev:
            return source_by_rev[spec.rev]
        source = sources_dir / f"{spec.label}_{spec.rev}"
        print(f"[build] {spec.label} {spec.rev}")
        export_commit(repo, spec.rev, source)
        build_laptime_sim(source, args.jobs)
        source_by_rev[spec.rev] = source
        return source

    if not args.skip_commit_sweep:
        for spec in commit_specs:
            source = ensure_source(spec)
            case = CaseSpec(f"commit_{spec.label}", "commit", spec)
            print(f"[run] {case.case_id}")
            rows.append(run_laptime_case(source, case, scenario, traces_dir))

    if not args.skip_ablations:
        head = next((spec for spec in commit_specs if spec.label == "head"), commit_specs[-1])
        source = ensure_source(head)
        for case in default_ablation_cases(head):
            print(f"[run] {case.case_id}")
            rows.append(run_laptime_case(source, case, scenario, traces_dir))

    samples_path = output_dir / "samples.csv"
    report_path = output_dir / "report.md"
    manifest_path = output_dir / "manifest.json"
    write_csv(samples_path, rows)
    write_report(report_path, rows, args.run_id, scenario)
    manifest = {
        "schema_version": "driver_ablation_sweep/v1",
        "run_id": args.run_id,
        "generated_at_utc": _dt.datetime.utcnow().isoformat(timespec="seconds") + "Z",
        "repo": str(repo),
        "output_dir": str(output_dir),
        "samples_csv": str(samples_path),
        "report": str(report_path),
        "commits": [spec.__dict__ for spec in commit_specs],
        "scenario": {
            "track": scenario.track,
            "time_sec": scenario.time_sec,
            "driver_torque_max": scenario.driver_torque_max,
            "params_file": str(scenario.params_file) if scenario.params_file is not None else "",
            "params_json": scenario.params_json if scenario.params_file is None else "",
            "torque_speed_scale": scenario.torque_speed_scale,
        },
    }
    manifest_path.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")

    print(f"[done] samples: {samples_path}")
    print(f"[done] report:  {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
