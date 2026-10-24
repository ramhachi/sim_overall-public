#!/usr/bin/env python3
"""Reproducible regression summary for the midterm TTC tire report."""

from __future__ import annotations

import argparse
import csv
import json
from dataclasses import asdict, dataclass
from pathlib import Path
import sys
import warnings

import matplotlib.pyplot as plt
import numpy as np
from scipy import stats
from scipy.stats import chi2

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tire_pinn.data.ttc_loader import TTCDataLoader


@dataclass
class RegressionSummary:
    case_name: str
    target: str
    explanatory_variables: list[str]
    filter_description: list[str]
    sample_count_before_outlier_removal: int
    sample_count_after_outlier_removal: int
    removed_outliers: int
    target_outliers_1_96sd: int
    target_outliers_1_5iqr: int
    mahalanobis_outliers_95: int
    r2: float
    adjusted_r2: float
    coefficients: dict[str, float]
    standardized_coefficients: dict[str, float]
    t_values: dict[str, float]
    p_values: dict[str, float]
    vifs: dict[str, float]
    variable_ranges: dict[str, list[float]]


def build_case_definitions(df) -> list[dict]:
    return [
        {
            "case_name": "fx_main",
            "target": "Fx",
            "variables": ["SR", "Fz", "IA", "TSTC"],
            "mask": (
                (df["source_test_type"] == "Db")
                & (df["SA"].abs() <= 1.0)
                & (df["Fz"] >= 300.0)
                & (df["TSTC"].between(35.0, 85.0))
            ),
            "filter_description": [
                "DriveBrake data only",
                "|SA| <= 1.0 deg",
                "Fz >= 300 N",
                "35 <= TSTC <= 85 degC",
            ],
        },
        {
            "case_name": "fy_comparison",
            "target": "Fy",
            "variables": ["SA", "Fz", "IA", "TSTC"],
            "mask": (
                (df["source_test_type"] == "Cr")
                & (df["SR"].abs() <= 0.02)
                & (df["Fz"] >= 300.0)
                & (df["TSTC"].between(35.0, 85.0))
            ),
            "filter_description": [
                "Cornering data only",
                "|SR| <= 0.02",
                "Fz >= 300 N",
                "35 <= TSTC <= 85 degC",
            ],
        },
    ]


def compute_vifs(x: np.ndarray, variable_names: list[str]) -> dict[str, float]:
    vifs: dict[str, float] = {}
    for index, variable_name in enumerate(variable_names):
        x_j = x[:, index]
        x_other = np.column_stack([np.ones(len(x)), np.delete(x, index, axis=1)])
        beta_j, *_ = np.linalg.lstsq(x_other, x_j, rcond=None)
        x_hat = x_other @ beta_j
        residual_ss = np.square(x_j - x_hat).sum()
        total_ss = np.square(x_j - x_j.mean()).sum()
        r2_j = 1.0 - residual_ss / total_ss
        vifs[variable_name] = float(1.0 / (1.0 - r2_j))
    return vifs


def compute_standardized_coefficients(
    beta: np.ndarray, x: np.ndarray, y: np.ndarray, variable_names: list[str]
) -> dict[str, float]:
    x_std = x.std(axis=0, ddof=1)
    y_std = y.std(ddof=1)
    return {
        variable_name: float(beta[index + 1] * x_std[index] / y_std)
        for index, variable_name in enumerate(variable_names)
    }


def summarize_case(df, case_definition: dict) -> RegressionSummary:
    sub = df.loc[case_definition["mask"]].copy()
    target = case_definition["target"]
    variables = case_definition["variables"]

    y = sub[target].to_numpy(dtype=np.float64)
    x = sub[variables].to_numpy(dtype=np.float64)

    mean = float(y.mean())
    std = float(y.std(ddof=1))
    q1, q3 = np.percentile(y, [25.0, 75.0])
    iqr = float(q3 - q1)

    sd_outliers = int(np.sum((y < mean - 1.96 * std) | (y > mean + 1.96 * std)))
    iqr_outliers = int(np.sum((y < q1 - 1.5 * iqr) | (y > q3 + 1.5 * iqr)))

    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        x_mean = x.mean(axis=0)
        cov = np.cov(x, rowvar=False)
        cov_inv = np.linalg.inv(cov)
        md2 = np.einsum("ij,jk,ik->i", x - x_mean, cov_inv, x - x_mean)

    md_threshold = float(chi2.ppf(0.95, x.shape[1]))
    keep_mask = md2 <= md_threshold
    sub_clean = sub.loc[keep_mask].copy()

    x_clean = sub_clean[variables].to_numpy(dtype=np.float64)
    y_clean = sub_clean[target].to_numpy(dtype=np.float64)
    design = np.column_stack([np.ones(len(sub_clean)), x_clean])

    beta, *_ = np.linalg.lstsq(design, y_clean, rcond=None)
    y_hat = design @ beta

    residual_ss = float(np.square(y_clean - y_hat).sum())
    total_ss = float(np.square(y_clean - y_clean.mean()).sum())
    dof = len(sub_clean) - design.shape[1]

    r2 = 1.0 - residual_ss / total_ss
    adjusted_r2 = 1.0 - (residual_ss / dof) / (total_ss / (len(sub_clean) - 1))

    sigma2 = residual_ss / dof
    beta_cov = sigma2 * np.linalg.inv(design.T @ design)
    beta_se = np.sqrt(np.diag(beta_cov))
    t_values = beta / beta_se
    p_values = 2.0 * stats.t.sf(np.abs(t_values), df=dof)

    coefficients = {
        name: float(value)
        for name, value in zip(["Intercept"] + variables, beta)
    }
    standardized_coefficients = compute_standardized_coefficients(beta, x_clean, y_clean, variables)
    t_values_dict = {
        name: float(value)
        for name, value in zip(["Intercept"] + variables, t_values)
    }
    p_values_dict = {
        name: float(value)
        for name, value in zip(["Intercept"] + variables, p_values)
    }

    variable_ranges = {}
    for column in variables + [target]:
        values = sub[column].to_numpy(dtype=np.float64)
        variable_ranges[column] = [float(values.min()), float(values.max())]

    return RegressionSummary(
        case_name=case_definition["case_name"],
        target=target,
        explanatory_variables=variables,
        filter_description=case_definition["filter_description"],
        sample_count_before_outlier_removal=int(len(sub)),
        sample_count_after_outlier_removal=int(len(sub_clean)),
        removed_outliers=int(len(sub) - len(sub_clean)),
        target_outliers_1_96sd=sd_outliers,
        target_outliers_1_5iqr=iqr_outliers,
        mahalanobis_outliers_95=int(np.sum(~keep_mask)),
        r2=float(r2),
        adjusted_r2=float(adjusted_r2),
        coefficients=coefficients,
        standardized_coefficients=standardized_coefficients,
        t_values=t_values_dict,
        p_values=p_values_dict,
        vifs=compute_vifs(x_clean, variables),
        variable_ranges=variable_ranges,
    )


def format_p_value(value: float) -> str:
    if value == 0.0:
        return "<1e-300"
    if value < 1.0e-3:
        return f"{value:.2e}"
    return f"{value:.4f}"


def build_table_rows(summary: RegressionSummary) -> list[list[str]]:
    rows: list[list[str]] = []
    for name in ["Intercept"] + summary.explanatory_variables:
        rows.append(
            [
                name,
                f"{summary.coefficients[name]:.4f}",
                (
                    f"{summary.standardized_coefficients[name]:.4f}"
                    if name in summary.standardized_coefficients
                    else "-"
                ),
                f"{summary.t_values[name]:.3f}",
                format_p_value(summary.p_values[name]),
                f"{summary.vifs[name]:.3f}" if name in summary.vifs else "-",
            ]
        )
    return rows


def write_csv_table(summary: RegressionSummary, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["variable", "coefficient", "standardized_coefficient", "t_value", "p_value", "vif"])
        writer.writerows(build_table_rows(summary))


def render_table_figure(summary: RegressionSummary, output_path: Path) -> None:
    rows = build_table_rows(summary)
    fig, ax = plt.subplots(figsize=(9.5, 2.8))
    ax.axis("off")
    title = (
        f"{summary.case_name}: {summary.target} regression summary\n"
        f"n={summary.sample_count_after_outlier_removal}, "
        f"R^2={summary.r2:.4f}, Adjusted R^2={summary.adjusted_r2:.4f}"
    )
    ax.set_title(title, fontsize=11, loc="left", pad=10)
    table = ax.table(
        cellText=rows,
        colLabels=["Variable", "Coef.", "Std. coef.", "t-value", "p-value", "VIF"],
        loc="center",
        cellLoc="center",
    )
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 1.55)
    for (row, col), cell in table.get_celld().items():
        if row == 0:
            cell.set_facecolor("#DCE6F2")
            cell.set_text_props(weight="bold")
        elif col == 0:
            cell.set_facecolor("#F5F7FA")
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def render_standardized_bar(summary: RegressionSummary, output_path: Path) -> None:
    names = summary.explanatory_variables
    values = [summary.standardized_coefficients[name] for name in names]
    fig, ax = plt.subplots(figsize=(7.4, 3.2))
    colors = ["#5B8FF9" if value >= 0.0 else "#61DDAA" for value in values]
    ax.bar(names, values, color=colors)
    ax.axhline(0.0, color="#444444", linewidth=1.0)
    ax.set_ylabel("Standardized coefficient")
    ax.set_title(f"{summary.case_name}: standardized coefficients")
    for index, value in enumerate(values):
        y = value + 0.02 if value >= 0.0 else value - 0.04
        va = "bottom" if value >= 0.0 else "top"
        ax.text(index, y, f"{value:.3f}", ha="center", va=va, fontsize=9)
    fig.tight_layout()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_path, dpi=220, bbox_inches="tight")
    plt.close(fig)


def write_support_markdown(report: dict, output_path: Path) -> None:
    case_map = {case["case_name"]: case for case in report["cases"]}
    fx = case_map["fx_main"]
    fy = case_map["fy_comparison"]

    def top_std(case: dict) -> list[tuple[str, float]]:
        return sorted(
            case["standardized_coefficients"].items(),
            key=lambda kv: abs(kv[1]),
            reverse=True,
        )

    def top_t(case: dict) -> list[tuple[str, float]]:
        pairs = [(k, v) for k, v in case["t_values"].items() if k != "Intercept"]
        return sorted(pairs, key=lambda kv: abs(kv[1]), reverse=True)

    lines = [
        "# 中間レポート補助資料",
        "",
        "## 要件チェック",
        "- 現在の deck はサンプル PDF に近い構成で、タイトル、データ説明、目的変数と説明変数、外れ値解析、共線性チェック、結果、考察はすでに入っています。",
        "- 現在の deck で最も不足が明確なのは `t-value` の明示です。",
        "- 標準化回帰係数は比較図で部分的には触れていますが、サンプルにより近づけるなら数表か小さな棒グラフを併記すると安全です。",
        "",
        "## 手動で足すとよいもの",
        "- 6枚目: Fx 主解析の係数表に `t-value` 列を追加する。",
        "- 7枚目: サンプルの体裁により寄せるなら、標準化回帰係数の数表か、生成した棒グラフを小さく貼る。",
        "- 2枚目: 仮説を明示するなら `仮説: Fx は主に SR と Fz に支配され、IA と TSTC が補助的に効く。` をそのまま使える。",
        "- 2枚目の文言注意: `Fx = 縦力`, `Fy = 横力` を全スライドで統一する。",
        "",
        "## Fx 主解析",
        f"- n = {fx['sample_count_after_outlier_removal']}",
        f"- R^2 = {fx['r2']:.4f}, Adjusted R^2 = {fx['adjusted_r2']:.4f}",
        f"- 標準化回帰係数が大きい順: {', '.join(f'{k}={v:.3f}' for k, v in top_std(fx)[:3])}",
        f"- |t| が大きい順: {', '.join(f'{k}={v:.1f}' for k, v in top_t(fx)[:3])}",
        "",
        "## Fy 比較解析",
        f"- n = {fy['sample_count_after_outlier_removal']}",
        f"- R^2 = {fy['r2']:.4f}, Adjusted R^2 = {fy['adjusted_r2']:.4f}",
        f"- 標準化回帰係数が大きい順: {', '.join(f'{k}={v:.3f}' for k, v in top_std(fy)[:3])}",
        f"- |t| が大きい順: {', '.join(f'{k}={v:.1f}' for k, v in top_t(fy)[:3])}",
        "",
        "## 生成したファイル",
        "- `midterm_regression_extended_summary.json`: t-value と標準化回帰係数を含む完全版サマリ",
        "- `fx_main_regression_table.csv` / `fy_comparison_regression_table.csv`: そのまま貼り替えやすい数表",
        "- `fx_main_regression_table.png` / `fy_comparison_regression_table.png`: 貼り込み用の表画像",
        "- `fx_main_standardized_coefficients.png` / `fy_comparison_standardized_coefficients.png`: 標準化回帰係数の棒グラフ",
        "",
        "## サンプル PDF から見える傾向",
        "- サンプル deck は 7-9 枚程度で、考察を 1-2 枚に分ける形が多い。",
        "- 外れ値解析と共線性チェックは、それぞれ独立スライドか明確に分離された節として見せている。",
        "- 結果スライドでは、採用変数の回帰係数と補助統計量をコンパクトな表で示す例が多い。",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--tire-name",
        default="Hoosier_18p0X6p0-10_R20_Rim6",
        help="TTC tire dataset identifier",
    )
    parser.add_argument(
        "--output-json",
        type=Path,
        default=None,
        help="Optional JSON output path",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory for tables/figures/markdown support materials",
    )
    args = parser.parse_args()

    warnings.filterwarnings("ignore", category=RuntimeWarning)

    loader = TTCDataLoader(tire_name=args.tire_name, normalize=False)
    df = loader.df

    report = {
        "tire_name": args.tire_name,
        "total_samples": int(len(df)),
        "source_counts": {
            key: int(value)
            for key, value in df["source_test_type"].value_counts().sort_index().items()
        },
        "cases": [
            asdict(summarize_case(df, case_definition))
            for case_definition in build_case_definitions(df)
        ],
    }

    if args.output_json is not None:
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
        args.output_json.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    if args.output_dir is not None:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        json_path = args.output_dir / "midterm_regression_extended_summary.json"
        json_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        for case in report["cases"]:
            summary = RegressionSummary(**case)
            write_csv_table(summary, args.output_dir / f"{summary.case_name}_regression_table.csv")
            render_table_figure(summary, args.output_dir / f"{summary.case_name}_regression_table.png")
            render_standardized_bar(
                summary,
                args.output_dir / f"{summary.case_name}_standardized_coefficients.png",
            )
        write_support_markdown(report, args.output_dir / "midterm_report_support.md")

    if args.output_json is None and args.output_dir is None:
        print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
