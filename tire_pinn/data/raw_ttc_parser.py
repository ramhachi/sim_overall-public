"""Utilities for parsing raw TTC/TIRF .dat tire files."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import re
from typing import Iterable

import numpy as np
import pandas as pd


PROJECT_PATTERN = re.compile(r"Project\s+(\d+)", re.IGNORECASE)
RUN_PATTERN = re.compile(r"Run\s+(\d+)", re.IGNORECASE)
TIRE_NAME_PATTERN = re.compile(r"Tire_Name=([^;]+)", re.IGNORECASE)
RIM_PATTERN = re.compile(r"^R\d{2}$", re.IGNORECASE)
COMPOUND_PATTERN = re.compile(r"^C\d{2,4}$", re.IGNORECASE)


@dataclass(frozen=True)
class RawTTCMetadata:
    """Metadata extracted from a raw TTC/TIRF .dat file."""

    path: Path
    project_id: str
    run_number: str
    tire_name: str | None
    compound: str | None
    dataset_id: str
    display_name: str
    test_type: str


def normalize_tire_name(tire_name: str, compound: str | None = None) -> str:
    """Convert TTC tire names to the repository naming convention."""
    normalized = tire_name.strip().replace(" ", "_").replace(",", "")
    normalized = normalized.replace(".", "p").replace("×", "X")
    if compound and compound not in normalized:
        normalized = f"{normalized}_{compound}"
    return normalized


def _read_header_lines(path: Path) -> tuple[str, str, str]:
    with path.open("r", encoding="utf-8", errors="ignore") as file_obj:
        metadata = file_obj.readline().strip()
        columns = file_obj.readline().strip()
        units = file_obj.readline().strip()
    return metadata, columns, units


def infer_test_type_from_path(path: Path | None) -> str | None:
    """Infer TTC test type from the folder or file name."""
    if path is None:
        return None

    path_obj = Path(path)
    lowered_parts = [part.lower() for part in path_obj.parts]
    filename = path_obj.name.lower()

    if filename.startswith("db_") or any("drivebrake" in part for part in lowered_parts):
        return "Db"
    if filename.startswith("cr_") or any("cornering" in part for part in lowered_parts):
        return "Cr"
    if filename.startswith("cmb_") or any("combined" in part for part in lowered_parts):
        return "Cmb"
    return None


def parse_raw_ttc_metadata(path: Path, sample_rows: int = 200) -> RawTTCMetadata:
    """Parse file metadata and infer TTC test type from a small data sample."""
    path = Path(path)
    metadata_line, _, _ = _read_header_lines(path)

    project_match = PROJECT_PATTERN.search(metadata_line)
    run_match = RUN_PATTERN.search(metadata_line)
    project_id = project_match.group(1) if project_match else "unknown"
    run_number = run_match.group(1) if run_match else path.stem

    tire_name: str | None = None
    compound: str | None = None
    tire_match = TIRE_NAME_PATTERN.search(metadata_line)
    if tire_match:
        tire_descriptor = tire_match.group(1).strip()
        parts = [part.strip() for part in tire_descriptor.split(",") if part.strip()]
        if parts:
            tire_name = parts[0]
            for part in parts[1:]:
                if RIM_PATTERN.match(part):
                    tire_name = f"{tire_name} {part}"
                elif COMPOUND_PATTERN.match(part):
                    compound = part

    dataset_id = (
        normalize_tire_name(tire_name, compound)
        if tire_name
        else f"Project{project_id}_UnknownTire"
    )
    display_name = tire_name or f"Project {project_id} Unknown Tire"
    if compound and tire_name:
        display_name = f"{display_name} {compound}"

    sample = load_raw_ttc_dataframe(path, nrows=sample_rows)
    test_type = infer_test_type(sample, path=path)

    return RawTTCMetadata(
        path=path,
        project_id=project_id,
        run_number=run_number,
        tire_name=tire_name,
        compound=compound,
        dataset_id=dataset_id,
        display_name=display_name,
        test_type=test_type,
    )


def load_raw_ttc_dataframe(path: Path, nrows: int | None = None) -> pd.DataFrame:
    """Load a raw TTC/TIRF .dat file into a DataFrame."""
    path = Path(path)
    _, columns_line, _ = _read_header_lines(path)
    columns = re.split(r"\s+", columns_line.strip())
    if not columns or columns == [""]:
        raise ValueError(f"Failed to parse TTC column names from: {path}")

    return pd.read_csv(
        path,
        sep=r"\s+",
        engine="python",
        names=columns,
        skiprows=3,
        nrows=nrows,
    )


def infer_test_type(df: pd.DataFrame, path: Path | None = None) -> str:
    """Infer whether a TTC file is DriveBrake, Cornering, or Combined."""
    path_test_type = infer_test_type_from_path(path)
    if path_test_type is not None:
        return path_test_type

    sr = pd.to_numeric(df.get("SR", pd.Series(dtype=float)), errors="coerce").dropna()
    sa = pd.to_numeric(df.get("SA", pd.Series(dtype=float)), errors="coerce").dropna()

    sr_activity = float(sr.abs().quantile(0.75)) if not sr.empty else 0.0
    sa_activity = float(sa.abs().quantile(0.75)) if not sa.empty else 0.0

    if sr_activity >= 0.1 and sa_activity <= 2.0:
        return "Db"
    if sa_activity >= 2.0 and sr_activity <= 0.05:
        return "Cr"
    if sr_activity >= 0.05 and sa_activity >= 1.0:
        return "Cmb"
    return "Unknown"


def extract_training_columns(df: pd.DataFrame) -> pd.DataFrame:
    """Convert a raw TTC frame to the normalized training schema."""

    def _series(candidates: Iterable[str], default: float | None = None) -> pd.Series:
        for candidate in candidates:
            if candidate in df.columns:
                return pd.to_numeric(df[candidate], errors="coerce")
        if default is None:
            raise KeyError(f"Missing required TTC columns: {list(candidates)}")
        return pd.Series(np.full(len(df), default, dtype=np.float32))

    tstc = None
    for name in ("TSTC", "TSTI", "TSTO"):
        if name in df.columns:
            tstc = pd.to_numeric(df[name], errors="coerce")
            break
    if tstc is None:
        tstc = pd.Series(np.full(len(df), 40.0, dtype=np.float32))

    training_df = pd.DataFrame(
        {
            "SR": _series(["SR"], default=0.0).astype(np.float32),
            "SA": _series(["SA"]).astype(np.float32),
            "Fz": _series(["FZ"]).abs().astype(np.float32),
            "IA": _series(["IA"], default=0.0).astype(np.float32),
            "TSTC": tstc.astype(np.float32),
            "Fx": _series(["FX"], default=0.0).astype(np.float32),
            "Fy": _series(["FY"], default=0.0).astype(np.float32),
            "Mx": _series(["MX"], default=0.0).astype(np.float32),
            "Mz": _series(["MZ"], default=0.0).astype(np.float32),
        }
    )

    return training_df.dropna().reset_index(drop=True)
