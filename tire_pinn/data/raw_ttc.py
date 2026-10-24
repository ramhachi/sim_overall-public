"""
Utilities for loading raw TTC/TIRF `.dat` files.

The repository already contains processed TTC CSV files with explicit tire names.
Raw `.dat` files are less consistent: some include Tire_Name metadata and some only
include project/run identifiers. This module centralizes metadata parsing, test-type
inference, and conversion to the canonical PINN training schema.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import re
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np
import pandas as pd


TIRF_HEADER_PATTERN = re.compile(
    r"Project\s+(?P<project>\d+).*?Run\s+(?P<run>\d+)",
    re.IGNORECASE,
)
TIRE_NAME_PATTERN = re.compile(r"Tire_Name=([^;]+)", re.IGNORECASE)

CANONICAL_COLUMNS = ["SR", "SA", "Fz", "IA", "TSTC", "Fx", "Fy", "Mx", "Mz"]
DEFAULT_RAW_TTC_MANIFEST = "raw_ttc_manifest.json"


@dataclass(frozen=True)
class RawTTCMetadata:
    """Metadata describing a raw TTC/TIRF file."""

    path: Path
    project_id: str
    run_number: str
    tire_name: Optional[str]
    tire_key: str
    test_type: str
    source_kind: str = "raw_dat"
    extra: Dict[str, Any] = field(default_factory=dict)

    @property
    def source_id(self) -> str:
        return f"{self.project_id}:{self.run_number}"


def normalize_tire_name(name: str) -> str:
    """Normalize tire names to the repository's underscore-separated convention."""
    normalized = name.strip()
    normalized = normalized.replace("×", "X")
    normalized = normalized.replace(" ", "_")
    normalized = normalized.replace(",", "")
    normalized = normalized.replace(".", "p")
    normalized = re.sub(r"[^A-Za-z0-9_\-]", "_", normalized)
    normalized = re.sub(r"_+", "_", normalized)
    return normalized.strip("_")


def infer_test_type_from_path(path: Path | None) -> Optional[str]:
    """Infer TTC test type from a file path before falling back to signal heuristics."""
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


def infer_source_kind_from_path(path: Path) -> str:
    """Classify TTC `.dat` files by distribution package, not by parser implementation."""
    lowered_parts = [part.lower() for part in Path(path).parts]
    if any(part.startswith("rawdata") for part in lowered_parts):
        return "rawdata"
    if any(part.startswith("rundata") for part in lowered_parts):
        return "rundata"
    return "dat"


def resolve_manifest_path(
    manifest_path: Optional[Path],
    data_dir: Optional[Path] = None,
) -> Optional[Path]:
    """Resolve the raw TTC manifest path, auto-discovering the default when present."""
    if manifest_path is not None:
        return Path(manifest_path)

    if data_dir is None:
        return None

    candidate = Path(data_dir) / DEFAULT_RAW_TTC_MANIFEST
    if candidate.exists():
        return candidate
    return None


def load_manifest(manifest_path: Optional[Path]) -> Dict[str, Any]:
    """Load an optional raw-data manifest."""
    if manifest_path is None:
        return {}

    path = Path(manifest_path)
    if not path.exists():
        raise FileNotFoundError(f"Raw TTC manifest not found: {path}")

    with path.open("r", encoding="utf-8") as handle:
        return json.load(handle)


def _manifest_entry_for(path: Path, manifest: Dict[str, Any], header: Dict[str, str]) -> Dict[str, Any]:
    files = manifest.get("files", {})
    if isinstance(files, dict):
        if path.name in files:
            return files[path.name]
    elif isinstance(files, list):
        for entry in files:
            if not isinstance(entry, dict):
                continue
            entry_name = entry.get("name") or entry.get("file_name") or entry.get("path")
            if entry_name and Path(str(entry_name)).name == path.name:
                return entry

    project_map = manifest.get("projects", {})
    project_key = header.get("project_id", "")
    if isinstance(project_map, dict):
        if project_key and project_key in project_map:
            return project_map[project_key]
    elif isinstance(project_map, list):
        for entry in project_map:
            if not isinstance(entry, dict):
                continue
            if str(entry.get("project_id", "")) != project_key:
                continue
            run_number = str(entry.get("run_number", "")).strip()
            if run_number and run_number != header.get("run_number", ""):
                continue
            return entry

    return {}


def parse_metadata_line(metadata_line: str) -> Dict[str, str]:
    """Parse project/run/tire metadata from the first TIRF line."""
    parsed: Dict[str, str] = {}

    project_match = TIRF_HEADER_PATTERN.search(metadata_line)
    if project_match:
        parsed["project_id"] = project_match.group("project")
        parsed["run_number"] = project_match.group("run")

    tire_match = TIRE_NAME_PATTERN.search(metadata_line)
    if tire_match:
        parsed["tire_name"] = tire_match.group(1).strip()

    return parsed


def infer_test_type(df: pd.DataFrame, path: Optional[Path] = None) -> str:
    """
    Infer TTC test type from the signal content.

    This is intentionally conservative:
    - DriveBrake: slip ratio is strongly excited while slip angle mostly stays near zero.
    - Cornering: slip angle is strongly excited while slip ratio mostly stays near zero.
    - Combined: both are strongly excited.
    - Unknown: insufficient evidence.
    """

    path_test_type = infer_test_type_from_path(path)
    if path_test_type is not None:
        return path_test_type

    sr_activity = float((df["SR"].abs() > 0.03).mean())
    sa_activity = float((df["SA"].abs() > 1.0).mean())

    if sr_activity > 0.30 and sa_activity < 0.15:
        return "Db"
    if sa_activity > 0.30 and sr_activity < 0.15:
        return "Cr"
    if sr_activity > 0.20 and sa_activity > 0.20:
        return "Cmb"
    return "Unknown"


def _read_raw_tirf(path: Path, nrows: Optional[int] = None) -> pd.DataFrame:
    """
    Read a raw TTC `.dat` file into a DataFrame.

    The repository's `.dat` files are ASCII TIRF files with:
    - row 1: metadata
    - row 2: column names
    - row 3: units
    - row 4+: whitespace/tab separated data
    """

    return pd.read_csv(
        path,
        sep=r"\s+",
        engine="python",
        skiprows=[0, 2],
        nrows=nrows,
    )


def load_raw_tirf_file(
    path: Path,
    manifest: Optional[Dict[str, Any]] = None,
    nrows: Optional[int] = None,
) -> tuple[RawTTCMetadata, pd.DataFrame]:
    """Load a raw TTC `.dat` file and convert it to the canonical training schema."""

    path = Path(path)
    if manifest is None:
        manifest = {}

    with path.open("r", encoding="utf-8", errors="ignore") as handle:
        metadata_line = handle.readline().strip()

    header = parse_metadata_line(metadata_line)
    manifest_entry = _manifest_entry_for(path, manifest, header)

    project_id = str(manifest_entry.get("project_id", header.get("project_id", "unknown")))
    run_number = str(manifest_entry.get("run_number", header.get("run_number", path.stem)))

    tire_name = manifest_entry.get("tire_name", header.get("tire_name"))
    tire_key = (
        normalize_tire_name(str(tire_name))
        if tire_name
        else f"Project{project_id}_UnknownTire"
    )

    raw_df = _read_raw_tirf(path, nrows=nrows)
    column_map = {
        "FZ": "Fz",
        "FX": "Fx",
        "FY": "Fy",
        "MX": "Mx",
        "MZ": "Mz",
    }
    raw_df = raw_df.rename(columns=column_map)

    extracted = pd.DataFrame(
        {
            "SR": raw_df.get("SR", 0.0).astype(np.float32),
            "SA": raw_df["SA"].astype(np.float32),
            "Fz": raw_df["Fz"].abs().astype(np.float32),
            "IA": raw_df["IA"].astype(np.float32),
            "TSTC": raw_df["TSTC"].astype(np.float32),
            "Fx": raw_df["Fx"].astype(np.float32),
            "Fy": raw_df["Fy"].astype(np.float32),
            "Mx": raw_df["Mx"].astype(np.float32),
            "Mz": raw_df["Mz"].astype(np.float32),
        }
    )

    extracted["source_file"] = path.name
    extracted["source_path"] = str(path)
    extracted["source_id"] = f"{project_id}:{run_number}"

    inferred_test_type = manifest_entry.get("test_type") or infer_test_type(extracted, path=path)
    extracted["source_test_type"] = inferred_test_type
    extracted["source_tire_key"] = tire_key

    metadata = RawTTCMetadata(
        path=path,
        project_id=project_id,
        run_number=run_number,
        tire_name=tire_name,
        tire_key=tire_key,
        test_type=inferred_test_type,
        source_kind=infer_source_kind_from_path(path),
        extra={
            "metadata_line": metadata_line,
            "manifest_entry": manifest_entry,
        },
    )

    return metadata, extracted


def parse_raw_tirf_metadata(
    path: Path,
    manifest: Optional[Dict[str, Any]] = None,
    sample_rows: int = 200,
) -> RawTTCMetadata:
    """Parse manifest-aware metadata without loading the full raw TTC file."""

    metadata, _ = load_raw_tirf_file(path, manifest=manifest, nrows=sample_rows)
    return metadata
