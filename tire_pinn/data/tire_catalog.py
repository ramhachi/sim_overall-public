"""Dataset catalog for TTC CSV and raw TTC/TIRF .dat tire files."""

from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional
import re

from .raw_ttc import load_manifest, parse_raw_tirf_metadata, resolve_manifest_path


CSV_PATTERN = re.compile(r"^(Db|Cr|Cmb)_(.+?)_Run\d+\.csv$")


class TireCatalog:
    """Catalog of available tire datasets across CSV and raw TTC sources."""

    def __init__(self, data_dir: Path, manifest_path: Path | None = None):
        self.data_dir = Path(data_dir)
        self.manifest_path = resolve_manifest_path(manifest_path, self.data_dir)
        self.raw_manifest = load_manifest(self.manifest_path)
        self.csv_dir = self.data_dir / "csv"
        self.datasets = self._scan_datasets()
        # Backward-compatible alias used elsewhere in the repository.
        self.tire_models = self.datasets

    def _empty_entry(self, identifier: str, display_name: str) -> Dict:
        parts = identifier.split("_")
        manufacturer = parts[0] if parts and not identifier.startswith("Project") else "Unknown"
        size = "_".join(parts[1:]) if len(parts) > 1 else "Unknown"
        return {
            "identifier": identifier,
            "display_name": display_name,
            "manufacturer": manufacturer,
            "size": size,
            "deployable": False,
            "has_db": False,
            "has_cr": False,
            "has_cmb": False,
            "n_db_files": 0,
            "n_cr_files": 0,
            "n_cmb_files": 0,
            "db_files": [],
            "cr_files": [],
            "cmb_files": [],
            "source_formats": [],
            "source_kinds": [],
            "project_ids": [],
        }

    def _register_file(
        self,
        entry: Dict,
        test_type: str,
        path: Path,
        source_format: str,
        project_id: str | None = None,
        source_kind: str | None = None,
    ):
        mapping = {
            "Db": ("has_db", "n_db_files", "db_files"),
            "Cr": ("has_cr", "n_cr_files", "cr_files"),
            "Cmb": ("has_cmb", "n_cmb_files", "cmb_files"),
        }
        if test_type not in mapping:
            return

        has_key, count_key, list_key = mapping[test_type]
        entry[has_key] = True
        entry[count_key] += 1
        entry[list_key].append(str(path))
        if source_format not in entry["source_formats"]:
            entry["source_formats"].append(source_format)
        source_kind = source_kind or source_format
        if source_kind not in entry["source_kinds"]:
            entry["source_kinds"].append(source_kind)
        if project_id and project_id not in entry["project_ids"]:
            entry["project_ids"].append(project_id)

    def _scan_datasets(self) -> Dict[str, Dict]:
        datasets: Dict[str, Dict] = {}

        if self.csv_dir.exists():
            for fpath in sorted(self.csv_dir.glob("*.csv")):
                match = CSV_PATTERN.match(fpath.name)
                if not match:
                    continue
                test_type, identifier = match.groups()
                entry = datasets.setdefault(identifier, self._empty_entry(identifier, identifier))
                self._register_file(entry, test_type, fpath, source_format="csv", source_kind="csv")

        for fpath in sorted(self.data_dir.rglob("*.dat")):
            if any(part.startswith(".") for part in fpath.parts):
                continue
            if any(part.lower().startswith("rawdata") for part in fpath.parts):
                continue
            metadata = parse_raw_tirf_metadata(fpath, manifest=self.raw_manifest)
            entry = datasets.setdefault(
                metadata.tire_key,
                self._empty_entry(metadata.tire_key, metadata.tire_name or metadata.tire_key),
            )
            if entry["display_name"] == entry["identifier"] and metadata.tire_name:
                entry["display_name"] = metadata.tire_name
            self._register_file(
                entry,
                metadata.test_type,
                fpath,
                source_format="dat",
                project_id=metadata.project_id,
                source_kind=metadata.source_kind,
            )

        for info in datasets.values():
            info["deployable"] = bool(info["has_db"] and info["has_cr"])

        return datasets

    def list_tire_models(self) -> List[str]:
        return sorted(self.datasets.keys())

    def get_tire_info(self, tire_name: str) -> Optional[Dict]:
        return self.datasets.get(tire_name)

    def get_dataset_entry(self, identifier: str) -> Dict:
        entry = self.get_tire_info(identifier)
        if entry is None:
            available = ", ".join(self.list_tire_models())
            raise KeyError(f"Unknown tire dataset '{identifier}'. Available: {available}")
        return entry

    def filter_by_manufacturer(self, manufacturer: str) -> List[str]:
        return [
            name
            for name, info in self.datasets.items()
            if info["manufacturer"].lower() == manufacturer.lower()
        ]

    def filter_by_size(self, size_pattern: str) -> List[str]:
        return [
            name
            for name, info in self.datasets.items()
            if size_pattern.lower() in info["size"].lower()
        ]

    def get_complete_datasets(self) -> List[str]:
        return [
            name
            for name, info in self.datasets.items()
            if info["has_db"] and info["has_cr"]
        ]

    def get_deployable_datasets(self) -> List[str]:
        return self.get_complete_datasets()

    def print_catalog(self):
        print("=" * 100)
        print("TIRE DATASET CATALOG")
        print("=" * 100)

        manufacturers: Dict[str, List[tuple[str, Dict]]] = {}
        for name, info in self.datasets.items():
            manufacturers.setdefault(info["manufacturer"], []).append((name, info))

        for manufacturer in sorted(manufacturers.keys()):
            print(f"\n{manufacturer}:")
            for name, info in sorted(manufacturers[manufacturer]):
                complete = "✓" if info["has_db"] and info["has_cr"] else "✗"
                sources = "/".join(info["source_formats"]) if info["source_formats"] else "unknown"
                db_str = f"Db:{info['n_db_files']}"
                cr_str = f"Cr:{info['n_cr_files']}"
                cmb_str = f"Cmb:{info['n_cmb_files']}" if info["n_cmb_files"] else ""
                projects = f" | Projects: {','.join(info['project_ids'])}" if info["project_ids"] else ""
                summary = ", ".join(filter(None, [db_str, cr_str, cmb_str]))
                print(f"  [{complete}] {name}")
                print(f"      Size: {info['size']}, Sources: {sources}, Data: {summary}{projects}")

        print("\n" + "=" * 100)
        print(f"Total datasets: {len(self.datasets)}")
        print(f"Deployable datasets (Db+Cr): {len(self.get_complete_datasets())}")
        print("=" * 100)


if __name__ == "__main__":
    script_dir = Path(__file__).parent
    data_dir = script_dir.parent.parent / "data" / "tiredata"
    catalog = TireCatalog(data_dir)
    catalog.print_catalog()
