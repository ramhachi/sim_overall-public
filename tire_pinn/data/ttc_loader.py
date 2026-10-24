"""TTC tire data loader supporting CSV and raw TTC/TIRF .dat datasets."""

from __future__ import annotations

from pathlib import Path
import re
from typing import Dict, Tuple

import numpy as np
import pandas as pd
import torch
from torch.utils.data import DataLoader, TensorDataset

from .raw_ttc import load_manifest, load_raw_tirf_file, resolve_manifest_path
from .tire_catalog import TireCatalog

CSV_TEST_TYPE_PATTERN = re.compile(r"^(Db|Cr|Cmb)_")
INPUT_COLUMNS = ["SR", "SA", "Fz", "IA", "TSTC"]
OUTPUT_COLUMNS = ["Fx", "Fy", "Mx", "Mz"]
INPUT_UNITS = {
    "SR": "ratio",
    "SA": "deg",
    "Fz": "N",
    "IA": "deg",
    "TSTC": "degC",
}
OUTPUT_UNITS = {"Fx": "N", "Fy": "N", "Mx": "Nm", "Mz": "Nm"}
SOURCE_COMPONENT_MASKS = {
    "Db": np.array([1.0, 0.0, 1.0, 1.0], dtype=np.float32),
    "Cr": np.array([0.0, 1.0, 1.0, 1.0], dtype=np.float32),
    "Cmb": np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
    "Combined": np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
    "Unknown": np.array([1.0, 1.0, 1.0, 1.0], dtype=np.float32),
}


class TTCDataLoader:
    """Load and preprocess TTC tire test data for PINN training."""

    def __init__(
        self,
        drivebrake_csv: Path | None = None,
        cornering_csv: Path | None = None,
        normalize: bool = True,
        tire_name: str | None = "Hoosier_18p0X6p0-10_R20_Rim6",
        dataset_id: str | None = None,
        data_dir: Path | None = None,
        manifest_path: Path | None = None,
        max_samples_total: int | None = None,
        sampling_seed: int = 42,
        source_component_masks: Dict[str, list[float]] | None = None,
        pressure_filter_kpa: float | None = None,
        pressure_tolerance_kpa: float = 3.5,
    ):
        self.normalize = normalize
        # Keep only samples at one inflation pressure (TTC P column, kPa) so the
        # pressure sweep is not pooled into one 5-input model (#18). None = no filter.
        self.pressure_filter_kpa = pressure_filter_kpa
        self.pressure_tolerance_kpa = pressure_tolerance_kpa
        self.data_dir = Path(data_dir) if data_dir is not None else None
        self.manifest_path = resolve_manifest_path(manifest_path, self.data_dir)
        self.raw_manifest = load_manifest(self.manifest_path)
        self.max_samples_total = (
            max_samples_total if max_samples_total and max_samples_total > 0 else None
        )
        self.sampling_seed = sampling_seed
        self.source_component_masks = self._normalize_source_component_masks(
            source_component_masks
        )

        if drivebrake_csv is not None or cornering_csv is not None:
            self.use_catalog = False
            self.drivebrake_paths = [Path(drivebrake_csv)] if drivebrake_csv else []
            self.cornering_paths = [Path(cornering_csv)] if cornering_csv else []
            self.dataset_id = dataset_id or tire_name or "legacy_dataset"
            self.dataset_info = {
                "identifier": self.dataset_id,
                "display_name": self.dataset_id,
                "db_files": [str(path) for path in self.drivebrake_paths],
                "cr_files": [str(path) for path in self.cornering_paths],
                "source_formats": ["csv"],
            }
        else:
            if self.data_dir is None:
                script_dir = Path(__file__).parent
                self.data_dir = script_dir.parent.parent / "data" / "tiredata"
                self.manifest_path = resolve_manifest_path(manifest_path, self.data_dir)
                self.raw_manifest = load_manifest(self.manifest_path)

            self.catalog = TireCatalog(self.data_dir, manifest_path=self.manifest_path)
            self.use_catalog = True
            identifier = dataset_id or tire_name
            if identifier is None:
                raise ValueError(
                    "Either tire_name/dataset_id or explicit CSV paths must be provided"
                )
            self.dataset_info = self.catalog.get_dataset_entry(identifier)
            self.dataset_id = self.dataset_info["identifier"]
            self.drivebrake_paths = [Path(path) for path in self.dataset_info.get("db_files", [])]
            self.cornering_paths = [Path(path) for path in self.dataset_info.get("cr_files", [])]

        self.df = self._load_and_merge()
        self.coverage = {
            "has_db": bool(self.drivebrake_paths),
            "has_cr": bool(self.cornering_paths),
        }
        self.component_mask = self._build_component_mask_array(self.df["source_test_type"])
        self.input_mean = None
        self.input_std = None
        self.output_mean = None
        self.output_std = None
        self.last_split_indices = None
        self.last_split_strategy = None
        self.last_split_config = None

    def _load_and_merge(self) -> pd.DataFrame:
        df_list = []
        db_count = 0
        cr_count = 0

        if self.drivebrake_paths:
            df_db = self._load_paths(self.drivebrake_paths)
            df_list.append(df_db)
            db_count = len(df_db)
        if self.cornering_paths:
            df_cr = self._load_paths(self.cornering_paths)
            df_list.append(df_cr)
            cr_count = len(df_cr)

        if not df_list:
            raise FileNotFoundError(f"No TTC data files found for dataset: {self.dataset_id}")

        df = pd.concat(df_list, ignore_index=True)
        if self.pressure_filter_kpa is not None:
            if "P" not in df.columns:
                raise ValueError("pressure_filter_kpa set but TTC data has no P column")
            keep = (df["P"] - self.pressure_filter_kpa).abs() <= self.pressure_tolerance_kpa
            print(
                f"  Pressure filter {self.pressure_filter_kpa:.1f}"
                f"+/-{self.pressure_tolerance_kpa:.1f} kPa: {int(keep.sum())}/{len(df)} samples kept"
            )
            df = df[keep]
            if df.empty:
                raise ValueError(
                    f"No TTC samples at {self.pressure_filter_kpa} kPa for {self.dataset_id}"
                )
        df = df.drop(columns="P", errors="ignore")
        df = df.dropna().reset_index(drop=True)
        if "source_test_type" in df.columns:
            df["source_test_type"] = df["source_test_type"].fillna("Unknown")

        n_before = len(df)
        df = df[df["SR"].abs() <= 1.5]
        df = df[df["Fz"] > 0]
        df = df[df["TSTC"].between(-50.0, 200.0)]
        n_removed = n_before - len(df)
        if n_removed > 0:
            print(f"  Removed {n_removed} outlier samples (|SR|>1.5, invalid Fz/TSTC)")

        if self.max_samples_total is not None and len(df) > self.max_samples_total:
            original_size = len(df)
            df = self._subsample_dataframe(df)
            print(
                f"  Stratified subsample applied: {original_size} -> {len(df)} "
                f"samples (seed={self.sampling_seed})"
            )

        print(f"Loaded {len(df)} samples from dataset: {self.dataset_id}")
        print(f"  DriveBrake: {db_count}, Cornering: {cr_count}")
        if "source_test_type" in df.columns:
            source_counts = df["source_test_type"].value_counts().sort_index()
            summary = ", ".join(f"{label}:{count}" for label, count in source_counts.items())
            print(f"  Source labels: {summary}")
        if not self.cornering_paths or not self.drivebrake_paths:
            print(
                "  Warning: dataset coverage is partial; deployment gating may reject this model."
            )

        return df

    def _load_paths(self, paths: list[Path]) -> pd.DataFrame:
        frames = []
        for path in paths:
            if path.suffix.lower() == ".csv":
                frames.append(self._load_csv_file(path))
            elif path.suffix.lower() == ".dat":
                _, raw_df = load_raw_tirf_file(path, manifest=self.raw_manifest)
                frames.append(raw_df)
            else:
                raise ValueError(f"Unsupported TTC file type: {path}")
        return pd.concat(frames, ignore_index=True)

    def _load_csv_file(self, path: Path) -> pd.DataFrame:
        df_raw = pd.read_csv(path, skiprows=[1])
        extracted = pd.DataFrame(
            {
            "SR": pd.to_numeric(df_raw.get("SR", 0.0), errors="coerce")
            .fillna(0.0)
            .astype(np.float32),
                "SA": pd.to_numeric(df_raw["SA"], errors="coerce").astype(np.float32),
                "Fz": pd.to_numeric(df_raw["FZ"], errors="coerce").abs().astype(np.float32),
                "IA": pd.to_numeric(df_raw["IA"], errors="coerce").astype(np.float32),
                "TSTC": pd.to_numeric(df_raw["TSTC"], errors="coerce").astype(np.float32),
                "Fx": pd.to_numeric(df_raw["FX"], errors="coerce").astype(np.float32),
                "Fy": pd.to_numeric(df_raw["FY"], errors="coerce").astype(np.float32),
                "Mx": pd.to_numeric(df_raw["MX"], errors="coerce").astype(np.float32),
                "Mz": pd.to_numeric(df_raw["MZ"], errors="coerce").astype(np.float32),
            }
        )
        if "P" in df_raw.columns:
            extracted["P"] = pd.to_numeric(df_raw["P"], errors="coerce").astype(np.float32)
        extracted["source_file"] = path.name
        extracted["source_path"] = str(path)
        extracted["source_id"] = path.stem
        extracted["source_test_type"] = self._infer_csv_test_type(path)
        extracted["source_tire_key"] = self.dataset_id
        return extracted.dropna().reset_index(drop=True)

    def _infer_csv_test_type(self, path: Path) -> str:
        match = CSV_TEST_TYPE_PATTERN.match(path.name)
        if match:
            return match.group(1)
        return "Unknown"

    def _subsample_dataframe(self, df: pd.DataFrame) -> pd.DataFrame:
        if self.max_samples_total is None or len(df) <= self.max_samples_total:
            return df.reset_index(drop=True)

        if "source_test_type" not in df.columns:
            return df.sample(n=self.max_samples_total, random_state=self.sampling_seed).reset_index(
                drop=True
            )

        counts = df["source_test_type"].fillna("Unknown").value_counts().sort_index()
        if len(counts) <= 1:
            return df.sample(n=self.max_samples_total, random_state=self.sampling_seed).reset_index(
                drop=True
            )

        exact = counts / counts.sum() * self.max_samples_total
        allocations = np.floor(exact).astype(int)
        allocations = allocations.clip(lower=1)
        allocations = allocations.combine(counts, min)

        while allocations.sum() > self.max_samples_total:
            removable = allocations[allocations > 1]
            if removable.empty:
                break
            label = removable.sort_values(ascending=False).index[0]
            allocations.loc[label] -= 1

        while allocations.sum() < self.max_samples_total:
            remaining_capacity = counts - allocations
            addable = remaining_capacity[remaining_capacity > 0]
            if addable.empty:
                break
            label = (exact - allocations).loc[addable.index].sort_values(ascending=False).index[0]
            allocations.loc[label] += 1

        sampled_frames = []
        for offset, label in enumerate(allocations.index):
            group = df[df["source_test_type"].fillna("Unknown") == label]
            sampled_frames.append(
                group.sample(
                    n=int(allocations.loc[label]),
                    random_state=self.sampling_seed + offset,
                )
            )

        sampled = pd.concat(sampled_frames, ignore_index=True)
        sampled = sampled.sample(frac=1.0, random_state=self.sampling_seed).reset_index(drop=True)
        return sampled

    @staticmethod
    def _normalize_source_component_masks(
        source_component_masks: Dict[str, list[float]] | None,
    ) -> Dict[str, np.ndarray]:
        if not source_component_masks:
            return {}
        normalized = {}
        for source, values in source_component_masks.items():
            mask = np.array(values, dtype=np.float32)
            if mask.shape != (4,):
                raise ValueError(
                    f"Component mask override for {source!r} must contain 4 values"
                )
            normalized[str(source)] = mask
        return normalized

    def _build_component_mask_array(self, source_types: pd.Series) -> np.ndarray:
        masks = [
            self.source_component_masks.get(
                str(test_type),
                SOURCE_COMPONENT_MASKS.get(str(test_type), SOURCE_COMPONENT_MASKS["Unknown"]),
            )
            for test_type in source_types.fillna("Unknown")
        ]
        return np.stack(masks).astype(np.float32)

    def fit_normalization(self, train_indices: np.ndarray | None = None) -> None:
        """Fit input/output statistics on training rows only.

        Output statistics use each output's component mask so that, for example,
        Mz is not normalized with rows where Mz was not measured.
        """
        input_cols = INPUT_COLUMNS
        output_cols = OUTPUT_COLUMNS
        missing = set(input_cols + output_cols) - set(self.df.columns)
        if missing:
            raise ValueError(f"Missing columns in TTC data: {missing}")

        if train_indices is None:
            raise ValueError(
                "train_indices are required; fit normalization after the train/val/test split"
            )
        train_indices = np.asarray(train_indices, dtype=np.int64)
        if train_indices.size == 0:
            raise ValueError("Cannot fit normalization on an empty training split")

        x_train = self.df.iloc[train_indices][input_cols]
        self.input_mean = x_train.mean().values.astype(np.float32)
        self.input_std = (
            x_train.std().replace(0.0, 1.0).fillna(1.0).values.astype(np.float32)
        )

        train_frame = self.df.iloc[train_indices]
        train_mask = self.component_mask[train_indices] > 0
        output_mean = np.zeros(len(output_cols), dtype=np.float32)
        output_std = np.ones(len(output_cols), dtype=np.float32)
        for j, col in enumerate(output_cols):
            values = train_frame.loc[train_mask[:, j], col]
            if len(values) == 0:
                continue
            output_mean[j] = float(values.mean())
            std = float(values.std())
            output_std[j] = std if std > 0.0 else 1.0
        self.output_mean = output_mean
        self.output_std = output_std

        print("\nNormalization parameters (training split only):")
        print("Input mean:", self.input_mean)
        print("Input std:", self.input_std)
        print("Output mean:", self.output_mean)
        print("Output std:", self.output_std)

    def get_normalization_params(self) -> Dict[str, torch.Tensor]:
        if self.input_mean is None:
            raise RuntimeError(
                "Normalization statistics are not fitted yet; run split_data()/create_dataloaders() "
                "or call fit_normalization(train_indices) explicitly."
            )
        return {
            "input_mean": torch.tensor(self.input_mean, dtype=torch.float32),
            "input_std": torch.tensor(self.input_std, dtype=torch.float32),
            "output_mean": torch.tensor(self.output_mean, dtype=torch.float32),
            "output_std": torch.tensor(self.output_std, dtype=torch.float32),
        }

    def get_input_bounds(self) -> Dict[str, Dict[str, float]]:
        return {
            column: {
                "min": float(self.df[column].min()),
                "max": float(self.df[column].max()),
            }
            for column in INPUT_COLUMNS
        }

    def get_input_units(self) -> Dict[str, str]:
        return dict(INPUT_UNITS)

    def split_data(
        self,
        train_ratio: float = 0.8,
        val_ratio: float = 0.1,
        test_ratio: float = 0.1,
        random_seed: int = 42,
        split_strategy: str = "group",
    ) -> Tuple[TensorDataset, TensorDataset, TensorDataset]:
        assert abs(train_ratio + val_ratio + test_ratio - 1.0) < 1e-6

        input_cols = INPUT_COLUMNS
        output_cols = OUTPUT_COLUMNS

        X = self.df[input_cols].values.astype(np.float32)
        y = self.df[output_cols].values.astype(np.float32)
        component_mask = self.component_mask.astype(np.float32)

        if split_strategy == "group":
            split_indices = self._build_split_indices(
                train_ratio=train_ratio,
                val_ratio=val_ratio,
                test_ratio=test_ratio,
                random_seed=random_seed,
            )
        elif split_strategy == "random":
            split_indices = self._build_random_split_indices(
                train_ratio=train_ratio,
                val_ratio=val_ratio,
                test_ratio=test_ratio,
                random_seed=random_seed,
            )
        else:
            raise ValueError(f"Unknown split_strategy '{split_strategy}'. Use 'group' or 'random'.")
        train_idx = split_indices["train"]
        val_idx = split_indices["val"]
        test_idx = split_indices["test"]

        self.last_split_indices = split_indices
        self.last_split_strategy = split_strategy
        self.last_split_config = {
            "train_ratio": train_ratio,
            "val_ratio": val_ratio,
            "test_ratio": test_ratio,
            "random_seed": random_seed,
            "split_strategy": split_strategy,
        }
        if self.normalize:
            self.fit_normalization(train_idx)

        train_dataset = TensorDataset(
            torch.tensor(X[train_idx]),
            torch.tensor(y[train_idx]),
            torch.tensor(component_mask[train_idx]),
        )
        val_dataset = TensorDataset(
            torch.tensor(X[val_idx]),
            torch.tensor(y[val_idx]),
            torch.tensor(component_mask[val_idx]),
        )
        test_dataset = TensorDataset(
            torch.tensor(X[test_idx]),
            torch.tensor(y[test_idx]),
            torch.tensor(component_mask[test_idx]),
        )

        print(f"\nData split ({split_strategy}):")
        if split_strategy == "group":
            print(
                f"  Train: {len(train_dataset)} samples "
                f"from {len(split_indices['train_groups'])} runs/files"
            )
            print(
                f"  Val:   {len(val_dataset)} samples "
                f"from {len(split_indices['val_groups'])} runs/files"
            )
            print(
                f"  Test:  {len(test_dataset)} samples "
                f"from {len(split_indices['test_groups'])} runs/files"
            )
            shared_eval_groups = sorted(
                set(split_indices["val_groups"]).intersection(split_indices["test_groups"])
            )
            if shared_eval_groups:
                print(
                    "  Note: val/test share holdout runs for sparse source groups; "
                    "train remains source_id-isolated."
                )
        else:
            print(f"  Train: {len(train_dataset)} samples")
            print(f"  Val:   {len(val_dataset)} samples")
            print(f"  Test:  {len(test_dataset)} samples")
            print(
                "  Note: random point split measures in-distribution interpolation; "
                "it is not a run-level extrapolation test."
            )

        return train_dataset, val_dataset, test_dataset

    def _build_random_split_indices(
        self,
        train_ratio: float,
        val_ratio: float,
        test_ratio: float,
        random_seed: int,
    ) -> Dict[str, np.ndarray]:
        n_samples = len(self.df)
        rng = np.random.default_rng(random_seed)
        indices = rng.permutation(n_samples)

        n_train = int(round(n_samples * train_ratio))
        n_val = int(round(n_samples * val_ratio))
        n_train = min(max(n_train, 1), n_samples)
        n_val = min(max(n_val, 1), max(0, n_samples - n_train))
        n_test = n_samples - n_train - n_val
        if n_test <= 0 and n_samples >= 3:
            n_test = 1
            if n_val > 1:
                n_val -= 1
            else:
                n_train -= 1

        train = indices[:n_train]
        val = indices[n_train : n_train + n_val]
        test = indices[n_train + n_val :]
        return {
            "train": np.sort(train),
            "val": np.sort(val),
            "test": np.sort(test),
            "train_groups": [],
            "val_groups": [],
            "test_groups": [],
        }

    def _build_split_indices(
        self,
        train_ratio: float,
        val_ratio: float,
        test_ratio: float,
        random_seed: int,
    ) -> Dict[str, np.ndarray]:
        if "source_id" not in self.df.columns:
            raise ValueError("source_id column is required for group-based split")

        rng = np.random.default_rng(random_seed)
        frame = self.df[["source_id", "source_test_type"]].copy()
        frame["source_test_type"] = frame["source_test_type"].fillna("Unknown")

        train_groups: list[str] = []
        val_groups: list[str] = []
        test_groups: list[str] = []

        for source_type, group_df in frame.groupby("source_test_type", sort=True):
            source_ids = group_df["source_id"].drop_duplicates().to_numpy(dtype=object)
            if len(source_ids) == 0:
                continue
            rng.shuffle(source_ids)

            if len(source_ids) == 1:
                train_groups.extend(source_ids.tolist())
                continue
            if len(source_ids) == 2:
                # No independent test group remains. Keep the holdout as validation
                # only; callers that require a test split get an explicit error below.
                train_groups.append(source_ids[0])
                val_groups.append(source_ids[1])
                continue
            if len(source_ids) == 3:
                train_groups.extend(source_ids[:2].tolist())
                val_groups.append(source_ids[2])
                continue

            n_groups = len(source_ids)
            n_train = int(round(n_groups * train_ratio))
            n_val = int(round(n_groups * val_ratio))
            n_test = n_groups - n_train - n_val

            if n_train <= 0:
                n_train = 1
            if n_test <= 0 and n_groups >= 3:
                n_test = 1
            remaining = n_groups - n_train - n_test
            n_val = max(0, remaining)

            if n_val == 0 and n_groups >= 3 and val_ratio > 0:
                if n_train > n_test:
                    n_train -= 1
                else:
                    n_test -= 1
                n_val = 1

            if n_train + n_val + n_test != n_groups:
                n_train = min(n_train, n_groups)
                n_val = min(n_val, max(0, n_groups - n_train))
                n_test = n_groups - n_train - n_val

            train_groups.extend(source_ids[:n_train].tolist())
            val_groups.extend(source_ids[n_train : n_train + n_val].tolist())
            test_groups.extend(source_ids[n_train + n_val :].tolist())

        train_mask = self.df["source_id"].isin(train_groups).to_numpy()
        val_mask = self.df["source_id"].isin(val_groups).to_numpy()
        test_mask = self.df["source_id"].isin(test_groups).to_numpy()

        if (train_mask & val_mask).any() or (train_mask & test_mask).any():
            raise AssertionError("Group-based split produced train/eval source_id overlap")
        if (val_mask & test_mask).any():
            raise AssertionError(
                "Group-based split produced validation/test source_id overlap; "
                "independent evaluation requires additional source groups"
            )
        if test_mask.sum() == 0:
            raise ValueError(
                "Group-based split has no independent test source groups. "
                "Add more source runs or use split_strategy='random' with an explicit "
                "in-distribution-only interpretation."
            )

        return {
            "train": np.flatnonzero(train_mask),
            "val": np.flatnonzero(val_mask),
            "test": np.flatnonzero(test_mask),
            "train_groups": sorted(set(train_groups)),
            "val_groups": sorted(set(val_groups)),
            "test_groups": sorted(set(test_groups)),
        }

    def get_split_frame(self, split_name: str) -> pd.DataFrame:
        if self.last_split_indices is None:
            raise RuntimeError("split_data() must be called before requesting split frames")
        if split_name not in ("train", "val", "test"):
            raise ValueError("split_name must be 'train', 'val', or 'test'")
        return self.df.iloc[self.last_split_indices[split_name]].reset_index(drop=True)

    def get_split_metadata(self) -> Dict[str, object]:
        if self.last_split_indices is None or self.last_split_config is None:
            raise RuntimeError("split_data() must be called before requesting split metadata")
        strategy = self.last_split_config["split_strategy"]
        if strategy == "group":
            interpretation = "run_level_holdout"
            generalization_scope = "held-out source runs/files"
        else:
            interpretation = "in_distribution_interpolation"
            generalization_scope = (
                "interpolation within the measured distribution; not a run-level "
                "extrapolation claim"
            )
        return {
            **self.last_split_config,
            "interpretation": interpretation,
            "generalization_scope": generalization_scope,
            "train_groups": list(self.last_split_indices["train_groups"]),
            "val_groups": list(self.last_split_indices["val_groups"]),
            "test_groups": list(self.last_split_indices["test_groups"]),
            "n_train": int(len(self.last_split_indices["train"])),
            "n_val": int(len(self.last_split_indices["val"])),
            "n_test": int(len(self.last_split_indices["test"])),
        }

    def create_dataloaders(
        self,
        batch_size: int = 512,
        train_ratio: float = 0.8,
        val_ratio: float = 0.1,
        test_ratio: float = 0.1,
        random_seed: int = 42,
        num_workers: int = 0,
        split_strategy: str = "group",
    ) -> Tuple[DataLoader, DataLoader, DataLoader]:
        train_ds, val_ds, test_ds = self.split_data(
            train_ratio,
            val_ratio,
            test_ratio,
            random_seed,
            split_strategy=split_strategy,
        )

        train_loader = DataLoader(
            train_ds,
            batch_size=batch_size,
            shuffle=True,
            num_workers=num_workers,
            pin_memory=False,
        )
        val_loader = DataLoader(
            val_ds,
            batch_size=batch_size,
            shuffle=False,
            num_workers=num_workers,
            pin_memory=False,
        )
        test_loader = DataLoader(
            test_ds,
            batch_size=batch_size,
            shuffle=False,
            num_workers=num_workers,
            pin_memory=False,
        )

        return train_loader, val_loader, test_loader
