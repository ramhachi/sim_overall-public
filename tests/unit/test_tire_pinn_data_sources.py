import json
from pathlib import Path

import numpy as np
import pandas as pd
import pytest
import torch

from tire_pinn.data.tire_catalog import TireCatalog
from tire_pinn.data.ttc_loader import TTCDataLoader
from tire_pinn.losses.data_loss import DataLoss, R2Score


def _write_raw_dat(path: Path, tire_name: str, rows: list[list[float]], include_sl: bool = False) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "ET",
        "V",
        "N",
        "SA",
        "IA",
        "RL",
        "RE",
        "P",
        "FX",
        "FY",
        "FZ",
        "MX",
        "MZ",
        "NFX",
        "NFY",
        "RST",
        "TSTI",
        "TSTC",
        "TSTO",
        "AmbTmp",
        "SR",
    ]
    units = [
        "s",
        "kph",
        "rpm",
        "deg",
        "deg",
        "cm",
        "cm",
        "kPa",
        "N",
        "N",
        "N",
        "Nm",
        "Nm",
        "none",
        "none",
        "deg c",
        "deg c",
        "deg c",
        "deg c",
        "deg c",
        "SAE",
    ]
    if include_sl:
        columns.append("SL")
        units.append("none")

    header = (
        "TIRF Data File: Project 2356; Run 001; Date 01-20-2022; Time 09:50; "
        f"Tire_Name={tire_name}; Rim_Width=6.0; ISO False; ID TEST"
    )
    lines = [
        header,
        "\t".join(columns),
        "\t".join(units),
        *("\t".join(str(value) for value in row) for row in rows),
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _write_project_only_raw_dat(
    path: Path,
    project_id: str,
    run_number: str,
    rows: list[list[float]],
    include_sl: bool = False,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    columns = [
        "ET",
        "V",
        "N",
        "SA",
        "IA",
        "RL",
        "RE",
        "P",
        "FX",
        "FY",
        "FZ",
        "MX",
        "MZ",
        "NFX",
        "NFY",
        "RST",
        "TSTI",
        "TSTC",
        "TSTO",
        "AmbTmp",
        "SR",
    ]
    units = [
        "s",
        "kph",
        "rpm",
        "deg",
        "deg",
        "cm",
        "cm",
        "kPa",
        "N",
        "N",
        "N",
        "Nm",
        "Nm",
        "none",
        "none",
        "deg c",
        "deg c",
        "deg c",
        "deg c",
        "deg c",
        "SAE",
    ]
    if include_sl:
        columns.append("SL")
        units.append("none")

    header = (
        f"TIRF Data File: Project {project_id}; Run {run_number}; Date 01-20-2022; Time 09:50; "
        "ISO False; ID TEST"
    )
    lines = [
        header,
        "\t".join(columns),
        "\t".join(units),
        *("\t".join(str(value) for value in row) for row in rows),
    ]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def test_tire_catalog_scans_nested_raw_dat_files_by_directory_type(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    db_rows = [
        [0.0, 40.0, 400.0, 2.5, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12, 0.0],
        [0.1, 40.0, 401.0, 2.0, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15, 0.0],
    ]
    cr_rows = [
        [0.0, 40.0, 400.0, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15],
        [0.1, 40.0, 401.0, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12],
    ]
    _write_raw_dat(
        tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run27.dat",
        tire_name,
        db_rows,
        include_sl=True,
    )
    _write_raw_dat(
        tmp_path / "RunData_Cornering_ASCII_SI_Round9" / "B2356run28.dat",
        tire_name,
        cr_rows,
    )

    catalog = TireCatalog(tmp_path)
    info = catalog.get_dataset_entry("Hoosier_18p0X6p0-10_R20")

    assert info["has_db"] is True
    assert info["has_cr"] is True
    assert info["n_db_files"] == 1
    assert info["n_cr_files"] == 1
    assert info["source_kinds"] == ["rundata"]
    assert info["deployable"] is True


def test_tire_catalog_ignores_rawdata_directories(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    rows = [
        [0.0, 40.0, 400.0, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12],
    ]
    _write_raw_dat(
        tmp_path / "RawData_10inch_DriveBrake_ASCII_SI_round6" / "B1654raw44.dat",
        tire_name,
        rows,
    )

    catalog = TireCatalog(tmp_path)

    assert "Hoosier_18p0X6p0-10_R20" not in catalog.list_tire_models()


def test_round8_manifest_auto_discovers_project_only_raw_dat_files(tmp_path: Path):
    cr_rows = [
        [0.0, 40.0, 400.0, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        [0.1, 40.0, 401.0, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
    ]
    round8_dir = tmp_path / "RunData_Cornering_ASCII_SI_10inch_Round8 "
    for run_number in ("001", "002", "004"):
        _write_project_only_raw_dat(
            round8_dir / f"B1965run{int(run_number)}.dat",
            project_id="1965",
            run_number=run_number,
            rows=cr_rows,
        )

    manifest = {
        "files": [
            {
                "name": "B1965run1.dat",
                "project_id": "1965",
                "run_number": "001",
                "tire_name": "Hoosier 16.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
            {
                "name": "B1965run2.dat",
                "project_id": "1965",
                "run_number": "002",
                "tire_name": "Hoosier 16.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
            {
                "name": "B1965run4.dat",
                "project_id": "1965",
                "run_number": "004",
                "tire_name": "Hoosier 16.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
        ]
    }
    (tmp_path / "raw_ttc_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    catalog = TireCatalog(tmp_path)
    info = catalog.get_dataset_entry("Hoosier_16p0X7p5-10_R25B_Rim7")

    assert info["has_db"] is False
    assert info["has_cr"] is True
    assert info["n_cr_files"] == 3
    assert info["project_ids"] == ["1965"]

    loader = TTCDataLoader(
        normalize=False,
        tire_name="Hoosier_16p0X7p5-10_R25B_Rim7",
        data_dir=tmp_path,
    )
    assert loader.coverage == {"has_db": False, "has_cr": True}
    assert set(loader.df["source_tire_key"]) == {"Hoosier_16p0X7p5-10_R25B_Rim7"}
    assert set(loader.df["source_test_type"]) == {"Cr"}


def test_round6_manifest_combines_drivebrake_rundata_and_cornering_for_complete_dataset(
    tmp_path: Path,
):
    db_rows = [
        [0.0, 40.0, 400.0, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12],
        [0.1, 40.0, 401.0, 0.1, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15],
    ]
    cr_rows = [
        [0.0, 40.0, 400.0, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        [0.1, 40.0, 401.0, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
    ]
    db_dir = tmp_path / "RunData_10inch_DriveBrake_ASCII_SI_round6"
    cr_dir = tmp_path / "RunData_10inch_Cornering_ASCII_SI_round6"
    for run_number in ("34", "35"):
        _write_project_only_raw_dat(
            db_dir / f"B1654run{run_number}.dat",
            project_id="1654",
            run_number=run_number,
            rows=db_rows,
        )
    for run_number in ("20", "21", "22"):
        _write_project_only_raw_dat(
            cr_dir / f"B1654run{run_number}.dat",
            project_id="1654",
            run_number=run_number,
            rows=cr_rows,
        )

    manifest = {
        "files": [
            {
                "name": "B1654run34.dat",
                "project_id": "1654",
                "run_number": "34",
                "tire_name": "Hoosier 18.0X7.5-10, R25B, Rim7",
                "test_type": "Db",
            },
            {
                "name": "B1654run35.dat",
                "project_id": "1654",
                "run_number": "35",
                "tire_name": "Hoosier 18.0X7.5-10, R25B, Rim7",
                "test_type": "Db",
            },
            {
                "name": "B1654run20.dat",
                "project_id": "1654",
                "run_number": "20",
                "tire_name": "Hoosier 18.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
            {
                "name": "B1654run21.dat",
                "project_id": "1654",
                "run_number": "21",
                "tire_name": "Hoosier 18.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
            {
                "name": "B1654run22.dat",
                "project_id": "1654",
                "run_number": "22",
                "tire_name": "Hoosier 18.0X7.5-10, R25B, Rim7",
                "test_type": "Cr",
            },
        ]
    }
    (tmp_path / "raw_ttc_manifest.json").write_text(json.dumps(manifest), encoding="utf-8")

    catalog = TireCatalog(tmp_path)
    info = catalog.get_dataset_entry("Hoosier_18p0X7p5-10_R25B_Rim7")

    assert info["has_db"] is True
    assert info["has_cr"] is True
    assert info["n_db_files"] == 2
    assert info["n_cr_files"] == 3
    assert info["deployable"] is True

    loader = TTCDataLoader(
        normalize=False,
        tire_name="Hoosier_18p0X7p5-10_R25B_Rim7",
        data_dir=tmp_path,
    )
    assert loader.coverage == {"has_db": True, "has_cr": True}
    assert set(loader.df["source_tire_key"]) == {"Hoosier_18p0X7p5-10_R25B_Rim7"}
    assert set(loader.df["source_test_type"]) == {"Db", "Cr"}


def test_ttc_data_loader_builds_force_component_masks_from_source_type(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    db_rows = [
        [0.0, 40.0, 400.0, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12, 0.0],
        [0.1, 40.0, 401.0, 0.1, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15, 0.0],
    ]
    cr_rows = [
        [0.0, 40.0, 400.0, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        [0.1, 40.0, 401.0, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
    ]
    _write_raw_dat(
        tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run27.dat",
        tire_name,
        db_rows,
        include_sl=True,
    )
    _write_raw_dat(
        tmp_path / "RunData_Cornering_ASCII_SI_Round9" / "B2356run28.dat",
        tire_name,
        cr_rows,
    )

    loader = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)

    assert loader.coverage == {"has_db": True, "has_cr": True}
    assert loader.get_input_units()["SA"] == "deg"
    assert loader.get_input_units()["IA"] == "deg"
    for label, mask in zip(loader.df["source_test_type"], loader.component_mask):
        if label == "Db":
            assert mask.tolist() == [1.0, 0.0, 1.0, 1.0]
        elif label == "Cr":
            assert mask.tolist() == [0.0, 1.0, 1.0, 1.0]
        else:
            raise AssertionError(f"Unexpected source label in test dataset: {label}")


def test_ttc_data_loader_drops_crawl_speed_rows(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    cr_rows = [
        [0.0, 40.0, 400.0, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 60.0, 60.0, 61.0, 25.0, 0.01],
        # Warm-up crawl: cold, noisy SR, no Fx.
        [0.1, 3.0, 30.0, 0.0, 0.0, 21.0, 21.2, 80.0, 1.0, 0.0, -1000.0, 1.0, 0.0, 0.0, 0.0, 30.0, 28.0, 28.0, 28.0, 25.0, 1.2],
    ]
    _write_raw_dat(
        tmp_path / "RunData_Cornering_ASCII_SI_Round9" / "B2356run28.dat",
        tire_name,
        cr_rows,
    )

    loader = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)

    assert len(loader.df) == 1
    assert loader.df["SR"].abs().max() < 0.1
    assert "V" not in loader.df.columns


def test_data_loss_and_r2_ignore_unsupervised_force_components():
    pred = torch.tensor(
        [
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ],
        dtype=torch.float32,
    )
    target = torch.tensor(
        [
            [0.0, 10.0, 0.0, 0.0],
            [10.0, 0.0, 0.0, 0.0],
        ],
        dtype=torch.float32,
    )
    component_mask = torch.tensor(
        [
            [1.0, 0.0, 1.0, 1.0],
            [0.0, 1.0, 1.0, 1.0],
        ],
        dtype=torch.float32,
    )

    loss_fn = DataLoss(component_weights=[1.0, 1.0, 1.0, 1.0])
    loss, _ = loss_fn(pred, target, component_mask=component_mask)
    r2 = R2Score.compute_dict(pred, target, component_mask=component_mask)

    assert torch.isclose(loss, torch.tensor(0.0))
    assert r2["r2_fx"] == 1.0
    assert r2["r2_fy"] == 1.0


def test_ttc_data_loader_split_data_keeps_source_ids_isolated(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    for run_idx in range(4):
        db_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 0.1, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15, 0.0],
        ]
        cr_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        ]
        _write_raw_dat(
            tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / f"B2356run{50 + run_idx}.dat",
            tire_name,
            db_rows,
            include_sl=True,
        )
        _write_raw_dat(
            tmp_path / "RunData_Cornering_ASCII_SI_Round9" / f"B2356run{10 + run_idx}.dat",
            tire_name,
            cr_rows,
        )

    loader = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)
    loader.df = loader.df.copy()
    loader.df.loc[loader.df["source_test_type"] == "Db", "source_id"] = [f"db_{idx}" for idx in range(4) for _ in range(2)]
    loader.df.loc[loader.df["source_test_type"] == "Cr", "source_id"] = [f"cr_{idx}" for idx in range(4) for _ in range(2)]
    split = loader._build_split_indices(train_ratio=0.5, val_ratio=0.25, test_ratio=0.25, random_seed=7)

    train_ids = set(loader.df.iloc[split["train"]]["source_id"])
    val_ids = set(loader.df.iloc[split["val"]]["source_id"])
    test_ids = set(loader.df.iloc[split["test"]]["source_id"])

    assert train_ids
    assert val_ids
    assert test_ids
    assert train_ids.isdisjoint(val_ids)
    assert train_ids.isdisjoint(test_ids)
    assert val_ids.isdisjoint(test_ids)


def test_ttc_data_loader_sparse_split_reuses_holdout_for_val_and_test(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    for run_idx in range(2):
        db_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 0.1, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15, 0.0],
        ]
        cr_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        ]
        _write_raw_dat(
            tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / f"B2356run{60 + run_idx}.dat",
            tire_name,
            db_rows,
            include_sl=True,
        )
        _write_raw_dat(
            tmp_path / "RunData_Cornering_ASCII_SI_Round9" / f"B2356run{20 + run_idx}.dat",
            tire_name,
            cr_rows,
        )

    loader = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)
    loader.df = loader.df.copy()
    loader.df.loc[loader.df["source_test_type"] == "Db", "source_id"] = [
        f"db_{idx}" for idx in range(2) for _ in range(2)
    ]
    loader.df.loc[loader.df["source_test_type"] == "Cr", "source_id"] = [
        f"cr_{idx}" for idx in range(2) for _ in range(2)
    ]

    with pytest.raises(ValueError, match="no independent test source groups"):
        loader._build_split_indices(
            train_ratio=0.8, val_ratio=0.1, test_ratio=0.1, random_seed=7
        )


def test_ttc_data_loader_random_split_mixes_source_ids_for_interpolation(tmp_path: Path):
    tire_name = "Hoosier 18.0X6.0-10, R20"
    for run_idx in range(2):
        db_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 0.2, 0.0, 21.0, 21.2, 80.0, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 0.1, 0.0, 21.0, 21.2, 80.0, 120.0, 25.0, -1020.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.15, 0.0],
            [0.2, 40.0 + run_idx, 402.0 + run_idx, 0.0, 0.0, 21.0, 21.2, 80.0, 140.0, 20.0, -1030.0, 1.0, 2.0, 0.1, 0.02, 30.0, 35.0, 40.0, 41.0, 25.0, 0.18, 0.0],
        ]
        cr_rows = [
            [0.0, 40.0 + run_idx, 400.0 + run_idx, 4.0, 0.0, 21.0, 21.2, 80.0, 25.0, 300.0, -1000.0, 1.0, 20.0, 0.02, 0.3, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
            [0.1, 40.0 + run_idx, 401.0 + run_idx, 5.0, 0.0, 21.0, 21.2, 80.0, 20.0, 320.0, -1020.0, 1.0, 22.0, 0.02, 0.31, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
            [0.2, 40.0 + run_idx, 402.0 + run_idx, 6.0, 0.0, 21.0, 21.2, 80.0, 15.0, 340.0, -1030.0, 1.0, 24.0, 0.02, 0.32, 30.0, 35.0, 40.0, 41.0, 25.0, 0.0],
        ]
        _write_raw_dat(
            tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / f"B2356run{70 + run_idx}.dat",
            tire_name,
            db_rows,
            include_sl=True,
        )
        _write_raw_dat(
            tmp_path / "RunData_Cornering_ASCII_SI_Round9" / f"B2356run{30 + run_idx}.dat",
            tire_name,
            cr_rows,
        )

    loader = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)
    split = loader._build_random_split_indices(
        train_ratio=0.5,
        val_ratio=0.25,
        test_ratio=0.25,
        random_seed=3,
    )

    train_ids = set(loader.df.iloc[split["train"]]["source_id"])
    val_ids = set(loader.df.iloc[split["val"]]["source_id"])
    test_ids = set(loader.df.iloc[split["test"]]["source_id"])

    assert split["train_groups"] == []
    assert train_ids.intersection(val_ids.union(test_ids))
    assert val_ids.union(test_ids)


def test_fit_normalization_uses_train_rows_only():
    loader = object.__new__(TTCDataLoader)
    loader.df = pd.DataFrame(
        {
            "SR": [0.0, 0.1, 10.0, 20.0],
            "SA": [0.0, 0.0, 0.0, 0.0],
            "Fz": [100.0, 200.0, 300.0, 400.0],
            "IA": [0.0, 0.0, 0.0, 0.0],
            "TSTC": [20.0, 20.0, 20.0, 20.0],
            "Fx": [10.0, 20.0, 1000.0, 2000.0],
            "Fy": [0.0, 0.0, 1000.0, 2000.0],
            "Mx": [0.0, 0.0, 0.0, 0.0],
            "Mz": [1.0, 1.0, 100.0, 100.0],
        }
    )
    loader.component_mask = np.ones((4, 4), dtype=np.float32)
    loader.fit_normalization(np.array([0, 1]))

    # The held-out rows contain extreme values and must not affect the stats.
    assert loader.input_mean[0] == pytest.approx(0.05)
    assert loader.output_mean[0] == pytest.approx(15.0)
    assert loader.output_mean[3] == pytest.approx(1.0)
    assert loader.output_std[3] == pytest.approx(1.0)


def test_input_bounds_limit_sr_to_fx_supervised_rows():
    loader = object.__new__(TTCDataLoader)
    loader.df = pd.DataFrame(
        {
            "SR": [-0.3, 0.17, 1.5, -1.4],
            "SA": [0.0, 0.0, 5.0, -5.0],
            "Fz": [100.0, 200.0, 300.0, 400.0],
            "IA": [0.0, 0.0, 0.0, 0.0],
            "TSTC": [20.0, 20.0, 20.0, 20.0],
        }
    )
    # Rows 0-1 are drive/brake (Fx supervised); rows 2-3 are cornering noise.
    loader.component_mask = np.array(
        [[1, 0, 1, 1], [1, 0, 1, 1], [0, 1, 1, 1], [0, 1, 1, 1]], dtype=np.float32
    )

    bounds = loader.get_input_bounds()

    assert bounds["SR"] == {"min": pytest.approx(-0.3), "max": pytest.approx(0.17)}
    assert bounds["SA"] == {"min": -5.0, "max": 5.0}


def test_validate_physics_config_rejects_unconsumed_keys():
    from tire_pinn.main_pinn_train import validate_physics_config

    validate_physics_config({"physics": {"zero_slip": {"force_threshold": 2.0}}})
    with pytest.raises(ValueError, match="Unsupported keys"):
        validate_physics_config({"physics": {"zero_slip": {"unknown_knob": 1.0}}})
    with pytest.raises(ValueError, match="Unsupported physics section"):
        validate_physics_config({"physics": {"unknown_section": {}}})


def test_split_metadata_labels_random_split_as_interpolation():
    loader = object.__new__(TTCDataLoader)
    loader.last_split_indices = {
        "train": np.array([0, 1]),
        "val": np.array([2]),
        "test": np.array([3]),
        "train_groups": [],
        "val_groups": [],
        "test_groups": [],
    }
    loader.last_split_config = {
        "train_ratio": 0.5,
        "val_ratio": 0.25,
        "test_ratio": 0.25,
        "random_seed": 7,
        "split_strategy": "random",
    }

    metadata = loader.get_split_metadata()
    assert metadata["interpretation"] == "in_distribution_interpolation"
    assert "not a run-level" in metadata["generalization_scope"]


def test_ttc_data_loader_pressure_filter_keeps_only_running_pressure(tmp_path: Path):
    # #18 stage 1: P column [kPa] selects one inflation pressure; others are dropped.
    tire_name = "Hoosier 18.0X6.0-10, R20"
    base = [0.0, 40.0, 400.0, 0.2, 0.0, 21.0, 21.2, 82.7, 100.0, 30.0, -1000.0, 1.0, 2.0, 0.1, 0.03, 30.0, 35.0, 40.0, 41.0, 25.0, 0.12]
    db_rows = [base, base[:7] + [55.2] + base[8:], base[:7] + [96.5] + base[8:]]
    cr_rows = [base[:3] + [4.0] + base[4:20] + [0.0], base[:3] + [4.0] + base[4:7] + [68.9] + base[8:20] + [0.0]]
    _write_raw_dat(tmp_path / "RunData_DriveBrake_ASCII_SI_Round9" / "B2356run27.dat", tire_name, db_rows)
    _write_raw_dat(tmp_path / "RunData_Cornering_ASCII_SI_Round9" / "B2356run28.dat", tire_name, cr_rows)

    pooled = TTCDataLoader(normalize=False, tire_name="Hoosier_18p0X6p0-10_R20", data_dir=tmp_path)
    assert len(pooled.df) == 5
    assert "P" not in pooled.df.columns

    filtered = TTCDataLoader(
        normalize=False,
        tire_name="Hoosier_18p0X6p0-10_R20",
        data_dir=tmp_path,
        pressure_filter_kpa=82.7,
    )
    assert len(filtered.df) == 2
    assert set(filtered.df["source_test_type"]) == {"Db", "Cr"}

    with pytest.raises(ValueError):
        TTCDataLoader(
            normalize=False,
            tire_name="Hoosier_18p0X6p0-10_R20",
            data_dir=tmp_path,
            pressure_filter_kpa=200.0,
        )


def test_mirror_negative_ia_flips_sa_ia_fy_mz_and_keeps_fx():
    loader = object.__new__(TTCDataLoader)
    x = np.array(
        [
            [0.1, 2.0, 800.0, 4.0, 60.0],
            [0.1, -2.0, 800.0, 4.0, 60.0],
            [0.1, 2.0, 800.0, 0.0, 60.0],
            [0.1, 2.0, 800.0, -2.0, 60.0],
        ],
        dtype=np.float32,
    )
    y = np.array(
        [
            [10.0, 100.0, 5.0, 20.0],
            [20.0, -100.0, 6.0, -20.0],
            [30.0, 50.0, 0.0, 10.0],
            [40.0, 0.0, 0.0, 0.0],
        ],
        dtype=np.float32,
    )
    mask = np.ones((4, 4), dtype=np.float32)

    mirrored_x, mirrored_y, mirrored_mask = loader._mirror_negative_ia(x, y, mask)

    assert mirrored_x.shape == (6, 5)
    assert mirrored_y.shape == (6, 4)
    assert mirrored_mask.shape == (6, 4)
    assert mirrored_x[4, 1] == pytest.approx(-2.0)
    assert mirrored_x[4, 3] == pytest.approx(-4.0)
    assert mirrored_y[4, 0] == pytest.approx(10.0)
    assert mirrored_y[4, 1] == pytest.approx(-100.0)
    assert mirrored_y[4, 2] == pytest.approx(-5.0)
    assert mirrored_y[4, 3] == pytest.approx(-20.0)
    assert mirrored_x[5, 1] == pytest.approx(2.0)
    assert mirrored_x[5, 3] == pytest.approx(-4.0)
    assert mirrored_y[5, 1] == pytest.approx(100.0)
