"""
Data Loader Module

This module provides the TireDataLoader class for loading tire test data
from CSV files provided by the FSAE Tire Test Consortium (TTC).
"""

import logging
from pathlib import Path
from typing import Dict, List
import re

import numpy as np
import pandas as pd

from tire_pinn.data.raw_ttc import parse_raw_tirf_metadata


class TireDataLoader:
    """
    Loads tire test data from CSV files.
    
    This class handles loading DriveBrake (longitudinal) and Cornering (lateral)
    test data, performs FZ sign conversion (negative to positive), and validates
    that all required columns are present.
    """
    
    def __init__(self):
        """Initialize the data loader with logging."""
        self.logger = logging.getLogger(__name__)
    
    # Required columns for DriveBrake data
    DRIVEBRAKE_COLUMNS = [
        'ET', 'V', 'N', 'SA', 'IA', 'RL', 'RE', 'P',
        'FX', 'FY', 'FZ', 'MX', 'MZ', 'NFX', 'NFY',
        'RST', 'TSTI', 'TSTC', 'TSTO', 'AmbTmp', 'SR', 'SL'
    ]

    # Required columns for Cornering data
    CORNERING_COLUMNS = ['SA', 'IA', 'FY', 'FZ', 'NFY', 'MZ', 'CP']

    OPTIONAL_COLUMNS: Dict[str, Dict[str, float]] = {
        'drivebrake': {
            'SL': 0.0,
        },
        'cornering': {
            'NFY': 0.0,
            'CP': 0.0,
            'SR': 0.0,
        },
    }

    def _normalize_legacy_tire_name(self, raw_name: str) -> str:
        normalized = raw_name.strip().replace(' ', '_').replace(',', '')
        normalized = normalized.replace('.', 'p').replace('×', 'X').replace('x', 'X')
        normalized = normalized.replace('-R', '_R')
        normalized = re.sub(r'^([A-Za-z]+)(\d)', r'\1_\2', normalized)
        return normalized

    def _resolve_candidate_paths(self, filepath: str, prefix: str) -> List[Path]:
        file_path = Path(filepath)
        if file_path.exists():
            return [file_path]

        repo_root = Path(__file__).resolve().parents[1]
        csv_dir = repo_root / 'data' / 'tiredata' / 'csv'

        stem = file_path.stem
        legacy_prefix = 'DriveBrake-' if prefix == 'Db' else 'Cornering-'
        if stem.startswith(legacy_prefix):
            tire_name = stem[len(legacy_prefix):]
            identifier = self._normalize_legacy_tire_name(tire_name)
            if csv_dir.exists():
                matches = sorted(csv_dir.glob(f"{prefix}_{identifier}_Run*.csv"))
                if matches:
                    return matches
            raw_matches = self._resolve_raw_dat_paths(repo_root / 'data' / 'tiredata', identifier, prefix)
            if raw_matches:
                return raw_matches

        raise FileNotFoundError(f"CSV file not found: {filepath}")

    def _resolve_raw_dat_paths(self, data_dir: Path, identifier: str, prefix: str) -> List[Path]:
        if not data_dir.exists():
            return []

        matches: List[Path] = []
        for path in sorted(data_dir.rglob('*.dat')):
            if any(part.startswith('.') for part in path.parts):
                continue
            metadata = parse_raw_tirf_metadata(path)
            if metadata.tire_key == identifier and metadata.test_type == prefix:
                matches.append(path)
        return matches

    def _read_ttc_file(self, filepath: Path) -> pd.DataFrame:
        if filepath.suffix.lower() == '.dat':
            return pd.read_csv(filepath, sep=r'\s+', engine='python', skiprows=[0, 2])

        with filepath.open('r', encoding='utf-8', errors='ignore') as handle:
            first_line = handle.readline().strip()
            second_line = handle.readline().strip()

        first_line_fields = first_line.split(',')
        structured_csv = (
            len(first_line_fields) >= 5
            and {'SA', 'IA', 'FZ'}.issubset(set(first_line_fields))
            and second_line
        )

        if structured_csv:
            return pd.read_csv(filepath, skiprows=[1])

        return pd.read_csv(filepath, skiprows=[0, 2])

    def _prepare_frame(
        self,
        df: pd.DataFrame,
        required_columns: List[str],
        optional_columns: Dict[str, float],
    ) -> pd.DataFrame:
        prepared = df.copy()
        for column, default_value in optional_columns.items():
            if column not in prepared.columns:
                prepared[column] = default_value

        missing_columns = [col for col in required_columns if col not in prepared.columns]
        if missing_columns:
            self.logger.error(f"Missing required columns: {missing_columns}")
            raise ValueError(f"Missing required columns: {missing_columns}")

        prepared['FZ'] = pd.to_numeric(prepared['FZ'], errors='coerce').abs()
        return prepared
    
    def load_drivebrake_data(self, filepath: str) -> pd.DataFrame:
        """
        Load DriveBrake test data from CSV file.
        
        DriveBrake data contains longitudinal force measurements at various
        slip ratios, vertical loads, and operating conditions.
        
        Args:
            filepath: Path to the DriveBrake CSV file
            
        Returns:
            DataFrame with columns: ET, V, N, SA, IA, RL, RE, P, FX, FY, FZ, 
            MX, MZ, NFX, NFY, RST, TSTI, TSTC, TSTO, AmbTmp, SR, SL
            FZ values are converted to positive (original data is negative)
            
        Raises:
            FileNotFoundError: If the file does not exist
            ValueError: If required columns are missing
        """
        self.logger.debug(f"Loading DriveBrake data from: {filepath}")
        
        paths = self._resolve_candidate_paths(filepath, 'Db')
        optional_columns = (
            self.OPTIONAL_COLUMNS['drivebrake']
            if any(path.name.startswith('Db_') or path.suffix.lower() == '.dat' for path in paths)
            else {}
        )

        try:
            frames = [
                self._prepare_frame(
                    self._read_ttc_file(path),
                    self.DRIVEBRAKE_COLUMNS,
                    optional_columns,
                )
                for path in paths
            ]
        except Exception as e:
            self.logger.error(f"Failed to parse DriveBrake CSV file: {str(e)}")
            raise ValueError(f"Invalid CSV format: unable to parse file - {str(e)}")

        df = pd.concat(frames, ignore_index=True)
        self.logger.info(f"Loaded DriveBrake data: {len(df)} data points from {len(paths)} file(s)")
        return df
    
    def load_cornering_data(self, filepath: str) -> pd.DataFrame:
        """
        Load Cornering test data from CSV file.
        
        Cornering data contains lateral force measurements at various
        slip angles, vertical loads, and camber angles.
        
        Args:
            filepath: Path to the Cornering CSV file
            
        Returns:
            DataFrame with columns: SA, IA, FY, FZ, NFY, MZ, CP
            FZ values are converted to positive (original data is negative)
            
        Raises:
            FileNotFoundError: If the file does not exist
            ValueError: If required columns are missing
        """
        self.logger.debug(f"Loading Cornering data from: {filepath}")
        
        paths = self._resolve_candidate_paths(filepath, 'Cr')
        optional_columns = (
            self.OPTIONAL_COLUMNS['cornering']
            if any(path.name.startswith('Cr_') or path.suffix.lower() == '.dat' for path in paths)
            else {}
        )

        try:
            frames = [
                self._prepare_frame(
                    self._read_ttc_file(path),
                    self.CORNERING_COLUMNS,
                    optional_columns,
                )
                for path in paths
            ]
        except Exception as e:
            self.logger.error(f"Failed to parse Cornering CSV file: {str(e)}")
            raise ValueError(f"Invalid CSV format: unable to parse file - {str(e)}")

        df = pd.concat(frames, ignore_index=True)
        self.logger.info(f"Loaded Cornering data: {len(df)} data points from {len(paths)} file(s)")
        return df
