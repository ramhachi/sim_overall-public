"""
Data Preprocessor Module

This module provides the TireDataPreprocessor class for filtering, grouping,
and removing outliers from tire test data.
"""

import pandas as pd
import numpy as np
import logging
from typing import Dict, Tuple


class TireDataPreprocessor:
    """
    Preprocesses tire test data through filtering, grouping, and outlier removal.
    
    This class handles:
    - Filtering data by speed, vertical load, slip angle/ratio, and temperature
    - Grouping data by vertical load levels (Low/Medium/High)
    - Removing outliers using the IQR (Interquartile Range) method
    """
    
    def __init__(self):
        """Initialize the preprocessor with logging."""
        self.logger = logging.getLogger(__name__)
    
    def filter_data(self, 
                   df: pd.DataFrame, 
                   min_speed: float = 30.0,
                   min_fz: float = 500.0,
                   max_sa_for_longitudinal: float = 0.5,
                   max_sr_for_lateral: float = 0.01,
                   temp_range: Tuple[float, float] = (40.0, 80.0)) -> pd.DataFrame:
        """
        Filter tire test data based on operating conditions.
        
        This method applies multiple filters to ensure data quality:
        - Minimum vehicle speed (to avoid low-speed singularities)
        - Minimum vertical load (to exclude near-zero load conditions)
        - Maximum slip angle for longitudinal data (pure longitudinal conditions)
        - Maximum slip ratio for lateral data (pure lateral conditions)
        - Temperature range (to ensure consistent tire conditions)
        
        Args:
            df: Input DataFrame containing tire test data
            min_speed: Minimum vehicle speed [kph] (default: 30.0)
            min_fz: Minimum vertical load magnitude [N] (default: 500.0)
            max_sa_for_longitudinal: Maximum slip angle magnitude for longitudinal 
                                    data [deg] (default: 0.5)
            max_sr_for_lateral: Maximum slip ratio magnitude for lateral data [-] 
                               (default: 0.1)
            temp_range: Tire temperature range [°C] as (min, max) tuple 
                       (default: (40.0, 80.0))
            
        Returns:
            Filtered DataFrame containing only rows that satisfy all conditions
            
        Notes:
            - For DriveBrake data, SA (slip angle) filter is applied
            - For Cornering data, SR (slip ratio) filter is applied if SR column exists
            - Temperature filter uses TSTC column if available
        """
        self.logger.debug(f"Filtering data: initial count = {len(df)}")
        
        # Start with a copy to avoid modifying the original
        filtered_df = df.copy()
        initial_count = len(filtered_df)
        
        # Filter 1: Minimum vehicle speed (Requirement 2.1)
        if 'V' in filtered_df.columns:
            before = len(filtered_df)
            filtered_df = filtered_df[filtered_df['V'] >= min_speed]
            after = len(filtered_df)
            self.logger.debug(f"Speed filter (V >= {min_speed} kph): {before} -> {after} points")
        
        # Filter 2: Minimum vertical load (Requirement 2.2)
        if 'FZ' in filtered_df.columns:
            before = len(filtered_df)
            filtered_df = filtered_df[filtered_df['FZ'].abs() >= min_fz]
            after = len(filtered_df)
            self.logger.debug(f"Load filter (|FZ| >= {min_fz} N): {before} -> {after} points")
        
        # Filter 3: Slip angle constraint for pure longitudinal conditions (Requirement 2.3)
        if 'SA' in filtered_df.columns:
            before = len(filtered_df)
            filtered_df = filtered_df[filtered_df['SA'].abs() <= max_sa_for_longitudinal]
            after = len(filtered_df)
            self.logger.debug(f"Slip angle filter (|SA| <= {max_sa_for_longitudinal} deg): {before} -> {after} points")
        
        # Filter 4: Slip ratio constraint for pure lateral conditions (Requirement 2.4)
        if 'SR' in filtered_df.columns:
            before = len(filtered_df)
            filtered_df = filtered_df[filtered_df['SR'].abs() <= max_sr_for_lateral]
            after = len(filtered_df)
            self.logger.debug(f"Slip ratio filter (|SR| <= {max_sr_for_lateral}): {before} -> {after} points")
        
        # Filter 5: Temperature range (Requirement 2.5)
        if 'TSTC' in filtered_df.columns:
            temp_min, temp_max = temp_range
            before = len(filtered_df)
            filtered_df = filtered_df[
                (filtered_df['TSTC'] >= temp_min) & 
                (filtered_df['TSTC'] <= temp_max)
            ]
            after = len(filtered_df)
            self.logger.debug(f"Temperature filter ({temp_min}-{temp_max} °C): {before} -> {after} points")
        
        final_count = len(filtered_df)
        retention_pct = (final_count / initial_count * 100) if initial_count > 0 else 0
        self.logger.info(f"Filtering complete: {initial_count} -> {final_count} points ({retention_pct:.1f}% retained)")
        
        return filtered_df
    
    def group_by_load(self, df: pd.DataFrame) -> Dict[str, pd.DataFrame]:
        """
        Group tire data by vertical load levels.
        
        This method classifies data into three load groups based on vertical load:
        - Low: 600N to 900N
        - Medium: 900N to 1100N
        - High: 1100N to 1400N
        
        Rows with FZ outside these ranges are excluded from the grouped output.
        
        Args:
            df: Input DataFrame containing tire test data with FZ column
            
        Returns:
            Dictionary with keys 'Low', 'Medium', 'High' mapping to DataFrames
            containing data for each load group. Groups with no data will have
            empty DataFrames.
            
        Raises:
            ValueError: If FZ column is not present in the DataFrame
            
        Notes:
            - Load boundaries follow Requirements 3.1, 3.2, 3.3
            - Data outside all ranges is excluded per Requirement 3.4
        """
        if 'FZ' not in df.columns:
            self.logger.error("Cannot group by load: FZ column not found")
            raise ValueError("DataFrame must contain 'FZ' column for load grouping")
        
        self.logger.debug(f"Grouping {len(df)} data points by vertical load")
        
        # Define load ranges (Requirements 3.1, 3.2, 3.3)
        load_ranges = {
            'Low': (600.0, 900.0),
            'Medium': (900.0, 1100.0),
            'High': (1100.0, 1400.0)
        }
        
        # Group data by load level
        groups = {}
        for group_name, (fz_min, fz_max) in load_ranges.items():
            # Use inclusive bounds on both ends
            mask = (df['FZ'] >= fz_min) & (df['FZ'] <= fz_max)
            groups[group_name] = df[mask].copy()
            self.logger.debug(f"  {group_name} load ({fz_min}-{fz_max} N): {len(groups[group_name])} points")
        
        total_grouped = sum(len(g) for g in groups.values())
        excluded = len(df) - total_grouped
        if excluded > 0:
            self.logger.debug(f"  Excluded {excluded} points outside all load ranges")
        
        self.logger.info(f"Load grouping complete: Low={len(groups['Low'])}, "
                        f"Medium={len(groups['Medium'])}, High={len(groups['High'])}")
        
        return groups
    
    def remove_outliers_iqr(self, 
                           df: pd.DataFrame, 
                           column: str, 
                           multiplier: float = 1.5) -> pd.DataFrame:
        """
        Remove outliers from data using the IQR (Interquartile Range) method.
        
        The IQR method identifies outliers as values that fall outside the range:
        [Q1 - multiplier * IQR, Q3 + multiplier * IQR]
        
        where:
        - Q1 is the first quartile (25th percentile)
        - Q3 is the third quartile (75th percentile)
        - IQR = Q3 - Q1 (interquartile range)
        - multiplier is typically 1.5 for standard outlier detection
        
        Args:
            df: Input DataFrame containing the data
            column: Name of the column to check for outliers
            multiplier: IQR multiplier for outlier bounds (default: 1.5)
            
        Returns:
            DataFrame with outlier rows removed
            
        Raises:
            ValueError: If the specified column is not present in the DataFrame
            
        Notes:
            - Implements Requirements 4.1, 4.2, 4.3, 4.4
            - Standard multiplier of 1.5 is used for typical outlier detection
            - Multiplier of 3.0 would identify only extreme outliers
        """
        if column not in df.columns:
            self.logger.error(f"Cannot remove outliers: column '{column}' not found")
            raise ValueError(f"Column '{column}' not found in DataFrame")
        
        initial_count = len(df)
        self.logger.debug(f"Removing outliers from column '{column}' using IQR method (multiplier={multiplier})")
        
        # Calculate quartiles and IQR (Requirement 4.1)
        Q1 = df[column].quantile(0.25)
        Q3 = df[column].quantile(0.75)
        IQR = Q3 - Q1
        
        # Calculate outlier bounds (Requirements 4.2, 4.3)
        lower_bound = Q1 - multiplier * IQR
        upper_bound = Q3 + multiplier * IQR
        
        self.logger.debug(f"  Q1={Q1:.2f}, Q3={Q3:.2f}, IQR={IQR:.2f}")
        self.logger.debug(f"  Outlier bounds: [{lower_bound:.2f}, {upper_bound:.2f}]")
        
        # Remove outliers (Requirement 4.4)
        filtered_df = df[
            (df[column] >= lower_bound) & 
            (df[column] <= upper_bound)
        ].copy()
        
        final_count = len(filtered_df)
        removed = initial_count - final_count
        retention_pct = (final_count / initial_count * 100) if initial_count > 0 else 0
        
        self.logger.info(f"Outlier removal complete: {initial_count} -> {final_count} points "
                        f"({removed} outliers removed, {retention_pct:.1f}% retained)")
        
        return filtered_df
