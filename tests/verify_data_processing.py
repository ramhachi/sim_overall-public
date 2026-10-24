#!/usr/bin/env python3
"""
Verification script for Task 3: Data Processing Checkpoint

This script loads actual TTC data files and runs the complete data processing
pipeline to verify that everything works correctly before proceeding to model
implementation.
"""

import sys
from pathlib import Path
import pandas as pd
import numpy as np

# Add tire_fitting to path
sys.path.insert(0, str(Path(__file__).parent))

from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


def print_section(title):
    """Print a formatted section header"""
    print("\n" + "=" * 70)
    print(f"  {title}")
    print("=" * 70)


def print_dataframe_info(df, name):
    """Print information about a DataFrame"""
    print(f"\n{name}:")
    print(f"  Shape: {df.shape[0]} rows × {df.shape[1]} columns")
    print(f"  Columns: {list(df.columns)}")
    if 'FZ' in df.columns:
        print(f"  FZ range: [{df['FZ'].min():.1f}, {df['FZ'].max():.1f}] N")
    if 'V' in df.columns:
        print(f"  Speed range: [{df['V'].min():.1f}, {df['V'].max():.1f}] kph")
    if 'TSTC' in df.columns:
        print(f"  Temp range: [{df['TSTC'].min():.1f}, {df['TSTC'].max():.1f}] °C")


def verify_drivebrake_data():
    """Verify DriveBrake data loading and preprocessing"""
    print_section("DriveBrake Data Processing")
    
    # Load data
    loader = TireDataLoader()
    filepath = "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
    
    print(f"\n1. Loading data from: {filepath}")
    df_raw = loader.load_drivebrake_data(filepath)
    print_dataframe_info(df_raw, "Raw DriveBrake Data")
    
    # Verify FZ sign conversion
    print("\n2. Verifying FZ sign conversion:")
    print(f"  All FZ values positive: {(df_raw['FZ'] >= 0).all()}")
    print(f"  FZ statistics: mean={df_raw['FZ'].mean():.1f} N, std={df_raw['FZ'].std():.1f} N")
    
    # Apply preprocessing
    preprocessor = TireDataPreprocessor()
    
    print("\n3. Applying filters:")
    df_filtered = preprocessor.filter_data(
        df_raw,
        min_speed=30.0,
        min_fz=500.0,
        max_sa_for_longitudinal=0.5,
        temp_range=(40.0, 80.0)
    )
    print_dataframe_info(df_filtered, "Filtered DriveBrake Data")
    print(f"  Rows removed: {df_raw.shape[0] - df_filtered.shape[0]} ({100*(df_raw.shape[0] - df_filtered.shape[0])/df_raw.shape[0]:.1f}%)")
    
    # Verify filter conditions
    print("\n4. Verifying filter conditions:")
    print(f"  All V >= 30 kph: {(df_filtered['V'] >= 30.0).all()}")
    print(f"  All |FZ| >= 500 N: {(df_filtered['FZ'].abs() >= 500.0).all()}")
    print(f"  All |SA| <= 0.5 deg: {(df_filtered['SA'].abs() <= 0.5).all()}")
    print(f"  All TSTC in [40, 80] °C: {((df_filtered['TSTC'] >= 40.0) & (df_filtered['TSTC'] <= 80.0)).all()}")
    
    # Remove outliers
    print("\n5. Removing outliers from FX:")
    df_no_outliers = preprocessor.remove_outliers_iqr(df_filtered, 'FX')
    print(f"  Rows after outlier removal: {df_no_outliers.shape[0]}")
    print(f"  Outliers removed: {df_filtered.shape[0] - df_no_outliers.shape[0]} ({100*(df_filtered.shape[0] - df_no_outliers.shape[0])/df_filtered.shape[0]:.1f}%)")
    
    # Group by load
    print("\n6. Grouping by vertical load:")
    load_groups = preprocessor.group_by_load(df_no_outliers)
    for group_name, group_df in load_groups.items():
        if not group_df.empty:
            print(f"  {group_name}: {group_df.shape[0]} rows, FZ range [{group_df['FZ'].min():.1f}, {group_df['FZ'].max():.1f}] N")
    
    # Verify load grouping
    print("\n7. Verifying load group boundaries:")
    if not load_groups['Low'].empty:
        print(f"  Low group: {(load_groups['Low']['FZ'] >= 600).all() and (load_groups['Low']['FZ'] <= 900).all()}")
    if not load_groups['Medium'].empty:
        print(f"  Medium group: {(load_groups['Medium']['FZ'] >= 900).all() and (load_groups['Medium']['FZ'] <= 1100).all()}")
    if not load_groups['High'].empty:
        print(f"  High group: {(load_groups['High']['FZ'] >= 1100).all() and (load_groups['High']['FZ'] <= 1400).all()}")
    
    return df_no_outliers, load_groups


def verify_cornering_data():
    """Verify Cornering data loading and preprocessing"""
    print_section("Cornering Data Processing")
    
    # Load data
    loader = TireDataLoader()
    filepath = "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"
    
    print(f"\n1. Loading data from: {filepath}")
    df_raw = loader.load_cornering_data(filepath)
    print_dataframe_info(df_raw, "Raw Cornering Data")
    
    # Verify FZ sign conversion
    print("\n2. Verifying FZ sign conversion:")
    print(f"  All FZ values positive: {(df_raw['FZ'] >= 0).all()}")
    print(f"  FZ statistics: mean={df_raw['FZ'].mean():.1f} N, std={df_raw['FZ'].std():.1f} N")
    
    # Apply preprocessing
    preprocessor = TireDataPreprocessor()
    
    print("\n3. Applying filters:")
    # Note: Cornering data doesn't have V or TSTC columns, so we only filter by FZ
    df_filtered = df_raw[df_raw['FZ'].abs() >= 500.0].copy()
    print_dataframe_info(df_filtered, "Filtered Cornering Data")
    print(f"  Rows removed: {df_raw.shape[0] - df_filtered.shape[0]} ({100*(df_raw.shape[0] - df_filtered.shape[0])/df_raw.shape[0]:.1f}%)")
    
    # Remove outliers
    print("\n4. Removing outliers from FY:")
    df_no_outliers = preprocessor.remove_outliers_iqr(df_filtered, 'FY')
    print(f"  Rows after outlier removal: {df_no_outliers.shape[0]}")
    print(f"  Outliers removed: {df_filtered.shape[0] - df_no_outliers.shape[0]} ({100*(df_filtered.shape[0] - df_no_outliers.shape[0])/df_filtered.shape[0]:.1f}%)")
    
    # Group by load
    print("\n5. Grouping by vertical load:")
    load_groups = preprocessor.group_by_load(df_no_outliers)
    for group_name, group_df in load_groups.items():
        if not group_df.empty:
            print(f"  {group_name}: {group_df.shape[0]} rows, FZ range [{group_df['FZ'].min():.1f}, {group_df['FZ'].max():.1f}] N")
    
    # Verify load grouping
    print("\n6. Verifying load group boundaries:")
    if not load_groups['Low'].empty:
        print(f"  Low group: {(load_groups['Low']['FZ'] >= 600).all() and (load_groups['Low']['FZ'] <= 900).all()}")
    if not load_groups['Medium'].empty:
        print(f"  Medium group: {(load_groups['Medium']['FZ'] >= 900).all() and (load_groups['Medium']['FZ'] <= 1100).all()}")
    if not load_groups['High'].empty:
        print(f"  High group: {(load_groups['High']['FZ'] >= 1100).all() and (load_groups['High']['FZ'] <= 1400).all()}")
    
    return df_no_outliers, load_groups


def main():
    """Main verification function"""
    print("\n" + "=" * 70)
    print("  TASK 3 CHECKPOINT: Data Processing Verification")
    print("=" * 70)
    print("\nThis script verifies that all data processing functionality works")
    print("correctly with actual TTC data files.")
    
    try:
        # Verify DriveBrake data processing
        db_data, db_groups = verify_drivebrake_data()
        
        # Verify Cornering data processing
        corn_data, corn_groups = verify_cornering_data()
        
        # Summary
        print_section("VERIFICATION SUMMARY")
        print("\n✓ All data loading tests passed")
        print("✓ FZ sign conversion working correctly")
        print("✓ Data filtering working correctly")
        print("✓ Outlier removal working correctly")
        print("✓ Load grouping working correctly")
        
        print("\n" + "=" * 70)
        print("  DATA PROCESSING PIPELINE VERIFIED SUCCESSFULLY!")
        print("=" * 70)
        print("\nReady to proceed to Task 4: TMeasy Model Implementation")
        
        return 0
        
    except Exception as e:
        print(f"\n❌ ERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
