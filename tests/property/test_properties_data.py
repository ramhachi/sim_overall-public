"""
Property-based tests for data loading and preprocessing

These tests use the Hypothesis library to verify universal properties
across many randomly generated inputs.
"""

import pytest
import pandas as pd
import numpy as np
import tempfile
from pathlib import Path
from hypothesis import given, strategies as st, settings
from tire_fitting.data_loader import TireDataLoader
from tire_fitting.preprocessor import TireDataPreprocessor


class TestDataLoadingProperties:
    """Property-based tests for TireDataLoader"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.loader = TireDataLoader()
    
    @settings(max_examples=50)
    @given(
        fz_values=st.lists(
            st.floats(min_value=-2000, max_value=-100, allow_nan=False, allow_infinity=False),
            min_size=1,
            max_size=100
        )
    )
    def test_property_2_fz_sign_conversion_round_trip(self, fz_values):
        """
        Property 2: FZ Sign Conversion Round Trip
        
        For any CSV file with FZ values, loading the data and then converting 
        FZ signs should result in all FZ values being positive when the original 
        data contained negative values.
        
        **Validates: Requirements 1.3**
        """
        # Create a temporary CSV file with negative FZ values
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            # Write metadata row (will be skipped)
            f.write("TIRF Data File: Test\n")
            # Write all required columns for DriveBrake
            f.write(",".join(self.loader.DRIVEBRAKE_COLUMNS) + "\n")
            # Write units row (will be skipped)
            f.write("s,kph,rpm,deg,deg,cm,cm,kPa,N,N,N,Nm,Nm,none,none,deg c,deg c,deg c,deg c,deg c,SAE,none\n")
            
            # Write data rows with negative FZ values
            for fz in fz_values:
                # Create a valid row with all required columns
                row = [
                    "1.0",  # ET
                    "40.0",  # V
                    "500.0",  # N
                    "0.0",  # SA
                    "0.0",  # IA
                    "21.0",  # RL
                    "19.0",  # RE
                    "80.0",  # P
                    "100.0",  # FX
                    "0.0",  # FY
                    str(fz),  # FZ (negative)
                    "0.0",  # MX
                    "0.0",  # MZ
                    "0.0",  # NFX
                    "0.0",  # NFY
                    "30.0",  # RST
                    "40.0",  # TSTI
                    "50.0",  # TSTC
                    "40.0",  # TSTO
                    "25.0",  # AmbTmp
                    "0.1",  # SR
                    "0.1"  # SL
                ]
                f.write(",".join(row) + "\n")
            
            temp_path = f.name
        
        try:
            # Load the data
            df = self.loader.load_drivebrake_data(temp_path)
            
            # Property: All FZ values should be positive
            assert (df['FZ'] > 0).all(), "All FZ values should be positive after loading"
            
            # Property: The magnitude should be preserved
            for i, original_fz in enumerate(fz_values):
                assert abs(df['FZ'].iloc[i] - abs(original_fz)) < 1e-6, \
                    "FZ magnitude should be preserved"
        finally:
            Path(temp_path).unlink()
    
    @settings(max_examples=30)
    @given(
        missing_cols=st.lists(
            st.sampled_from(TireDataLoader.DRIVEBRAKE_COLUMNS),
            min_size=1,
            max_size=5,
            unique=True
        )
    )
    def test_property_3_missing_column_error_detection(self, missing_cols):
        """
        Property 3: Missing Column Error Detection
        
        For any CSV file missing one or more required columns, the Data_Loader 
        should raise an error that lists all missing columns.
        
        **Validates: Requirements 1.5**
        """
        # Create a list of columns excluding the missing ones
        present_cols = [col for col in self.loader.DRIVEBRAKE_COLUMNS 
                       if col not in missing_cols]
        
        # Skip if no columns remain
        if len(present_cols) == 0:
            return
        
        # Create a temporary CSV file with missing columns
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            # Write only the present columns
            f.write(",".join(present_cols) + "\n")
            
            # Write a data row with values for present columns
            f.write(",".join(["0.0"] * len(present_cols)) + "\n")
            
            temp_path = f.name
        
        try:
            # Property: Should raise ValueError
            with pytest.raises(ValueError) as exc_info:
                self.loader.load_drivebrake_data(temp_path)
            
            # Property: Error message should mention missing columns
            error_msg = str(exc_info.value)
            assert "Missing required columns" in error_msg, \
                "Error message should mention missing columns"
            
            # Property: All missing columns should be listed in the error
            for missing_col in missing_cols:
                assert missing_col in error_msg, \
                    f"Missing column {missing_col} should be listed in error message"
        finally:
            Path(temp_path).unlink()
    
    def test_property_1_csv_data_loading_preserves_all_required_columns(self):
        """
        Property 1: CSV Data Loading Preserves All Required Columns
        
        For any valid DriveBrake CSV file, loading the file should return a 
        DataFrame containing all 22 required columns.
        
        **Validates: Requirements 1.1**
        """
        # Use the actual DriveBrake data file
        filepath = "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
        
        # Load the data
        df = self.loader.load_drivebrake_data(filepath)
        
        # Property: All required columns should be present
        for col in self.loader.DRIVEBRAKE_COLUMNS:
            assert col in df.columns, \
                f"Required column {col} should be present in loaded DataFrame"
        
        # Property: Should have exactly the required columns (or more from the file)
        required_cols_set = set(self.loader.DRIVEBRAKE_COLUMNS)
        actual_cols_set = set(df.columns)
        assert required_cols_set.issubset(actual_cols_set), \
            "All required columns should be present"
    
    def test_property_1_cornering_csv_preserves_all_required_columns(self):
        """
        Property 1: CSV Data Loading Preserves All Required Columns (Cornering)
        
        For any valid Cornering CSV file, loading the file should return a 
        DataFrame containing all 7 required columns.
        
        **Validates: Requirements 1.2**
        """
        # Use the actual Cornering data file
        filepath = "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"
        
        # Load the data
        df = self.loader.load_cornering_data(filepath)
        
        # Property: All required columns should be present
        for col in self.loader.CORNERING_COLUMNS:
            assert col in df.columns, \
                f"Required column {col} should be present in loaded DataFrame"
        
        # Property: Should have exactly the required columns (or more from the file)
        required_cols_set = set(self.loader.CORNERING_COLUMNS)
        actual_cols_set = set(df.columns)
        assert required_cols_set.issubset(actual_cols_set), \
            "All required columns should be present"



class TestDataPreprocessingProperties:
    """Property-based tests for TireDataPreprocessor"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.preprocessor = TireDataPreprocessor()
    
    @settings(max_examples=100)
    @given(
        n_rows=st.integers(min_value=10, max_value=100),
        seed=st.integers(min_value=0, max_value=10000)
    )
    def test_property_4_data_filtering_preserves_only_valid_rows(self, n_rows, seed):
        """
        Property 4: Data Filtering Preserves Only Valid Rows
        
        For any DataFrame with mixed valid and invalid rows, applying all filter 
        conditions (speed >= 30 kph, |FZ| >= 500 N, temperature in range, 
        slip angle/ratio constraints) should result in a DataFrame where every 
        row satisfies all filter conditions.
        
        **Validates: Requirements 2.1, 2.2, 2.3, 2.4, 2.5**
        """
        # Set random seed for reproducibility
        np.random.seed(seed)
        
        # Generate random data with mix of valid and invalid values
        data = {
            'V': np.random.uniform(0, 100, n_rows),  # Speed: 0-100 kph
            'FZ': np.random.uniform(100, 2000, n_rows),  # Load: 100-2000 N
            'SA': np.random.uniform(-5, 5, n_rows),  # Slip angle: -5 to 5 deg
            'SR': np.random.uniform(-0.3, 0.3, n_rows),  # Slip ratio: -0.3 to 0.3
            'TSTC': np.random.uniform(20, 100, n_rows),  # Temperature: 20-100 °C
        }
        df = pd.DataFrame(data)
        
        # Apply filters with default parameters
        filtered = self.preprocessor.filter_data(df)
        
        # Property: All rows should satisfy speed constraint (Requirement 2.1)
        assert (filtered['V'] >= 30.0).all(), \
            "All filtered rows should have V >= 30.0 kph"
        
        # Property: All rows should satisfy vertical load constraint (Requirement 2.2)
        assert (filtered['FZ'].abs() >= 500.0).all(), \
            "All filtered rows should have |FZ| >= 500.0 N"
        
        # Property: All rows should satisfy slip angle constraint (Requirement 2.3)
        assert (filtered['SA'].abs() <= 0.5).all(), \
            "All filtered rows should have |SA| <= 0.5 deg"
        
        # Property: All rows should satisfy slip ratio constraint (Requirement 2.4)
        assert (filtered['SR'].abs() <= 0.01).all(), \
            "All filtered rows should have |SR| <= 0.01"
        
        # Property: All rows should satisfy temperature constraint (Requirement 2.5)
        assert ((filtered['TSTC'] >= 40.0) & (filtered['TSTC'] <= 80.0)).all(), \
            "All filtered rows should have 40.0 <= TSTC <= 80.0 °C"
    
    @settings(max_examples=100)
    @given(
        n_rows=st.integers(min_value=20, max_value=100),
        seed=st.integers(min_value=0, max_value=10000)
    )
    def test_property_5_load_grouping_classification_correctness(self, n_rows, seed):
        """
        Property 5: Load Grouping Classification Correctness
        
        For any DataFrame with FZ values, grouping by load should result in three 
        groups where every row in the Low group has 600 <= FZ <= 900, every row 
        in Medium has 900 <= FZ <= 1100, every row in High has 1100 <= FZ <= 1400, 
        and rows outside these ranges are excluded.
        
        **Validates: Requirements 3.1, 3.2, 3.3, 3.4**
        """
        # Set random seed for reproducibility
        np.random.seed(seed)
        
        # Generate random FZ values covering all ranges including outside ranges
        fz_values = np.random.uniform(400, 1600, n_rows)
        
        # Create DataFrame
        df = pd.DataFrame({'FZ': fz_values})
        
        # Group by load
        groups = self.preprocessor.group_by_load(df)
        
        # Property: Should have three groups
        assert set(groups.keys()) == {'Low', 'Medium', 'High'}, \
            "Should have exactly three groups: Low, Medium, High"
        
        # Property: Low group should have 600 <= FZ <= 900 (Requirement 3.1)
        if len(groups['Low']) > 0:
            assert (groups['Low']['FZ'] >= 600.0).all(), \
                "All Low group rows should have FZ >= 600 N"
            assert (groups['Low']['FZ'] <= 900.0).all(), \
                "All Low group rows should have FZ <= 900 N"
        
        # Property: Medium group should have 900 <= FZ <= 1100 (Requirement 3.2)
        if len(groups['Medium']) > 0:
            assert (groups['Medium']['FZ'] >= 900.0).all(), \
                "All Medium group rows should have FZ >= 900 N"
            assert (groups['Medium']['FZ'] <= 1100.0).all(), \
                "All Medium group rows should have FZ <= 1100 N"
        
        # Property: High group should have 1100 <= FZ <= 1400 (Requirement 3.3)
        if len(groups['High']) > 0:
            assert (groups['High']['FZ'] >= 1100.0).all(), \
                "All High group rows should have FZ >= 1100 N"
            assert (groups['High']['FZ'] <= 1400.0).all(), \
                "All High group rows should have FZ <= 1400 N"
        
        # Property: Total grouped rows should be <= original rows (Requirement 3.4)
        total_grouped = len(groups['Low']) + len(groups['Medium']) + len(groups['High'])
        assert total_grouped <= n_rows, \
            "Total grouped rows should not exceed original row count"
        
        # Property: No row should appear in multiple groups
        # (This is implicitly guaranteed by the implementation, but we verify)
        low_indices = set(groups['Low'].index) if len(groups['Low']) > 0 else set()
        medium_indices = set(groups['Medium'].index) if len(groups['Medium']) > 0 else set()
        high_indices = set(groups['High'].index) if len(groups['High']) > 0 else set()
        
        assert len(low_indices & medium_indices) == 0, \
            "Low and Medium groups should not share rows"
        assert len(low_indices & high_indices) == 0, \
            "Low and High groups should not share rows"
        assert len(medium_indices & high_indices) == 0, \
            "Medium and High groups should not share rows"
    
    @settings(max_examples=100)
    @given(
        n_rows=st.integers(min_value=20, max_value=100),
        seed=st.integers(min_value=0, max_value=10000),
        multiplier=st.floats(min_value=1.0, max_value=3.0)
    )
    def test_property_6_iqr_outlier_removal_completeness(self, n_rows, seed, multiplier):
        """
        Property 6: IQR Outlier Removal Completeness
        
        For any DataFrame column, applying IQR-based outlier removal should result 
        in a DataFrame where no values fall outside the range [Q1 - 1.5*IQR, Q3 + 1.5*IQR].
        
        **Validates: Requirements 4.1, 4.2, 4.3, 4.4**
        """
        # Set random seed for reproducibility
        np.random.seed(seed)
        
        # Generate random data with some outliers
        # Use a mix of normal distribution and extreme values
        normal_data = np.random.normal(100, 20, int(n_rows * 0.9))
        outliers = np.random.uniform(-100, 300, int(n_rows * 0.1))
        data_values = np.concatenate([normal_data, outliers])
        np.random.shuffle(data_values)
        
        # Create DataFrame
        df = pd.DataFrame({'test_column': data_values})
        
        # Calculate expected bounds before filtering
        Q1 = df['test_column'].quantile(0.25)
        Q3 = df['test_column'].quantile(0.75)
        IQR = Q3 - Q1
        lower_bound = Q1 - multiplier * IQR
        upper_bound = Q3 + multiplier * IQR
        
        # Apply outlier removal
        filtered = self.preprocessor.remove_outliers_iqr(df, 'test_column', multiplier)
        
        # Property: All values should be within bounds (Requirements 4.2, 4.3, 4.4)
        if len(filtered) > 0:
            assert (filtered['test_column'] >= lower_bound).all(), \
                f"All values should be >= Q1 - {multiplier}*IQR"
            assert (filtered['test_column'] <= upper_bound).all(), \
                f"All values should be <= Q3 + {multiplier}*IQR"
        
        # Property: Filtered data should be a subset of original data
        assert len(filtered) <= len(df), \
            "Filtered data should not have more rows than original"
        
        # Property: All filtered values should exist in original data
        for val in filtered['test_column']:
            assert val in df['test_column'].values, \
                "All filtered values should exist in original data"
