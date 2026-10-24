"""
Unit tests for TireDataPreprocessor class
"""

import pytest
import pandas as pd
import numpy as np
from tire_fitting.preprocessor import TireDataPreprocessor


class TestTireDataPreprocessor:
    """Unit tests for TireDataPreprocessor"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.preprocessor = TireDataPreprocessor()
    
    def test_filter_data_speed_constraint(self):
        """Test that speed filter removes rows with V < 30 kph"""
        df = pd.DataFrame({
            'V': [20.0, 30.0, 40.0, 50.0],
            'FZ': [1000.0, 1000.0, 1000.0, 1000.0],
            'SA': [0.0, 0.0, 0.0, 0.0],
            'SR': [0.0, 0.0, 0.0, 0.0],
            'TSTC': [50.0, 50.0, 50.0, 50.0]
        })
        
        filtered = self.preprocessor.filter_data(df, min_speed=30.0)
        
        # Should exclude first row (V=20)
        assert len(filtered) == 3
        assert (filtered['V'] >= 30.0).all()
    
    def test_filter_data_vertical_load_constraint(self):
        """Test that vertical load filter removes rows with |FZ| < 500 N"""
        df = pd.DataFrame({
            'V': [40.0, 40.0, 40.0, 40.0],
            'FZ': [400.0, 500.0, 600.0, 1000.0],
            'SA': [0.0, 0.0, 0.0, 0.0],
            'SR': [0.0, 0.0, 0.0, 0.0],
            'TSTC': [50.0, 50.0, 50.0, 50.0]
        })
        
        filtered = self.preprocessor.filter_data(df, min_fz=500.0)
        
        # Should exclude first row (FZ=400)
        assert len(filtered) == 3
        assert (filtered['FZ'] >= 500.0).all()
    
    def test_filter_data_slip_angle_constraint(self):
        """Test that slip angle filter removes rows with |SA| > 0.5 deg"""
        df = pd.DataFrame({
            'V': [40.0, 40.0, 40.0, 40.0],
            'FZ': [1000.0, 1000.0, 1000.0, 1000.0],
            'SA': [-1.0, -0.5, 0.0, 0.5],
            'SR': [0.0, 0.0, 0.0, 0.0],
            'TSTC': [50.0, 50.0, 50.0, 50.0]
        })
        
        filtered = self.preprocessor.filter_data(df, max_sa_for_longitudinal=0.5)
        
        # Should exclude first row (SA=-1.0)
        assert len(filtered) == 3
        assert (filtered['SA'].abs() <= 0.5).all()
    
    def test_filter_data_slip_ratio_constraint(self):
        """Test that slip ratio filter removes rows with |SR| > 0.01"""
        df = pd.DataFrame({
            'V': [40.0, 40.0, 40.0, 40.0],
            'FZ': [1000.0, 1000.0, 1000.0, 1000.0],
            'SA': [0.0, 0.0, 0.0, 0.0],
            'SR': [-0.05, -0.01, 0.0, 0.01],
            'TSTC': [50.0, 50.0, 50.0, 50.0]
        })
        
        filtered = self.preprocessor.filter_data(df, max_sr_for_lateral=0.01)
        
        # Should exclude first row (SR=-0.05)
        assert len(filtered) == 3
        assert (filtered['SR'].abs() <= 0.01).all()
    
    def test_filter_data_temperature_constraint(self):
        """Test that temperature filter removes rows outside 40-80°C range"""
        df = pd.DataFrame({
            'V': [40.0, 40.0, 40.0, 40.0, 40.0],
            'FZ': [1000.0, 1000.0, 1000.0, 1000.0, 1000.0],
            'SA': [0.0, 0.0, 0.0, 0.0, 0.0],
            'SR': [0.0, 0.0, 0.0, 0.0, 0.0],
            'TSTC': [30.0, 40.0, 50.0, 80.0, 90.0]
        })
        
        filtered = self.preprocessor.filter_data(df, temp_range=(40.0, 80.0))
        
        # Should exclude first and last rows (TSTC=30 and TSTC=90)
        assert len(filtered) == 3
        assert (filtered['TSTC'] >= 40.0).all()
        assert (filtered['TSTC'] <= 80.0).all()
    
    def test_filter_data_combined_constraints(self):
        """Test that all filters work together correctly"""
        df = pd.DataFrame({
            'V': [20.0, 40.0, 40.0, 40.0],
            'FZ': [1000.0, 400.0, 1000.0, 1000.0],
            'SA': [0.0, 0.0, 2.0, 0.0],
            'SR': [0.0, 0.0, 0.0, 0.0],
            'TSTC': [50.0, 50.0, 50.0, 50.0]
        })
        
        filtered = self.preprocessor.filter_data(df)
        
        # Should only keep last row
        assert len(filtered) == 1
        assert filtered.iloc[0]['V'] == 40.0
        assert filtered.iloc[0]['FZ'] == 1000.0
        assert filtered.iloc[0]['SA'] == 0.0

    def test_group_by_load_low_group(self):
        """Test that Low group contains FZ values between 600-900N"""
        df = pd.DataFrame({
            'FZ': [600.0, 700.0, 800.0, 900.0]
        })
        
        groups = self.preprocessor.group_by_load(df)
        
        assert len(groups['Low']) == 4
        assert (groups['Low']['FZ'] >= 600.0).all()
        assert (groups['Low']['FZ'] <= 900.0).all()
    
    def test_group_by_load_medium_group(self):
        """Test that Medium group contains FZ values between 900-1100N"""
        df = pd.DataFrame({
            'FZ': [900.0, 1000.0, 1100.0]
        })
        
        groups = self.preprocessor.group_by_load(df)
        
        assert len(groups['Medium']) == 3
        assert (groups['Medium']['FZ'] >= 900.0).all()
        assert (groups['Medium']['FZ'] <= 1100.0).all()
    
    def test_group_by_load_high_group(self):
        """Test that High group contains FZ values between 1100-1400N"""
        df = pd.DataFrame({
            'FZ': [1100.0, 1200.0, 1300.0, 1400.0]
        })
        
        groups = self.preprocessor.group_by_load(df)
        
        assert len(groups['High']) == 4
        assert (groups['High']['FZ'] >= 1100.0).all()
        assert (groups['High']['FZ'] <= 1400.0).all()
    
    def test_group_by_load_boundary_values(self):
        """Test that boundary values are handled correctly"""
        # 900N should be in both Low and Medium
        # 1100N should be in both Medium and High
        df = pd.DataFrame({
            'FZ': [600.0, 900.0, 1100.0, 1400.0]
        })
        
        groups = self.preprocessor.group_by_load(df)
        
        # Check that 900N appears in both Low and Medium
        assert 900.0 in groups['Low']['FZ'].values
        assert 900.0 in groups['Medium']['FZ'].values
        
        # Check that 1100N appears in both Medium and High
        assert 1100.0 in groups['Medium']['FZ'].values
        assert 1100.0 in groups['High']['FZ'].values
    
    def test_group_by_load_excludes_out_of_range(self):
        """Test that values outside all ranges are excluded"""
        df = pd.DataFrame({
            'FZ': [500.0, 700.0, 1000.0, 1300.0, 1500.0]
        })
        
        groups = self.preprocessor.group_by_load(df)
        
        # 500N and 1500N should be excluded
        total_grouped = len(groups['Low']) + len(groups['Medium']) + len(groups['High'])
        assert total_grouped == 3  # Only 700, 1000, 1300
    
    def test_group_by_load_missing_fz_column(self):
        """Test that ValueError is raised when FZ column is missing"""
        df = pd.DataFrame({
            'V': [40.0, 50.0]
        })
        
        with pytest.raises(ValueError) as exc_info:
            self.preprocessor.group_by_load(df)
        
        assert "FZ" in str(exc_info.value)
    
    def test_remove_outliers_iqr_basic(self):
        """Test basic IQR outlier removal"""
        # Create data with clear outliers
        df = pd.DataFrame({
            'value': [1, 2, 3, 4, 5, 6, 7, 8, 9, 100]  # 100 is an outlier
        })
        
        filtered = self.preprocessor.remove_outliers_iqr(df, 'value', multiplier=1.5)
        
        # Should remove the outlier (100)
        assert 100 not in filtered['value'].values
        assert len(filtered) < len(df)
    
    def test_remove_outliers_iqr_calculation(self):
        """Test that IQR bounds are calculated correctly"""
        # Use known data to verify calculation
        df = pd.DataFrame({
            'value': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
        })
        
        Q1 = df['value'].quantile(0.25)  # 2.75
        Q3 = df['value'].quantile(0.75)  # 7.25
        IQR = Q3 - Q1  # 4.5
        lower_bound = Q1 - 1.5 * IQR  # -4.0
        upper_bound = Q3 + 1.5 * IQR  # 14.0
        
        filtered = self.preprocessor.remove_outliers_iqr(df, 'value', multiplier=1.5)
        
        # All values should be within bounds
        assert (filtered['value'] >= lower_bound).all()
        assert (filtered['value'] <= upper_bound).all()
    
    def test_remove_outliers_iqr_custom_multiplier(self):
        """Test IQR with custom multiplier"""
        df = pd.DataFrame({
            'value': [1, 2, 3, 4, 5, 6, 7, 8, 9, 50]
        })
        
        # With multiplier=3.0, fewer outliers should be removed
        filtered_strict = self.preprocessor.remove_outliers_iqr(df, 'value', multiplier=1.5)
        filtered_lenient = self.preprocessor.remove_outliers_iqr(df, 'value', multiplier=3.0)
        
        # Lenient filter should keep more data
        assert len(filtered_lenient) >= len(filtered_strict)
    
    def test_remove_outliers_iqr_missing_column(self):
        """Test that ValueError is raised when column is missing"""
        df = pd.DataFrame({
            'value': [1, 2, 3, 4, 5]
        })
        
        with pytest.raises(ValueError) as exc_info:
            self.preprocessor.remove_outliers_iqr(df, 'nonexistent', multiplier=1.5)
        
        assert "nonexistent" in str(exc_info.value)
    
    def test_remove_outliers_iqr_no_outliers(self):
        """Test that data without outliers is preserved"""
        # Create uniform data without outliers
        df = pd.DataFrame({
            'value': [5, 6, 7, 8, 9, 10, 11, 12, 13, 14]
        })
        
        filtered = self.preprocessor.remove_outliers_iqr(df, 'value', multiplier=1.5)
        
        # All data should be preserved
        assert len(filtered) == len(df)
