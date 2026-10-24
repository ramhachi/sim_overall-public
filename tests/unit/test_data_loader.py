"""
Unit tests for TireDataLoader class
"""

import pytest
import pandas as pd
import tempfile
from pathlib import Path
from tire_fitting.data_loader import TireDataLoader


class TestTireDataLoader:
    """Unit tests for TireDataLoader"""
    
    def setup_method(self):
        """Set up test fixtures"""
        self.loader = TireDataLoader()
    
    def test_load_drivebrake_data_success(self):
        """Test loading valid DriveBrake CSV file"""
        # Use the actual data file
        filepath = "data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv"
        
        df = self.loader.load_drivebrake_data(filepath)
        
        # Check that all required columns are present
        for col in self.loader.DRIVEBRAKE_COLUMNS:
            assert col in df.columns, f"Missing column: {col}"
        
        # Check that FZ values are positive
        assert (df['FZ'] > 0).all(), "FZ values should be positive"
        
        # Check that we have data
        assert len(df) > 0, "DataFrame should not be empty"
    
    def test_load_cornering_data_success(self):
        """Test loading valid Cornering CSV file"""
        # Use the actual data file
        filepath = "data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv"
        
        df = self.loader.load_cornering_data(filepath)
        
        # Check that all required columns are present
        for col in self.loader.CORNERING_COLUMNS:
            assert col in df.columns, f"Missing column: {col}"
        
        # Check that FZ values are positive
        assert (df['FZ'] > 0).all(), "FZ values should be positive"
        
        # Check that we have data
        assert len(df) > 0, "DataFrame should not be empty"
    
    def test_load_drivebrake_file_not_found(self):
        """Test that FileNotFoundError is raised for non-existent file"""
        with pytest.raises(FileNotFoundError) as exc_info:
            self.loader.load_drivebrake_data("nonexistent_file.csv")
        
        assert "CSV file not found" in str(exc_info.value)
    
    def test_load_cornering_file_not_found(self):
        """Test that FileNotFoundError is raised for non-existent file"""
        with pytest.raises(FileNotFoundError) as exc_info:
            self.loader.load_cornering_data("nonexistent_file.csv")
        
        assert "CSV file not found" in str(exc_info.value)
    
    def test_load_drivebrake_missing_columns(self):
        """Test that ValueError is raised when required columns are missing"""
        # Create a temporary CSV file with missing columns
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            f.write("ET,V,N,SA\n")
            f.write("1.0,40.0,500.0,0.0\n")
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError) as exc_info:
                self.loader.load_drivebrake_data(temp_path)
            
            assert "Missing required columns" in str(exc_info.value)
        finally:
            Path(temp_path).unlink()
    
    def test_load_cornering_missing_columns(self):
        """Test that ValueError is raised when required columns are missing"""
        # Create a temporary CSV file with missing columns
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            f.write("SA,IA,FY\n")
            f.write("0.0,0.0,100.0\n")
            temp_path = f.name
        
        try:
            with pytest.raises(ValueError) as exc_info:
                self.loader.load_cornering_data(temp_path)
            
            assert "Missing required columns" in str(exc_info.value)
        finally:
            Path(temp_path).unlink()
    
    def test_fz_sign_conversion_drivebrake(self):
        """Test that FZ values are converted from negative to positive"""
        # Create a temporary CSV file with negative FZ values
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            # Write metadata row (will be skipped)
            f.write("TIRF Data File: Test\n")
            # Write all required columns
            f.write(",".join(self.loader.DRIVEBRAKE_COLUMNS) + "\n")
            # Write units row (will be skipped)
            f.write("s,kph,rpm,deg,deg,cm,cm,kPa,N,N,N,Nm,Nm,none,none,deg c,deg c,deg c,deg c,deg c,SAE,none\n")
            # Write a data row with negative FZ
            f.write("1.0,40.0,500.0,0.0,0.0,21.0,19.0,80.0,100.0,0.0,-1000.0,0.0,0.0,0.0,0.0,30.0,40.0,50.0,40.0,25.0,0.1,0.1\n")
            temp_path = f.name
        
        try:
            df = self.loader.load_drivebrake_data(temp_path)
            
            # Check that FZ is now positive
            assert df['FZ'].iloc[0] == 1000.0, "FZ should be converted to positive"
        finally:
            Path(temp_path).unlink()
    
    def test_fz_sign_conversion_cornering(self):
        """Test that FZ values are converted from negative to positive"""
        # Create a temporary CSV file with negative FZ values
        with tempfile.NamedTemporaryFile(mode='w', suffix='.csv', delete=False) as f:
            # Write all required columns
            f.write(",".join(self.loader.CORNERING_COLUMNS) + "\n")
            # Write units row (will be skipped)
            f.write("deg,deg,N,N,none,Nm,N/deg\n")
            # Write a data row with negative FZ
            f.write("0.0,0.0,100.0,-1000.0,0.0,0.0,0.0\n")
            temp_path = f.name
        
        try:
            df = self.loader.load_cornering_data(temp_path)
            
            # Check that FZ is now positive
            assert df['FZ'].iloc[0] == 1000.0, "FZ should be converted to positive"
        finally:
            Path(temp_path).unlink()
