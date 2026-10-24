"""
Verification script for export functionality (Checkpoint 9)

This script verifies that:
1. YAML files are correctly generated and can be parsed
2. C++ header files have valid syntax
3. All required fields are present in both formats
"""

import yaml
import subprocess
import tempfile
from pathlib import Path
from tire_fitting.exporter import ParameterExporter


def verify_yaml_export():
    """Verify YAML export functionality"""
    print("=" * 60)
    print("YAML Export Verification")
    print("=" * 60)
    
    # Check existing YAML file
    yaml_path = Path("output/tire_params/hoosier_18x6_r20.yaml")
    if yaml_path.exists():
        print(f"✓ YAML file exists: {yaml_path}")
        
        # Parse the YAML file
        try:
            with open(yaml_path, 'r') as f:
                data = yaml.safe_load(f)
            print("✓ YAML file can be parsed successfully")
            
            # Verify required fields
            required_fields = [
                'tire_model', 'tire_name', 'effective_radius', 'nominal_load',
                'longitudinal', 'lateral', 'camber', 'fitting_info'
            ]
            
            for field in required_fields:
                if field in data:
                    print(f"✓ Required field present: {field}")
                else:
                    print(f"✗ Missing required field: {field}")
                    return False
            
            # Verify longitudinal section
            long_fields = ['mu_peak', 'mu_sensitivity', 'Kx', 'relaxation_length']
            for field in long_fields:
                if field in data['longitudinal']:
                    print(f"✓ Longitudinal field present: {field}")
                else:
                    print(f"✗ Missing longitudinal field: {field}")
                    return False
            
            # Verify lateral section
            lat_fields = ['mu_peak', 'mu_sensitivity', 'Ky', 'relaxation_length']
            for field in lat_fields:
                if field in data['lateral']:
                    print(f"✓ Lateral field present: {field}")
                else:
                    print(f"✗ Missing lateral field: {field}")
                    return False
            
            # Verify camber section
            if 'thrust_coefficient' in data['camber']:
                print("✓ Camber field present: thrust_coefficient")
            else:
                print("✗ Missing camber field: thrust_coefficient")
                return False
            
            # Verify fitting_info section
            fit_fields = ['data_source', 'fit_date', 'rmse_fx', 'rmse_fy', 'r2_fx', 'r2_fy']
            for field in fit_fields:
                if field in data['fitting_info']:
                    print(f"✓ Fitting info field present: {field}")
                else:
                    print(f"✗ Missing fitting info field: {field}")
                    return False
            
            print("\n✓ YAML export verification PASSED")
            return True
            
        except Exception as e:
            print(f"✗ Error parsing YAML file: {e}")
            return False
    else:
        print(f"✗ YAML file not found: {yaml_path}")
        return False


def verify_cpp_header_export():
    """Verify C++ header export functionality"""
    print("\n" + "=" * 60)
    print("C++ Header Export Verification")
    print("=" * 60)
    
    # Check existing C++ header file
    header_path = Path("output/tire_params/hoosier_18x6_r20.hpp")
    if header_path.exists():
        print(f"✓ C++ header file exists: {header_path}")
        
        # Read the header file
        try:
            with open(header_path, 'r') as f:
                content = f.read()
            print("✓ C++ header file can be read successfully")
            
            # Verify include guards
            if '#ifndef' in content and '#define' in content and '#endif' in content:
                print("✓ Include guards present")
            else:
                print("✗ Include guards missing")
                return False
            
            # Verify namespace
            if 'namespace fsae' in content and 'namespace tire' in content:
                print("✓ Namespaces present (fsae::tire)")
            else:
                print("✗ Namespaces missing")
                return False
            
            # Verify struct
            if 'struct' in content:
                print("✓ Struct definition present")
            else:
                print("✗ Struct definition missing")
                return False
            
            # Verify static constexpr members
            if 'static constexpr double' in content:
                print("✓ Static constexpr double members present")
            else:
                print("✗ Static constexpr double members missing")
                return False
            
            # Verify all parameters
            required_params = [
                'effective_radius', 'nominal_load',
                'mu_x_peak', 'k_mu_x', 'Kx',
                'mu_y_peak', 'k_mu_y', 'Ky',
                'C_gamma'
            ]
            
            for param in required_params:
                if param in content:
                    print(f"✓ Parameter present: {param}")
                else:
                    print(f"✗ Missing parameter: {param}")
                    return False
            
            # Verify comments with units
            if '[N]' in content and '[m]' in content and '[-]' in content:
                print("✓ Unit comments present")
            else:
                print("✗ Unit comments missing")
                return False
            
            # Verify file header comment
            if '@file' in content or 'TMeasy' in content:
                print("✓ File header comment present")
            else:
                print("✗ File header comment missing")
                return False
            
            # Test C++ syntax by compiling
            print("\nTesting C++ syntax by compiling...")
            # Use absolute path for include
            abs_header_path = header_path.absolute()
            test_cpp = f"""
#include "{abs_header_path}"
int main() {{
    double x = fsae::tire::Hoosier_18x6_R20::mu_x_peak;
    return 0;
}}
"""
            with tempfile.NamedTemporaryFile(mode='w', suffix='.cpp', delete=False) as f:
                f.write(test_cpp)
                test_file = f.name
            
            try:
                result = subprocess.run(
                    ['g++', '-std=c++17', '-c', test_file, '-o', '/tmp/test.o'],
                    capture_output=True,
                    text=True
                )
                
                if result.returncode == 0:
                    print("✓ C++ header has valid syntax (compilation successful)")
                else:
                    print(f"✗ C++ compilation failed: {result.stderr}")
                    return False
                    
            finally:
                Path(test_file).unlink()
                if Path('/tmp/test.o').exists():
                    Path('/tmp/test.o').unlink()
            
            print("\n✓ C++ header export verification PASSED")
            return True
            
        except Exception as e:
            print(f"✗ Error reading C++ header file: {e}")
            return False
    else:
        print(f"✗ C++ header file not found: {header_path}")
        return False


def main():
    """Run all export verification checks"""
    print("\n" + "=" * 60)
    print("CHECKPOINT 9: Export Functionality Verification")
    print("=" * 60 + "\n")
    
    yaml_ok = verify_yaml_export()
    cpp_ok = verify_cpp_header_export()
    
    print("\n" + "=" * 60)
    print("VERIFICATION SUMMARY")
    print("=" * 60)
    
    if yaml_ok and cpp_ok:
        print("✓ All export functionality checks PASSED")
        print("\nConclusion:")
        print("- YAML files are correctly generated and can be parsed")
        print("- C++ header files have valid syntax and correct structure")
        print("- All required fields are present in both formats")
        print("\n✓ Checkpoint 9 COMPLETE")
        return 0
    else:
        print("✗ Some export functionality checks FAILED")
        if not yaml_ok:
            print("  - YAML export verification failed")
        if not cpp_ok:
            print("  - C++ header export verification failed")
        return 1


if __name__ == '__main__':
    exit(main())
