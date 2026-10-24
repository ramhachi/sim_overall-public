function exportForCpp(model_path, output_dir, grid_config)
% EXPORTFORCPP Export trained model for C++ integration
%
% This function generates all necessary files for C++ integration:
% 1. Lookup table in binary format
% 2. Normalization parameters
% 3. C++ header and implementation files
% 4. Test data for validation
%
% Inputs:
%   model_path  - Path to trained model .mat file
%   output_dir  - Directory for output files
%   grid_config - Optional grid configuration structure
%
% Example:
%   model_path = 'output/ultra_deep_validation_model.mat';
%   output_dir = 'cpp_export';
%   exportForCpp(model_path, output_dir);

    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('Exporting Model for C++ Integration\n');
    fprintf('=================================================================\n\n');
    
    % Load model
    fprintf('Step 1: Loading model...\n');
    if ~isfile(model_path)
        error('Model file not found: %s', model_path);
    end
    model = load(model_path);
    fprintf('  ✓ Model loaded: %s\n\n', model_path);
    
    % Create output directory
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Default grid configuration
    if nargin < 3 || isempty(grid_config)
        fprintf('Step 2: Using default grid configuration...\n');
        grid_config = struct();
        grid_config.SR_range = [-0.2, 0.2, 40];
        grid_config.SA_range = [-6, 6, 50];
        grid_config.Fz_range = [200, 1300, 30];
        grid_config.IA_range = [0, 4, 15];
        grid_config.TSTC_range = [55, 80, 15];
    end
    
    fprintf('  Grid configuration:\n');
    fprintf('    SR:   %d points [%.2f, %.2f]\n', grid_config.SR_range(3), grid_config.SR_range(1), grid_config.SR_range(2));
    fprintf('    SA:   %d points [%.1f, %.1f]\n', grid_config.SA_range(3), grid_config.SA_range(1), grid_config.SA_range(2));
    fprintf('    Fz:   %d points [%.0f, %.0f]\n', grid_config.Fz_range(3), grid_config.Fz_range(1), grid_config.Fz_range(2));
    fprintf('    IA:   %d points [%.1f, %.1f]\n', grid_config.IA_range(3), grid_config.IA_range(1), grid_config.IA_range(2));
    fprintf('    TSTC: %d points [%.0f, %.0f]\n\n', grid_config.TSTC_range(3), grid_config.TSTC_range(1), grid_config.TSTC_range(2));
    
    % Generate lookup table
    fprintf('Step 3: Generating lookup table...\n');
    lut_path = fullfile(output_dir, 'tire_lookup_table.mat');
    generateLookupTable(model.trained_net, model.norm_params, lut_path, grid_config);
    fprintf('\n');
    
    % Export normalization parameters
    fprintf('Step 4: Exporting normalization parameters...\n');
    exportNormalizationParams(model.norm_params, fullfile(output_dir, 'normalization_params.txt'));
    fprintf('\n');
    
    % Generate C++ header
    fprintf('Step 5: Generating C++ header file...\n');
    generateCppHeader(model.norm_params, grid_config, fullfile(output_dir, 'dl_tire_model.hpp'));
    fprintf('\n');
    
    % Generate test data
    fprintf('Step 6: Generating test data for validation...\n');
    generateTestData(model.trained_net, model.norm_params, fullfile(output_dir, 'test_data.csv'));
    fprintf('\n');
    
    % Export lookup table to CSV (for debugging)
    fprintf('Step 7: Exporting lookup table to CSV (debug)...\n');
    exportLookupTableToCSV(lut_path, output_dir);
    fprintf('\n');
    
    fprintf('=================================================================\n');
    fprintf('✓ Export Complete!\n');
    fprintf('=================================================================\n');
    fprintf('Output directory: %s\n', output_dir);
    fprintf('Files generated:\n');
    fprintf('  - tire_lookup_table.mat (MATLAB format)\n');
    fprintf('  - normalization_params.txt (text format)\n');
    fprintf('  - dl_tire_model.hpp (C++ header)\n');
    fprintf('  - test_data.csv (validation data)\n');
    fprintf('  - SR_array.csv, SA_array.csv, ... (grid arrays)\n');
    fprintf('  - Fx_grid.csv, Fy_grid.csv, ... (prediction grids)\n');
    fprintf('\nNext steps:\n');
    fprintf('1. Copy files to your C++ project\n');
    fprintf('2. Include dl_tire_model.hpp in your code\n');
    fprintf('3. Run validation tests\n');
    fprintf('=================================================================\n\n');
end

function exportNormalizationParams(norm_params, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, '# Normalization Parameters for DL Tire Model\n');
    fprintf(fid, '# Generated: %s\n\n', datestr(now));
    
    fprintf(fid, '# Input normalization (Z-score)\n');
    fprintf(fid, '# Features: SR, SA, Fz, IA, TSTC\n');
    fprintf(fid, 'INPUT_MEAN: %.10f, %.10f, %.10f, %.10f, %.10f\n', norm_params.input_mean);
    fprintf(fid, 'INPUT_STD: %.10f, %.10f, %.10f, %.10f, %.10f\n\n', norm_params.input_std);
    
    fprintf(fid, '# Output normalization (Z-score)\n');
    fprintf(fid, '# Targets: Fx, Fy, Mx, Mz\n');
    fprintf(fid, 'OUTPUT_MEAN: %.10f, %.10f, %.10f, %.10f\n', norm_params.output_mean);
    fprintf(fid, 'OUTPUT_STD: %.10f, %.10f, %.10f, %.10f\n', norm_params.output_std);
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n', output_path);
end

function generateCppHeader(norm_params, grid_config, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, '#pragma once\n');
    fprintf(fid, '/// @file dl_tire_model.hpp\n');
    fprintf(fid, '/// @brief Deep Learning Tire Model - C++ Integration\n');
    fprintf(fid, '/// Generated: %s\n\n', datestr(now));
    
    fprintf(fid, '#include <array>\n');
    fprintf(fid, '#include <vector>\n');
    fprintf(fid, '#include <string>\n');
    fprintf(fid, '#include <cmath>\n');
    fprintf(fid, '#include <algorithm>\n\n');
    
    fprintf(fid, 'namespace fsae::physics::tire {\n\n');
    
    % Normalization parameters as constexpr
    fprintf(fid, '/// Normalization parameters (embedded at compile time)\n');
    fprintf(fid, 'template<typename Scalar = double>\n');
    fprintf(fid, 'struct DLTireNormParams {\n');
    fprintf(fid, '    static constexpr std::array<Scalar, 5> INPUT_MEAN = {\n');
    fprintf(fid, '        %.10f, %.10f, %.10f, %.10f, %.10f\n', norm_params.input_mean);
    fprintf(fid, '    };\n');
    fprintf(fid, '    static constexpr std::array<Scalar, 5> INPUT_STD = {\n');
    fprintf(fid, '        %.10f, %.10f, %.10f, %.10f, %.10f\n', norm_params.input_std);
    fprintf(fid, '    };\n');
    fprintf(fid, '    static constexpr std::array<Scalar, 4> OUTPUT_MEAN = {\n');
    fprintf(fid, '        %.10f, %.10f, %.10f, %.10f\n', norm_params.output_mean);
    fprintf(fid, '    };\n');
    fprintf(fid, '    static constexpr std::array<Scalar, 4> OUTPUT_STD = {\n');
    fprintf(fid, '        %.10f, %.10f, %.10f, %.10f\n', norm_params.output_std);
    fprintf(fid, '    };\n');
    fprintf(fid, '};\n\n');
    
    % Output structure
    fprintf(fid, '/// DL Tire Model Output\n');
    fprintf(fid, 'template<typename Scalar = double>\n');
    fprintf(fid, 'struct DLTireOutput {\n');
    fprintf(fid, '    Scalar Fx;  // Longitudinal force [N]\n');
    fprintf(fid, '    Scalar Fy;  // Lateral force [N]\n');
    fprintf(fid, '    Scalar Mx;  // Overturning moment [Nm]\n');
    fprintf(fid, '    Scalar Mz;  // Self-aligning moment [Nm]\n');
    fprintf(fid, '};\n\n');
    
    % Lookup table class declaration
    fprintf(fid, '/// Deep Learning Tire Model using 5D Lookup Table\n');
    fprintf(fid, 'template<typename Scalar = double>\n');
    fprintf(fid, 'class DLTireModel {\n');
    fprintf(fid, 'public:\n');
    fprintf(fid, '    /// Constructor: Load lookup table from CSV files\n');
    fprintf(fid, '    explicit DLTireModel(const std::string& data_dir);\n\n');
    
    fprintf(fid, '    /// Predict tire forces and moments\n');
    fprintf(fid, '    /// @param SR Slip ratio [-]\n');
    fprintf(fid, '    /// @param SA Slip angle [deg]\n');
    fprintf(fid, '    /// @param Fz Vertical load [N]\n');
    fprintf(fid, '    /// @param IA Inclination angle [deg]\n');
    fprintf(fid, '    /// @param TSTC Tire surface temperature [C]\n');
    fprintf(fid, '    DLTireOutput<Scalar> predict(Scalar SR, Scalar SA, Scalar Fz, \n');
    fprintf(fid, '                                  Scalar IA, Scalar TSTC) const;\n\n');
    
    fprintf(fid, 'private:\n');
    fprintf(fid, '    // Grid arrays\n');
    fprintf(fid, '    std::vector<Scalar> SR_array_;\n');
    fprintf(fid, '    std::vector<Scalar> SA_array_;\n');
    fprintf(fid, '    std::vector<Scalar> Fz_array_;\n');
    fprintf(fid, '    std::vector<Scalar> IA_array_;\n');
    fprintf(fid, '    std::vector<Scalar> TSTC_array_;\n\n');
    
    fprintf(fid, '    // Prediction grids (flattened 5D arrays)\n');
    fprintf(fid, '    std::vector<Scalar> Fx_grid_;\n');
    fprintf(fid, '    std::vector<Scalar> Fy_grid_;\n');
    fprintf(fid, '    std::vector<Scalar> Mx_grid_;\n');
    fprintf(fid, '    std::vector<Scalar> Mz_grid_;\n\n');
    
    fprintf(fid, '    // Grid dimensions\n');
    fprintf(fid, '    size_t n_SR_, n_SA_, n_Fz_, n_IA_, n_TSTC_;\n\n');
    
    fprintf(fid, '    // Helper functions\n');
    fprintf(fid, '    Scalar interp5d(const std::vector<Scalar>& grid,\n');
    fprintf(fid, '                    Scalar SR, Scalar SA, Scalar Fz, \n');
    fprintf(fid, '                    Scalar IA, Scalar TSTC) const;\n');
    fprintf(fid, '    size_t find_index(const std::vector<Scalar>& arr, Scalar val) const;\n');
    fprintf(fid, '};\n\n');
    
    fprintf(fid, '} // namespace fsae::physics::tire\n');
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n', output_path);
end

function generateTestData(trained_net, norm_params, output_path)
    % Generate test cases
    n_test = 20;
    rng(42);
    
    % Random test points within training range
    SR_test = unifrnd(-0.2, 0.2, n_test, 1);
    SA_test = unifrnd(-6, 6, n_test, 1);
    Fz_test = unifrnd(200, 1300, n_test, 1);
    IA_test = unifrnd(0, 4, n_test, 1);
    TSTC_test = unifrnd(55, 80, n_test, 1);
    
    X_test = [SR_test, SA_test, Fz_test, IA_test, TSTC_test];
    Y_test = predictTireForces(trained_net, X_test, norm_params);
    
    % Write to CSV
    fid = fopen(output_path, 'w');
    fprintf(fid, 'SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz\n');
    for i = 1:n_test
        fprintf(fid, '%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
                X_test(i,:), Y_test(i,:));
    end
    fclose(fid);
    
    fprintf('  ✓ Generated %d test cases: %s\n', n_test, output_path);
end

function exportLookupTableToCSV(lut_path, output_dir)
    % Load lookup table
    data = load(lut_path);
    lut = data.lookup_table;
    
    % Export grid arrays
    writematrix(lut.SR_array', fullfile(output_dir, 'SR_array.csv'));
    writematrix(lut.SA_array', fullfile(output_dir, 'SA_array.csv'));
    writematrix(lut.Fz_array', fullfile(output_dir, 'Fz_array.csv'));
    writematrix(lut.IA_array', fullfile(output_dir, 'IA_array.csv'));
    writematrix(lut.TSTC_array', fullfile(output_dir, 'TSTC_array.csv'));
    
    % Export prediction grids (flattened)
    writematrix(lut.Fx_grid(:), fullfile(output_dir, 'Fx_grid.csv'));
    writematrix(lut.Fy_grid(:), fullfile(output_dir, 'Fy_grid.csv'));
    writematrix(lut.Mx_grid(:), fullfile(output_dir, 'Mx_grid.csv'));
    writematrix(lut.Mz_grid(:), fullfile(output_dir, 'Mz_grid.csv'));
    
    fprintf('  ✓ Exported lookup table to CSV files\n');
end
