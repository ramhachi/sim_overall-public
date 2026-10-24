function exportModelForCpp(model_path, output_dir)
% EXPORTMODELFORCPP Export complete model for C++ integration
%
% This function exports:
% 1. Network weights and biases (text format)
% 2. Normalization parameters (text format)
% 3. Network architecture info (text format)
% 4. Test data for validation (CSV format)
%
% All files are in standard text formats that can be easily parsed in C++.
%
% Inputs:
%   model_path  - Path to trained model .mat file
%   output_dir  - Directory for output files
%
% Example:
%   exportModelForCpp('output/ultra_deep_validation_model.mat', 'cpp_export');

    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('Exporting Model for C++ Integration (Text Format)\n');
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
        fprintf('  ✓ Created directory: %s\n\n', output_dir);
    end
    
    % Export network weights
    fprintf('Step 2: Exporting network weights...\n');
    weights_path = fullfile(output_dir, 'network_weights.txt');
    exportNetworkWeights(model.trained_net, weights_path);
    
    % Export normalization parameters
    fprintf('Step 3: Exporting normalization parameters...\n');
    norm_path = fullfile(output_dir, 'normalization_params.txt');
    exportNormalizationParams(model.norm_params, norm_path);
    
    % Export network architecture
    fprintf('Step 4: Exporting network architecture...\n');
    arch_path = fullfile(output_dir, 'network_architecture.txt');
    exportNetworkArchitecture(model.trained_net, arch_path);
    
    % Generate test data
    fprintf('Step 5: Generating test data for validation...\n');
    test_path = fullfile(output_dir, 'test_data.csv');
    generateTestData(model.trained_net, model.norm_params, test_path);
    
    % Generate C++ implementation template
    fprintf('Step 6: Generating C++ implementation template...\n');
    cpp_path = fullfile(output_dir, 'dl_tire_model.cpp');
    generateCppImplementation(output_dir, cpp_path);
    
    % Generate README
    fprintf('Step 7: Generating README...\n');
    readme_path = fullfile(output_dir, 'README.txt');
    generateReadme(output_dir, readme_path);
    
    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('✓ Export Complete!\n');
    fprintf('=================================================================\n');
    fprintf('Output directory: %s\n\n', output_dir);
    fprintf('Files generated:\n');
    fprintf('  1. network_weights.txt       - All weights and biases\n');
    fprintf('  2. normalization_params.txt  - Input/output normalization\n');
    fprintf('  3. network_architecture.txt  - Layer structure\n');
    fprintf('  4. test_data.csv             - Validation test cases\n');
    fprintf('  5. dl_tire_model.cpp         - C++ implementation template\n');
    fprintf('  6. README.txt                - Usage instructions\n\n');
    fprintf('Next steps:\n');
    fprintf('  1. Review the generated files\n');
    fprintf('  2. Integrate dl_tire_model.cpp into your C++ project\n');
    fprintf('  3. Run validation tests using test_data.csv\n');
    fprintf('=================================================================\n\n');
end

function exportNormalizationParams(norm_params, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, '# Normalization Parameters for DL Tire Model\n');
    fprintf(fid, '# Generated: %s\n', datestr(now));
    fprintf(fid, '# Format: Z-score normalization (x_norm = (x - mean) / std)\n\n');
    
    fprintf(fid, '# Input Features: SR, SA, Fz, IA, TSTC\n');
    fprintf(fid, 'INPUT_SIZE: 5\n');
    fprintf(fid, 'INPUT_MEAN:\n');
    for i = 1:5
        fprintf(fid, '%.15e\n', norm_params.input_mean(i));
    end
    fprintf(fid, '\nINPUT_STD:\n');
    for i = 1:5
        fprintf(fid, '%.15e\n', norm_params.input_std(i));
    end
    
    fprintf(fid, '\n# Output Targets: Fx, Fy, Mx, Mz\n');
    fprintf(fid, 'OUTPUT_SIZE: 4\n');
    fprintf(fid, 'OUTPUT_MEAN:\n');
    for i = 1:4
        fprintf(fid, '%.15e\n', norm_params.output_mean(i));
    end
    fprintf(fid, '\nOUTPUT_STD:\n');
    for i = 1:4
        fprintf(fid, '%.15e\n', norm_params.output_std(i));
    end
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n\n', output_path);
end

function exportNetworkArchitecture(trained_net, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, '# Neural Network Architecture\n');
    fprintf(fid, '# Generated: %s\n\n', datestr(now));
    
    layers = trained_net.Layers;
    
    fprintf(fid, 'TOTAL_LAYERS: %d\n\n', length(layers));
    
    fc_idx = 0;
    for i = 1:length(layers)
        layer = layers(i);
        fprintf(fid, 'LAYER_%d:\n', i);
        fprintf(fid, '  Type: %s\n', class(layer));
        fprintf(fid, '  Name: %s\n', layer.Name);
        
        if isa(layer, 'nnet.cnn.layer.FullyConnectedLayer')
            fc_idx = fc_idx + 1;
            fprintf(fid, '  FC_Index: %d\n', fc_idx);
            fprintf(fid, '  InputSize: %d\n', layer.InputSize);
            fprintf(fid, '  OutputSize: %d\n', layer.OutputSize);
        elseif isa(layer, 'nnet.cnn.layer.ReLULayer')
            fprintf(fid, '  Activation: ReLU\n');
        elseif isa(layer, 'nnet.cnn.layer.DropoutLayer')
            fprintf(fid, '  Probability: %.2f\n', layer.Probability);
        elseif isa(layer, 'nnet.cnn.layer.BatchNormalizationLayer')
            fprintf(fid, '  NumChannels: %d\n', layer.NumChannels);
        end
        fprintf(fid, '\n');
    end
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n\n', output_path);
end

function generateTestData(trained_net, norm_params, output_path)
    % Generate test cases
    n_test = 50;
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
    fprintf(fid, '# Test Data for DL Tire Model Validation\n');
    fprintf(fid, '# Generated: %s\n', datestr(now));
    fprintf(fid, '# Format: SR,SA,Fz,IA,TSTC,Fx_expected,Fy_expected,Mx_expected,Mz_expected\n');
    fprintf(fid, 'SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz\n');
    for i = 1:n_test
        fprintf(fid, '%.10e,%.10e,%.10e,%.10e,%.10e,%.10e,%.10e,%.10e,%.10e\n', ...
                X_test(i,:), Y_test(i,:));
    end
    fclose(fid);
    
    fprintf('  ✓ Generated %d test cases: %s\n\n', n_test, output_path);
end

function generateCppImplementation(output_dir, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, '// dl_tire_model.cpp\n');
    fprintf(fid, '// Deep Learning Tire Model - C++ Implementation\n');
    fprintf(fid, '// Generated: %s\n\n', datestr(now));
    
    fprintf(fid, '#include <vector>\n');
    fprintf(fid, '#include <array>\n');
    fprintf(fid, '#include <fstream>\n');
    fprintf(fid, '#include <sstream>\n');
    fprintf(fid, '#include <string>\n');
    fprintf(fid, '#include <cmath>\n');
    fprintf(fid, '#include <algorithm>\n');
    fprintf(fid, '#include <stdexcept>\n\n');
    
    fprintf(fid, 'namespace fsae::physics::tire {\n\n');
    
    fprintf(fid, '// Neural network layer\n');
    fprintf(fid, 'struct DenseLayer {\n');
    fprintf(fid, '    std::vector<std::vector<double>> weights;  // [output][input]\n');
    fprintf(fid, '    std::vector<double> biases;                // [output]\n');
    fprintf(fid, '    \n');
    fprintf(fid, '    std::vector<double> forward(const std::vector<double>& input) const {\n');
    fprintf(fid, '        std::vector<double> output(biases.size());\n');
    fprintf(fid, '        for (size_t i = 0; i < biases.size(); ++i) {\n');
    fprintf(fid, '            output[i] = biases[i];\n');
    fprintf(fid, '            for (size_t j = 0; j < input.size(); ++j) {\n');
    fprintf(fid, '                output[i] += weights[i][j] * input[j];\n');
    fprintf(fid, '            }\n');
    fprintf(fid, '        }\n');
    fprintf(fid, '        return output;\n');
    fprintf(fid, '    }\n');
    fprintf(fid, '};\n\n');
    
    fprintf(fid, '// ReLU activation\n');
    fprintf(fid, 'inline void relu(std::vector<double>& x) {\n');
    fprintf(fid, '    for (auto& val : x) {\n');
    fprintf(fid, '        val = std::max(0.0, val);\n');
    fprintf(fid, '    }\n');
    fprintf(fid, '}\n\n');
    
    fprintf(fid, '// Deep Learning Tire Model\n');
    fprintf(fid, 'class DLTireModel {\n');
    fprintf(fid, 'public:\n');
    fprintf(fid, '    DLTireModel(const std::string& data_dir) {\n');
    fprintf(fid, '        load_normalization_params(data_dir + "/normalization_params.txt");\n');
    fprintf(fid, '        load_network_weights(data_dir + "/network_weights.txt");\n');
    fprintf(fid, '    }\n\n');
    
    fprintf(fid, '    struct Output {\n');
    fprintf(fid, '        double Fx, Fy, Mx, Mz;\n');
    fprintf(fid, '    };\n\n');
    
    fprintf(fid, '    Output predict(double SR, double SA, double Fz, double IA, double TSTC) const {\n');
    fprintf(fid, '        // Normalize inputs\n');
    fprintf(fid, '        std::vector<double> x = {SR, SA, Fz, IA, TSTC};\n');
    fprintf(fid, '        for (size_t i = 0; i < 5; ++i) {\n');
    fprintf(fid, '            x[i] = (x[i] - input_mean_[i]) / input_std_[i];\n');
    fprintf(fid, '        }\n\n');
    
    fprintf(fid, '        // Forward pass through network\n');
    fprintf(fid, '        for (size_t i = 0; i < layers_.size(); ++i) {\n');
    fprintf(fid, '            x = layers_[i].forward(x);\n');
    fprintf(fid, '            if (i < layers_.size() - 1) {  // No activation on output layer\n');
    fprintf(fid, '                relu(x);\n');
    fprintf(fid, '            }\n');
    fprintf(fid, '        }\n\n');
    
    fprintf(fid, '        // Denormalize outputs\n');
    fprintf(fid, '        for (size_t i = 0; i < 4; ++i) {\n');
    fprintf(fid, '            x[i] = x[i] * output_std_[i] + output_mean_[i];\n');
    fprintf(fid, '        }\n\n');
    
    fprintf(fid, '        return {x[0], x[1], x[2], x[3]};\n');
    fprintf(fid, '    }\n\n');
    
    fprintf(fid, 'private:\n');
    fprintf(fid, '    std::vector<DenseLayer> layers_;\n');
    fprintf(fid, '    std::array<double, 5> input_mean_, input_std_;\n');
    fprintf(fid, '    std::array<double, 4> output_mean_, output_std_;\n\n');
    
    fprintf(fid, '    void load_normalization_params(const std::string& path) {\n');
    fprintf(fid, '        // TODO: Implement file parsing\n');
    fprintf(fid, '        // Read INPUT_MEAN, INPUT_STD, OUTPUT_MEAN, OUTPUT_STD\n');
    fprintf(fid, '    }\n\n');
    
    fprintf(fid, '    void load_network_weights(const std::string& path) {\n');
    fprintf(fid, '        // TODO: Implement file parsing\n');
    fprintf(fid, '        // Read LAYER_X_WEIGHTS and LAYER_X_BIASES\n');
    fprintf(fid, '    }\n');
    fprintf(fid, '};\n\n');
    
    fprintf(fid, '} // namespace fsae::physics::tire\n');
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n\n', output_path);
end

function generateReadme(output_dir, output_path)
    fid = fopen(output_path, 'w');
    
    fprintf(fid, 'Deep Learning Tire Model - C++ Integration Guide\n');
    fprintf(fid, '================================================\n\n');
    fprintf(fid, 'Generated: %s\n\n', datestr(now));
    
    fprintf(fid, 'Files in this directory:\n');
    fprintf(fid, '------------------------\n');
    fprintf(fid, '1. network_weights.txt\n');
    fprintf(fid, '   - Contains all weights and biases for each layer\n');
    fprintf(fid, '   - Format: LAYER_X_WEIGHTS followed by LAYER_X_BIASES\n');
    fprintf(fid, '   - Weights are in row-major order (each row = output neuron)\n\n');
    
    fprintf(fid, '2. normalization_params.txt\n');
    fprintf(fid, '   - Input and output normalization parameters\n');
    fprintf(fid, '   - Z-score normalization: x_norm = (x - mean) / std\n');
    fprintf(fid, '   - Must apply before network forward pass\n\n');
    
    fprintf(fid, '3. network_architecture.txt\n');
    fprintf(fid, '   - Layer-by-layer description of network structure\n');
    fprintf(fid, '   - Includes layer types and dimensions\n\n');
    
    fprintf(fid, '4. test_data.csv\n');
    fprintf(fid, '   - 50 test cases for validation\n');
    fprintf(fid, '   - Format: SR,SA,Fz,IA,TSTC,Fx,Fy,Mx,Mz\n');
    fprintf(fid, '   - Use to verify C++ implementation matches MATLAB\n\n');
    
    fprintf(fid, '5. dl_tire_model.cpp\n');
    fprintf(fid, '   - C++ implementation template\n');
    fprintf(fid, '   - Includes basic structure and forward pass logic\n');
    fprintf(fid, '   - TODO: Complete file parsing functions\n\n');
    
    fprintf(fid, 'Usage:\n');
    fprintf(fid, '------\n');
    fprintf(fid, '1. Parse normalization_params.txt to get mean/std arrays\n');
    fprintf(fid, '2. Parse network_weights.txt to load all layer weights/biases\n');
    fprintf(fid, '3. Implement forward pass:\n');
    fprintf(fid, '   a. Normalize inputs: x_norm = (x - input_mean) / input_std\n');
    fprintf(fid, '   b. For each layer:\n');
    fprintf(fid, '      - Compute: y = W * x + b\n');
    fprintf(fid, '      - Apply ReLU: y = max(0, y)  (except last layer)\n');
    fprintf(fid, '   c. Denormalize outputs: y = y_norm * output_std + output_mean\n\n');
    
    fprintf(fid, '4. Validate using test_data.csv:\n');
    fprintf(fid, '   - Read each test case\n');
    fprintf(fid, '   - Run prediction\n');
    fprintf(fid, '   - Compare with expected values\n');
    fprintf(fid, '   - Acceptable error: < 0.1%% relative difference\n\n');
    
    fprintf(fid, 'Input Features:\n');
    fprintf(fid, '---------------\n');
    fprintf(fid, '1. SR   - Slip Ratio [-]\n');
    fprintf(fid, '2. SA   - Slip Angle [deg]\n');
    fprintf(fid, '3. Fz   - Vertical Load [N]\n');
    fprintf(fid, '4. IA   - Inclination Angle [deg]\n');
    fprintf(fid, '5. TSTC - Tire Surface Temperature [C]\n\n');
    
    fprintf(fid, 'Output Targets:\n');
    fprintf(fid, '---------------\n');
    fprintf(fid, '1. Fx - Longitudinal Force [N]\n');
    fprintf(fid, '2. Fy - Lateral Force [N]\n');
    fprintf(fid, '3. Mx - Overturning Moment [Nm]\n');
    fprintf(fid, '4. Mz - Self-Aligning Moment [Nm]\n\n');
    
    fprintf(fid, 'Notes:\n');
    fprintf(fid, '------\n');
    fprintf(fid, '- All text files use scientific notation (e.g., 1.234567890123456e-02)\n');
    fprintf(fid, '- Precision: 15 decimal places\n');
    fprintf(fid, '- File encoding: UTF-8\n');
    fprintf(fid, '- Line endings: Platform-dependent\n\n');
    
    fclose(fid);
    fprintf('  ✓ Saved: %s\n\n', output_path);
end
