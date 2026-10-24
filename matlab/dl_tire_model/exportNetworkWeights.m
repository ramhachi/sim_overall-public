function exportNetworkWeights(trained_net, output_path)
% EXPORTNETWORKWEIGHTS Export neural network weights to text format
%
% This function exports all weights and biases from a trained neural network
% to a human-readable text format that can be easily loaded in C++.
%
% Inputs:
%   trained_net - Trained neural network
%   output_path - Path for output text file
%
% Output file format:
%   LAYER_1_WEIGHTS: rows x cols
%   <weight matrix data>
%   LAYER_1_BIASES: size
%   <bias vector data>
%   ...
%
% Example:
%   exportNetworkWeights(trained_net, 'network_weights.txt');

    fprintf('=== Exporting Network Weights ===\n');
    
    % Open output file
    fid = fopen(output_path, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', output_path);
    end
    
    % Write header
    fprintf(fid, '# Neural Network Weights and Biases\n');
    fprintf(fid, '# Generated: %s\n', datestr(now));
    fprintf(fid, '# Format: Each layer has WEIGHTS and BIASES sections\n');
    fprintf(fid, '# WEIGHTS: row-major order (each row is output neuron weights)\n');
    fprintf(fid, '# BIASES: one value per output neuron\n\n');
    
    % Get network layers
    layers = trained_net.Layers;
    
    % Count fully connected layers
    fc_count = 0;
    for i = 1:length(layers)
        if isa(layers(i), 'nnet.cnn.layer.FullyConnectedLayer')
            fc_count = fc_count + 1;
        end
    end
    
    fprintf('Network has %d fully connected layers\n', fc_count);
    fprintf(fid, '# Network Architecture: %d fully connected layers\n', fc_count);
    fprintf(fid, '# Input: 5 features (SR, SA, Fz, IA, TSTC)\n');
    fprintf(fid, '# Output: 4 targets (Fx, Fy, Mx, Mz)\n\n');
    
    % Extract and write weights for each layer
    layer_idx = 0;
    for i = 1:length(layers)
        layer = layers(i);
        
        if isa(layer, 'nnet.cnn.layer.FullyConnectedLayer')
            layer_idx = layer_idx + 1;
            
            % Get weights and biases
            W = layer.Weights;  % Output x Input matrix
            b = layer.Bias;     % Output vector
            
            [n_out, n_in] = size(W);
            
            fprintf('Layer %d: %d inputs -> %d outputs\n', layer_idx, n_in, n_out);
            
            % Write layer header
            fprintf(fid, '# ========================================\n');
            fprintf(fid, '# Layer %d: Fully Connected\n', layer_idx);
            fprintf(fid, '# Input size: %d\n', n_in);
            fprintf(fid, '# Output size: %d\n', n_out);
            fprintf(fid, '# ========================================\n\n');
            
            % Write weights
            fprintf(fid, 'LAYER_%d_WEIGHTS: %d %d\n', layer_idx, n_out, n_in);
            for row = 1:n_out
                for col = 1:n_in
                    fprintf(fid, '%.15e', W(row, col));
                    if col < n_in
                        fprintf(fid, ' ');
                    end
                end
                fprintf(fid, '\n');
            end
            fprintf(fid, '\n');
            
            % Write biases
            fprintf(fid, 'LAYER_%d_BIASES: %d\n', layer_idx, n_out);
            for j = 1:n_out
                fprintf(fid, '%.15e\n', b(j));
            end
            fprintf(fid, '\n');
        end
    end
    
    fclose(fid);
    
    % Report file size
    file_info = dir(output_path);
    fprintf('✓ Weights exported successfully\n');
    fprintf('  File: %s\n', output_path);
    fprintf('  Size: %.2f KB\n', file_info.bytes / 1024);
    fprintf('  Layers: %d\n', fc_count);
    fprintf('================================\n\n');
end
