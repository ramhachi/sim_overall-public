function generateLookupTable(trained_net, norm_params, output_path, grid_config)
% GENERATELOOKUPTABLE Generate multi-dimensional lookup table for C++ integration
%
% This function generates a lookup table by evaluating the neural network
% on a multi-dimensional grid of input values. The lookup table can be used
% for fast C++ integration without requiring MATLAB Coder.
%
% Inputs:
%   trained_net  - Trained neural network
%   norm_params  - Structure with normalization parameters
%   output_path  - Path where the lookup table should be saved
%   grid_config  - Optional structure with grid configuration:
%                  .SR_range   - [min, max, n_points] for slip ratio
%                  .SA_range   - [min, max, n_points] for slip angle
%                  .Fz_range   - [min, max, n_points] for vertical load
%                  .IA_range   - [min, max, n_points] for inclination angle
%                  .TSTC_range - [min, max, n_points] for temperature
%
% Outputs:
%   None (saves lookup table to disk)
%
% Example:
%   % Use default grid
%   generateLookupTable(trained_net, norm_params, 'lookup_table.mat');
%   
%   % Custom grid with higher resolution
%   config.SR_range = [-0.2, 0.2, 50];
%   config.SA_range = [-15, 15, 50];
%   config.Fz_range = [500, 2500, 30];
%   config.IA_range = [-5, 5, 20];
%   config.TSTC_range = [20, 80, 20];
%   generateLookupTable(trained_net, norm_params, 'lookup_table.mat', config);
%
% Requirements: 6.4

    fprintf('=== Generating Lookup Table ===\n');
    
    % Default grid configuration
    if nargin < 4 || isempty(grid_config)
        fprintf('Using default grid configuration...\n');
        grid_config = struct();
        grid_config.SR_range = [-0.2, 0.2, 20];      % 20 points
        grid_config.SA_range = [-15, 15, 30];        % 30 points
        grid_config.Fz_range = [500, 2500, 20];      % 20 points
        grid_config.IA_range = [-5, 5, 10];          % 10 points
        grid_config.TSTC_range = [20, 80, 10];       % 10 points
    end
    
    % Extract grid parameters
    SR_min = grid_config.SR_range(1);
    SR_max = grid_config.SR_range(2);
    SR_n = grid_config.SR_range(3);
    
    SA_min = grid_config.SA_range(1);
    SA_max = grid_config.SA_range(2);
    SA_n = grid_config.SA_range(3);
    
    Fz_min = grid_config.Fz_range(1);
    Fz_max = grid_config.Fz_range(2);
    Fz_n = grid_config.Fz_range(3);
    
    IA_min = grid_config.IA_range(1);
    IA_max = grid_config.IA_range(2);
    IA_n = grid_config.IA_range(3);
    
    TSTC_min = grid_config.TSTC_range(1);
    TSTC_max = grid_config.TSTC_range(2);
    TSTC_n = grid_config.TSTC_range(3);
    
    % Create 1D arrays for each dimension
    SR_array = linspace(SR_min, SR_max, SR_n);
    SA_array = linspace(SA_min, SA_max, SA_n);
    Fz_array = linspace(Fz_min, Fz_max, Fz_n);
    IA_array = linspace(IA_min, IA_max, IA_n);
    TSTC_array = linspace(TSTC_min, TSTC_max, TSTC_n);
    
    % Calculate total number of grid points
    total_points = SR_n * SA_n * Fz_n * IA_n * TSTC_n;
    fprintf('Grid dimensions: %d × %d × %d × %d × %d = %d points\n', ...
            SR_n, SA_n, Fz_n, IA_n, TSTC_n, total_points);
    
    % Warn if grid is very large
    if total_points > 1e6
        warning('Large grid size (%d points) may take significant time and memory', total_points);
    end
    
    % Generate grid using ndgrid
    fprintf('Generating grid...\n');
    [SR_grid, SA_grid, Fz_grid, IA_grid, TSTC_grid] = ndgrid(...
        SR_array, SA_array, Fz_array, IA_array, TSTC_array);
    
    % Flatten grid to N×5 matrix
    X_grid = [SR_grid(:), SA_grid(:), Fz_grid(:), IA_grid(:), TSTC_grid(:)];
    
    % Compute predictions on grid
    fprintf('Computing predictions on grid (this may take a while)...\n');
    tic;
    Y_grid = predictTireForces(trained_net, X_grid, norm_params);
    elapsed = toc;
    fprintf('Predictions computed in %.2f seconds\n', elapsed);
    
    % Reshape predictions to match grid dimensions
    % Y_grid is N×4, reshape each output separately
    Fx_grid = reshape(Y_grid(:,1), SR_n, SA_n, Fz_n, IA_n, TSTC_n);
    Fy_grid = reshape(Y_grid(:,2), SR_n, SA_n, Fz_n, IA_n, TSTC_n);
    Mx_grid = reshape(Y_grid(:,3), SR_n, SA_n, Fz_n, IA_n, TSTC_n);
    Mz_grid = reshape(Y_grid(:,4), SR_n, SA_n, Fz_n, IA_n, TSTC_n);
    
    % Create lookup table structure
    lookup_table = struct();
    lookup_table.SR_array = SR_array;
    lookup_table.SA_array = SA_array;
    lookup_table.Fz_array = Fz_array;
    lookup_table.IA_array = IA_array;
    lookup_table.TSTC_array = TSTC_array;
    lookup_table.Fx_grid = Fx_grid;
    lookup_table.Fy_grid = Fy_grid;
    lookup_table.Mx_grid = Mx_grid;
    lookup_table.Mz_grid = Mz_grid;
    lookup_table.grid_dimensions = [SR_n, SA_n, Fz_n, IA_n, TSTC_n];
    lookup_table.total_points = total_points;
    
    % Add metadata
    lookup_table.description = 'Neural network tire model lookup table';
    lookup_table.input_names = {'SR', 'SA', 'Fz', 'IA', 'TSTC'};
    lookup_table.output_names = {'Fx', 'Fy', 'Mx', 'Mz'};
    lookup_table.input_units = {'-', 'deg', 'N', 'deg', 'C'};
    lookup_table.output_units = {'N', 'N', 'Nm', 'Nm'};
    
    % Create output directory if needed
    [output_dir, ~, ~] = fileparts(output_path);
    if ~isempty(output_dir) && ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end
    
    % Save lookup table
    fprintf('Saving lookup table to: %s\n', output_path);
    save(output_path, 'lookup_table', '-v7.3');
    
    % Report file size
    file_info = dir(output_path);
    file_size_mb = file_info.bytes / (1024 * 1024);
    fprintf('Lookup table saved successfully (%.2f MB)\n', file_size_mb);
    
    % Print usage instructions
    fprintf('\n--- Usage Instructions ---\n');
    fprintf('To use the lookup table in MATLAB:\n');
    fprintf('  data = load(''%s'');\n', output_path);
    fprintf('  lut = data.lookup_table;\n');
    fprintf('  % Use interpn for interpolation:\n');
    fprintf('  Fx = interpn(lut.SR_array, lut.SA_array, lut.Fz_array, ...\n');
    fprintf('               lut.IA_array, lut.TSTC_array, lut.Fx_grid, ...\n');
    fprintf('               SR, SA, Fz, IA, TSTC, ''linear'');\n');
    fprintf('==================================\n\n');
end
