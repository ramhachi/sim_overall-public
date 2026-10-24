function result = beam_fem_solver(nodes, elements, bcs, loads, options)
% BEAM_FEM_SOLVER Wrapper for standalone C++ FEM solver
%
% This function wraps the beam_fem_standalone executable, preferring a local
% native binary and falling back to Docker when needed.
%
% Syntax:
%   result = beam_fem_solver(nodes, elements, bcs, loads, options)
%
% Inputs:
%   nodes    - Nx3 double [x, y, z] in meters
%   elements - Mx13 double [node_i, node_j, D_outer, t, E, G, rho, shape, jf, kAy, kAz, off_i, off_j] (Mx11 / legacy Mx9 accepted)
%   bcs      - Bx3 double [node_id, dof, value]
%   loads    - Lx3 double [node_id, dof, value]
%   options  - struct (currently unused)
%
% Output:
%   result - struct with fields:
%            U         - Nx6 displacement [ux, uy, uz, rx, ry, rz]
%            K_torsion - Torsional rigidity [Nm/deg]
%            mass      - Total mass [kg]
%            success   - 1 if solve succeeded, 0 otherwise

    narginchk(4, 5);

    % Get paths
    script_dir = fileparts(mfilename('fullpath'));
    project_dir = fileparts(script_dir);
    bin_dir = fullfile(project_dir, 'bin');
    temp_dir = fullfile(project_dir, 'temp');

    % Create temp directory if it doesn't exist
    if ~isfolder(temp_dir)
        mkdir(temp_dir);
    end

    % Generate unique filenames
    timestamp = sprintf('%d', round(now() * 1e10));
    input_file = fullfile(temp_dir, sprintf('input_%s.json', timestamp));
    output_file = fullfile(temp_dir, sprintf('output_%s.json', timestamp));

    % Convert MATLAB data to JSON structure
    input_json = struct();
    input_json.nodes = force2D(nodes);
    input_json.elements = force2D(elements);
    input_json.bcs = force2D(bcs);
    input_json.loads = force2D(loads);

    % Write input JSON
    fid = fopen(input_file, 'w');
    if fid == -1
        error('beam_fem_solver:FileWriteError', 'Cannot write to %s', input_file);
    end
    fprintf(fid, '%s', jsonencode(input_json));
    fclose(fid);

    % Run local binary first, then fall back to Docker
    binary_path = find_local_standalone_binary(project_dir);
    if ~isempty(binary_path)
        solver_cmd = sprintf('%s %s %s', ...
            shell_quote(binary_path), shell_quote(input_file), shell_quote(output_file));
    else
        if ispc
            incompatible_binary = find_incompatible_standalone_binary(project_dir);
            if ~isempty(incompatible_binary)
                error('beam_fem_solver:MissingBinary', ...
                    ['Windows executable beam_fem_standalone.exe was not found. ', ...
                     'Found incompatible non-Windows binary instead: %s'], incompatible_binary);
            end
        end
        docker_cmd = build_docker_solver_command(project_dir, timestamp);
        solver_cmd = docker_cmd;
    end

    [status, cmdout] = system(solver_cmd);

    % Check if output file was created
    if ~isfile(output_file)
        error('beam_fem_solver:ExecutionFailed', ...
              'Solver failed to produce output.\nStatus: %d\nOutput: %s', status, cmdout);
    end

    % Read output JSON
    fid = fopen(output_file, 'r');
    if fid == -1
        error('beam_fem_solver:FileReadError', 'Cannot read from %s', output_file);
    end
    output_text = fread(fid, '*char')';
    fclose(fid);

    output_json = jsondecode(output_text);

    % Clean up temporary files (keep for debugging if error)
    if output_json.success
        delete(input_file);
        delete(output_file);
    else
        fprintf('Debug: Input saved to %s\n', input_file);
        fprintf('Debug: Output saved to %s\n', output_file);
    end

    % Check success
    if ~output_json.success
        if isfield(output_json, 'error')
            error('beam_fem_solver:SolverError', 'Solver error: %s', output_json.error);
        else
            error('beam_fem_solver:SolverError', 'Solver failed with unknown error');
        end
    end

    % Convert output to MATLAB format
    result = struct();
    result.success = 1;
    result.mass = output_json.mass;
    result.K_torsion = output_json.K_torsion;

    % Convert U from JSON format to matrix
    % Handle both cell array and struct array (MATLAB version compatibility)
    U_data = output_json.U;

    if iscell(U_data)
        % Cell array format
        n_nodes = length(U_data);
        U = zeros(n_nodes, 6);
        for i = 1:n_nodes
            U(i, :) = U_data{i};
        end
    elseif isstruct(U_data)
        % Struct array format - convert to matrix
        n_nodes = length(U_data);
        U = zeros(n_nodes, 6);
        for i = 1:n_nodes
            row_data = U_data(i);
            % Each row is a struct with numeric fields
            fields = fieldnames(row_data);
            for j = 1:min(6, length(fields))
                U(i, j) = row_data.(fields{j});
            end
        end
    else
        % Already a matrix (numeric array)
        U = U_data;
    end

    result.U = U;
    for extra = {'element_forces', 'element_stress_vm', 'stress_max', 'element_buckling_ratio', 'buckling_ratio', 'warnings'}
        if isfield(output_json, extra{1})
            result.(extra{1}) = output_json.(extra{1});
        end
    end
end

function out_cell = force2D(mat)
    % Helper to force 1D arrays (1 row) to be encoded as [[...]] in JSON
    if isempty(mat)
        out_cell = [];
    elseif size(mat, 1) == 1
        out_cell = {mat};
    else
        out_cell = mat;
    end
end

function binary_path = find_local_standalone_binary(project_dir)
    if ispc
        binary_names = {'beam_fem_standalone.exe'};
    else
        binary_names = {'beam_fem_standalone'};
    end

    search_dirs = { ...
        fullfile(project_dir, 'bin'), ...
        fullfile(project_dir, 'cpp', 'build'), ...
        fullfile(project_dir, 'cpp', 'build', 'Release'), ...
        fullfile(project_dir, 'cpp', 'build', 'RelWithDebInfo'), ...
        fullfile(project_dir, 'cpp', 'build', 'Debug') ...
    };

    binary_path = '';
    for d = 1:numel(search_dirs)
        for b = 1:numel(binary_names)
            candidate = fullfile(search_dirs{d}, binary_names{b});
            if isfile(candidate)
                binary_path = candidate;
                return;
            end
        end
    end
end

function binary_path = find_incompatible_standalone_binary(project_dir)
    if ~ispc
        binary_path = '';
        return;
    end

    search_dirs = { ...
        fullfile(project_dir, 'bin'), ...
        fullfile(project_dir, 'cpp', 'build'), ...
        fullfile(project_dir, 'cpp', 'build', 'Release'), ...
        fullfile(project_dir, 'cpp', 'build', 'RelWithDebInfo'), ...
        fullfile(project_dir, 'cpp', 'build', 'Debug') ...
    };

    binary_path = '';
    for d = 1:numel(search_dirs)
        candidate = fullfile(search_dirs{d}, 'beam_fem_standalone');
        if isfile(candidate)
            binary_path = candidate;
            return;
        end
    end
end

function docker_cmd = build_docker_solver_command(project_dir, timestamp)
    compose_file = fullfile(fileparts(project_dir), 'docker-compose.yml');
    if ~isfile(compose_file)
        error('beam_fem_solver:MissingBinary', ...
            'Local beam_fem_standalone binary was not found and docker-compose.yml is missing.');
    end

    [status, output] = system('docker info');
    if status ~= 0
        error('beam_fem_solver:MissingBinary', ...
            'Local beam_fem_standalone binary was not found and Docker is unavailable: %s', strtrim(output));
    end

    [status, ~] = system('docker compose version');
    if status == 0
        compose_base = 'docker compose';
    else
        [status, ~] = system('docker-compose version');
        if status ~= 0
            error('beam_fem_solver:MissingBinary', ...
                'Local beam_fem_standalone binary was not found and Docker Compose is unavailable.');
        end
        compose_base = 'docker-compose';
    end

    project_root = fileparts(project_dir);
    docker_cmd = sprintf([ ...
        '%s --project-directory %s -f %s run --rm fsae-sim ', ...
        'sh -lc "cd /workspace/frame_optimize/bin && ./beam_fem_standalone ', ...
        '/workspace/frame_optimize/temp/%s /workspace/frame_optimize/temp/%s"'], ...
        compose_base, shell_quote(project_root), shell_quote(compose_file), ...
        sprintf('input_%s.json', timestamp), sprintf('output_%s.json', timestamp));
end

function quoted = shell_quote(value)
    quoted = ['"' char(value) '"'];
end
