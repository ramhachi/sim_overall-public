function result = beam_fem_solver_tcp(nodes, elements, bcs, loads, options)
% BEAM_FEM_SOLVER_TCP TCP client for FEM server
%
% This function connects to a TCP FEM server running locally or in Docker,
% providing high-speed evaluation without repeated startup overhead.
%
% Syntax:
%   result = beam_fem_solver_tcp(nodes, elements, bcs, loads)
%   beam_fem_solver_tcp('start')    % Start server manually
%   beam_fem_solver_tcp('stop')     % Stop server manually
%   beam_fem_solver_tcp('shutdown') % Alias for 'stop'
%
% Inputs:
%   nodes    - Nx3 double [x, y, z] in meters
%   elements - Mx13 double [node_i, node_j, D_outer, t, E, G, rho, shape, jf, kAy, kAz, off_i, off_j] (Mx11 / legacy Mx9 accepted)
%   bcs      - Bx3 double [node_id, dof, value]
%   loads    - Lx3 double [node_id, dof, value]
%   options  - struct, optional .rigid_links Rx2 [node_a, node_b] (rigid bodies, RBE2)
%
% Output:
%   result - struct with fields:
%            U         - Nx6 displacement [ux, uy, uz, rx, ry, rz]
%            K_torsion - Torsional rigidity [Nm/deg]
%            mass      - Total mass [kg]
%            success   - 1 if solve succeeded, 0 otherwise
%
% Performance:
%   - First call: server startup cost (local executable or Docker)
%   - Subsequent calls: ~5-10 ms (TCP communication + FEM solve)
%   - 200x faster than standalone execution path

    persistent server_handle server_ready

    % Handle control commands
    if nargin == 1 && ischar(nodes)
        command = nodes;
        if strcmpi(command, 'start')
            server_handle = beam_fem_server_manager('start');
            server_ready = true;
            result = struct('success', 1, 'message', 'Server started');
            return;
        elseif strcmpi(command, 'stop') || strcmpi(command, 'shutdown')
            beam_fem_server_manager('stop', server_handle);
            server_handle = [];
            server_ready = false;
            result = struct('success', 1, 'message', 'Server stopped');
            return;
        else
            error('beam_fem_solver_tcp:UnknownCommand', 'Unknown command: %s', command);
        end
    end

    narginchk(4, 5);

    % Start server on first call
    if isempty(server_ready) || ~server_ready
        fprintf('Starting FEM server (first call)...\n');
        server_handle = beam_fem_server_manager('start');
        server_ready = true;
        fprintf('Server ready (%s)\n', beam_fem_server_manager('describe', server_handle));
    end

    % Create new connection for each request (server closes connection after each response)
    tcp_client = tcpclient('localhost', get_fem_server_host_port(), 'Timeout', 10);
    configureTerminator(tcp_client, "LF");

    % Convert MATLAB data to JSON structure
    request = struct();
    request.command = 'SOLVE';
    request.nodes = force2D(nodes);
    request.elements = force2D(elements);
    request.bcs = force2D(bcs);
    request.loads = force2D(loads);
    if nargin >= 5 && isstruct(options) && isfield(options, 'rigid_links') && ~isempty(options.rigid_links)
        request.rigid_links = force2D(options.rigid_links);
    end

    % Send request (with newline terminator)
    request_json = jsonencode(request);
    try
        % Send request with newline
        writeline(tcp_client, request_json);
        flush(tcp_client);  % Ensure data is sent
    catch ME
        % Connection failed
        delete(tcp_client);
        error('beam_fem_solver_tcp:SendFailed', 'Failed to send request: %s', ME.message);
    end

    % Read response (with timeout handling)
    try
        response_json = readline(tcp_client);

        % Check if response is empty
        if isempty(response_json) || strlength(response_json) == 0
            delete(tcp_client);
            error('beam_fem_solver_tcp:EmptyResponse', 'Server returned empty response');
        end

        response = jsondecode(response_json);
    catch ME
        % If timeout or parse error, show what we got
        if exist('response_json', 'var')
            fprintf('Error reading response: %s\n', ME.message);
            fprintf('Response string: ''%s''\n', response_json);
        end
        delete(tcp_client);
        rethrow(ME);
    end

    % Close connection (server will close its end after sending response)
    delete(tcp_client);

    % Check success
    if ~response.success
        if isfield(response, 'error')
            error('beam_fem_solver_tcp:SolverError', 'Solver error: %s', response.error);
        else
            error('beam_fem_solver_tcp:SolverError', 'Solver failed with unknown error');
        end
    end

    % Convert output to MATLAB format
    result = struct();
    result.success = 1;
    result.mass = response.mass;
    result.K_torsion = response.K_torsion;

    % Convert U from JSON format to matrix
    U_data = response.U;

    if iscell(U_data)
        % Cell array format
        n_nodes = length(U_data);
        U = zeros(n_nodes, 6);
        for i = 1:n_nodes
            U(i, :) = U_data{i};
        end
    elseif isstruct(U_data)
        % Struct array format
        n_nodes = length(U_data);
        U = zeros(n_nodes, 6);
        for i = 1:n_nodes
            row_data = U_data(i);
            fields = fieldnames(row_data);
            for j = 1:min(6, length(fields))
                U(i, j) = row_data.(fields{j});
            end
        end
    else
        % Already a matrix
        U = U_data;
    end

    result.U = U;
    for extra = {'element_forces', 'element_stress_vm', 'stress_max', 'element_buckling_ratio', 'buckling_ratio', 'warnings'}
        if isfield(response, extra{1})
            result.(extra{1}) = response.(extra{1});
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
