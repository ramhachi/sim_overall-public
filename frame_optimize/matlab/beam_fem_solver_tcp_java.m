function result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads, options)
% BEAM_FEM_SOLVER_TCP_JAVA High-performance TCP client using Java Socket API
%
% This function uses MATLAB's internal Java engine (java.net.Socket) directly
% for minimal latency (~0.1-0.2ms per call vs 1.9ms for tcpclient).
%
% Syntax:
%   result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads)
%   beam_fem_solver_tcp_java('start')    % Start server manually
%   beam_fem_solver_tcp_java('stop')     % Stop server manually
%
% Performance:
%   - First call: server startup cost (local executable or Docker)
%   - Subsequent calls: ~0.1-0.2 ms (Java Socket overhead) + FEM solve time
%   - 10-20x faster than MATLAB tcpclient

    persistent server_handle server_ready java_socket input_stream output_stream

    % Handle control commands
    if nargin == 1 && ischar(nodes)
        command = nodes;
        if strcmpi(command, 'start')
            server_handle = beam_fem_server_manager('start');
            server_ready = true;
            result = struct('success', 1, 'message', 'Server started');
            return;
        elseif strcmpi(command, 'stop') || strcmpi(command, 'shutdown')
            % Close socket
            try
                if ~isempty(java_socket)
                    java_socket.close();
                end
            catch
            end
            java_socket = [];
            input_stream = [];
            output_stream = [];

            % Stop server
            beam_fem_server_manager('stop', server_handle);
            server_handle = [];
            server_ready = false;
            result = struct('success', 1, 'message', 'Server stopped');
            return;
        else
            error('beam_fem_solver_tcp_java:UnknownCommand', 'Unknown command: %s', command);
        end
    end

    narginchk(4, 5);

    % Try to connect with up to 2 attempts (second attempt if first failed due to stale server state)
    for attempt = 1:2
        % Check if server is ready (first call or after restart)
        % Check if server is ready (first call or after restart)
        if isempty(server_ready) || (~islogical(server_ready) || ~server_ready)
            % Try to connect to existing server first
            if beam_fem_server_manager('check_alive')
                server_ready = true;
                server_handle = struct('mode', 'external', 'kind', 'external', 'display', 'existing external server');
            else
                % Are we in a parallel worker? (getCurrentTask returns non-empty if so)
                try
                    t = getCurrentTask();
                    is_worker = ~isempty(t);
                catch
                    is_worker = false;
                end

                if is_worker
                    % Workers should NOT try to start the server. 
                    % Just wait a bit and try to connect again in the next attempt.
                    pause(1.0 + rand()); 
                else
                    % Main process or non-parallel: start new server
                    fprintf('Starting FEM server (first call/retry)...\n');
                    server_handle = beam_fem_server_manager('start');
                    server_ready = true;
                    fprintf('Server ready (%s)\n', beam_fem_server_manager('describe', server_handle));
                end
            end
        end

        % Create new Java socket connection for each request
        % (Server closes connection after sending response)
        try
            java_socket = java.net.Socket('localhost', get_fem_server_host_port());
            java_socket.setTcpNoDelay(true);  % Disable Nagle's algorithm for low latency
            java_socket.setSoTimeout(10000);  % 10 second timeout

            % Get I/O streams
            input_stream = java.io.BufferedReader(java.io.InputStreamReader(java_socket.getInputStream()));
            output_stream = java.io.PrintWriter(java_socket.getOutputStream(), true);
            
            % If we reached here, connection is successful
            break;
        catch ME
            % Are we in a parallel worker?
            try
                t = getCurrentTask();
                is_worker = ~isempty(t);
            catch
                is_worker = false;
            end

            if attempt == 1
                if is_worker
                    % Worker: JUST WAIT. The main process might be restarting it.
                    fprintf('Worker connection failed: %s. Retrying after pause...\n', ME.message);
                    pause(2.0 + 2.0*rand()); 
                    server_ready = false; % Force re-check in next attempt
                    continue;
                else
                    % Main process: If first attempt failed, it might be a truly stale server state.
                    fprintf('Main connection failed: %s. Retrying with server restart...\n', ME.message);
                    server_ready = false;
                    server_handle = [];
                    continue;
                end
            else
                % Second attempt failed
                error('beam_fem_solver_tcp_java:ConnectionFailed', 'Failed to connect after retry: %s', ME.message);
            end
        end
    end

    % Convert MATLAB data to JSON structure
    request = struct();
    request.command = 'SOLVE';
    request.nodes = force2D(nodes);
    request.elements = force2D(elements);
    request.bcs = force2D(bcs);
    request.loads = force2D(loads);

    % Send request (JSON + newline)
    request_json = jsonencode(request);
    fprintf('Sending request to FEM server (%d bytes)...\n', length(request_json));
    try
        output_stream.println(request_json);  % Sends with \n automatically
        output_stream.flush();
    catch ME
        % Close socket on error
        try
            java_socket.close();
        catch
        end
        error('beam_fem_solver_tcp_java:SendFailed', 'Failed to send request: %s', ME.message);
    end

    % Read response (single line JSON)
    try
        response_json = char(input_stream.readLine());

        % Close socket after reading response
        try
            java_socket.close();
        catch
        end

        if isempty(response_json) || strcmp(response_json, 'null')
            fprintf('Error: Server returned empty response. Checking logs...\n');
            if ~isempty(server_handle)
                beam_fem_server_manager('print_logs', server_handle);
            end
            error('beam_fem_solver_tcp_java:EmptyResponse', 'Server returned empty response');
        end

        response = jsondecode(response_json);
    catch ME
        % Close connection on error
        try
            java_socket.close();
        catch
        end

        if contains(ME.identifier, 'EmptyResponse')
            rethrow(ME);
        end
        error('beam_fem_solver_tcp_java:ReadFailed', 'Failed to read response: %s', ME.message);
    end

    % Check success
    if ~response.success
        if isfield(response, 'error')
            error('beam_fem_solver_tcp_java:SolverError', 'Solver error: %s', response.error);
        else
            error('beam_fem_solver_tcp_java:SolverError', 'Solver failed with unknown error');
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
