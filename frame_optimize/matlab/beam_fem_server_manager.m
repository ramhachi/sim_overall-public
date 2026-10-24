function varargout = beam_fem_server_manager(action, varargin)
% BEAM_FEM_SERVER_MANAGER Cross-platform lifecycle management for beam_fem_server.
%
% Supported actions:
%   handle = beam_fem_server_manager('start')
%   beam_fem_server_manager('stop', handle)
%   alive = beam_fem_server_manager('check_alive')
%   beam_fem_server_manager('print_logs', handle)
%   desc = beam_fem_server_manager('describe', handle)

    narginchk(1, 2);

    switch lower(action)
        case 'start'
            varargout{1} = start_server_only();
        case 'stop'
            stop_server(get_optional_handle(varargin));
        case 'check_alive'
            varargout{1} = check_server_alive();
        case 'print_logs'
            print_server_logs(get_optional_handle(varargin));
        case 'describe'
            varargout{1} = describe_server_handle(get_optional_handle(varargin));
        otherwise
            error('beam_fem_server_manager:UnknownAction', 'Unknown action: %s', action);
    end
end

function handle = get_optional_handle(args)
    if isempty(args)
        handle = [];
    else
        handle = args{1};
    end
end

function handle = start_server_only()
    ctx = get_runtime_context();
    cleanup_existing_server(ctx);

    if check_server_alive()
        handle = make_server_handle('external', 'external', '', '', '', '');
        return;
    end

    attempt_errors = {};
    if ispc || ismac
        [handle, err_msg] = try_start_local(ctx);
        if ~isempty(handle)
            return;
        end
        if ~isempty(err_msg)
            attempt_errors{end + 1} = err_msg; %#ok<AGROW>
        end

        [handle, err_msg] = try_start_docker(ctx);
        if ~isempty(handle)
            return;
        end
        if ~isempty(err_msg)
            attempt_errors{end + 1} = err_msg; %#ok<AGROW>
        end
    else
        [handle, err_msg] = try_start_docker(ctx);
        if ~isempty(handle)
            return;
        end
        if ~isempty(err_msg)
            attempt_errors{end + 1} = err_msg; %#ok<AGROW>
        end

        [handle, err_msg] = try_start_local(ctx);
        if ~isempty(handle)
            return;
        end
        if ~isempty(err_msg)
            attempt_errors{end + 1} = err_msg; %#ok<AGROW>
        end
    end

    error('beam_fem_server_manager:StartFailed', ...
        'Unable to start FEM server.%s', format_attempt_errors(attempt_errors));
end

function [handle, err_msg] = try_start_local(ctx)
    handle = [];
    err_msg = '';

    binary_path = find_local_server_binary(ctx);
    if isempty(binary_path)
        if ispc
            incompatible_paths = find_present_candidates(ctx.incompatible_local_candidates);
            if ~isempty(incompatible_paths)
                err_msg = sprintf(['Local start skipped: Windows executable beam_fem_server.exe was not found. ', ...
                    'Found incompatible non-Windows binary instead: %s'], join_for_error(incompatible_paths));
            else
                err_msg = sprintf(['Local start skipped: Windows executable beam_fem_server.exe was not found. ', ...
                    'Checked: %s'], join_for_error(ctx.local_candidates));
            end
        elseif ismac
            % Check if non-native (Linux ELF) binaries exist
            present = find_present_candidates(ctx.local_candidates);
            if ~isempty(present)
                err_msg = sprintf(['Local start skipped: found Linux binary (not macOS native). ', ...
                    'Will use Docker instead. Binary: %s'], join_for_error(present));
            else
                err_msg = sprintf(['Local start skipped: beam_fem_server binary was not found. ', ...
                    'Checked: %s'], join_for_error(ctx.local_candidates));
            end
        else
            err_msg = sprintf(['Local start skipped: beam_fem_server binary was not found. ', ...
                'Checked: %s'], join_for_error(ctx.local_candidates));
        end
        return;
    end

    fprintf('Starting FEM server locally...\n');

    try
        handle = start_local_server(binary_path, ctx);
        wait_for_server_ready(handle, ctx);
    catch ME
        handle = [];
        err_msg = sprintf('Local start failed (%s): %s', binary_path, ME.message);
    end
end

function [handle, err_msg] = try_start_docker(ctx)
    handle = [];
    err_msg = '';

    docker = detect_docker(ctx);
    if ~docker.available && (ispc || ismac)
        docker = try_autostart_docker_desktop(ctx, docker.reason);
    end
    if ~docker.available
        err_msg = sprintf('Docker start skipped: %s', docker.reason);
        return;
    end

    fprintf('Starting FEM server via Docker...\n');

    try
        handle = start_docker_server(ctx, docker);
        wait_for_server_ready(handle, ctx);
    catch ME
        handle = [];
        err_msg = sprintf('Docker start failed: %s', ME.message);
    end
end

function wait_for_server_ready(handle, ctx)
    fprintf('Waiting for server to start...');
    if strcmp(handle.mode, 'docker')
        max_attempts = 60;  % 30 seconds for Docker (cold start can be slow)
    else
        max_attempts = 40;  % 20 seconds for local
    end

    for attempt = 1:max_attempts
        if check_server_alive()
            fprintf(' ready!\n');
            return;
        end

        if strcmp(handle.mode, 'local') && ~process_is_running(handle, ctx)
            fprintf('\n');
            print_server_logs(handle);
            error('beam_fem_server_manager:LocalExited', ...
                'Local FEM server exited before becoming ready (%s).', ...
                describe_server_handle(handle));
        end

        pause(0.5);
        if mod(attempt, 2) == 0
            fprintf('.');
        end
    end

    fprintf('\n');
    print_server_logs(handle);

    % Print diagnostics for Docker mode
    if strcmp(handle.mode, 'docker') && isfield(handle, 'compose_cmd') && ~isempty(handle.compose_cmd)
        docker_struct = struct('compose_cmd', handle.compose_cmd);
        fprintf('--- Docker diagnostics ---\n');
        [~, ps_out] = system(compose_command(docker_struct, ...
            'exec -T fsae-sim sh -c "pgrep -a beam_fem_server 2>&1 || echo NO_PROCESS"'));
        fprintf('  Process: %s\n', strtrim(ps_out));
        [~, ss_out] = system(compose_command(docker_struct, ...
            'exec -T fsae-sim sh -c "ss -tlnp 2>/dev/null | grep 50051 || netstat -tlnp 2>/dev/null | grep 50051 || echo NO_LISTENER"'));
        fprintf('  Port 50051: %s\n', strtrim(ss_out));
        [~, arch_out] = system(compose_command(docker_struct, ...
            'exec -T fsae-sim sh -c "od -A n -t x1 -j 18 -N 2 /workspace/frame_optimize/bin/beam_fem_server 2>/dev/null || echo MISSING"'));
        fprintf('  Binary e_machine: %s (3e 00=x86-64, b7 00=aarch64)\n', strtrim(arch_out));
        fprintf('--------------------------\n');
    end

    error('beam_fem_server_manager:StartTimeout', ...
        'Server failed to start within timeout (%s).', describe_server_handle(handle));
end

function cleanup_existing_server(ctx)
    fprintf('Cleaning up stale FEM containers/processes...\n');
    send_shutdown_request(ctx.port);
    pause(0.2);

    pids = find_listening_pids(ctx.port);
    kill_processes(pids);

    cleanup_docker_processes(ctx);
    pause(0.3);
end

function send_shutdown_request(port)
    try
        shutdown_socket = java.net.Socket('localhost', port);
        out = java.io.PrintWriter(shutdown_socket.getOutputStream(), true);
        out.println(jsonencode(struct('command', 'SHUTDOWN')));
        out.flush();
        pause(0.2);
        shutdown_socket.close();
    catch
    end
end

function pids = find_listening_pids(port)
    pids = {};

    if ispc
        command = sprintf(['powershell -NoProfile -Command "', ...
            '$connections = Get-NetTCPConnection -LocalPort %d -State Listen ', ...
            '-ErrorAction SilentlyContinue | Select-Object -ExpandProperty OwningProcess; ', ...
            'if ($connections) { $connections | ForEach-Object { $_ } }"'], port);
        [status, output] = system(command);
        if status ~= 0 || isempty(strtrim(output))
            command = sprintf('netstat -ano -p tcp | findstr ":%d"', port);
            [~, output] = system(command);
            lines = split_nonempty_lines(output);
            for i = 1:numel(lines)
                tokens = regexp(strtrim(lines{i}), '\s+', 'split');
                if ~isempty(tokens)
                    pid = strtrim(tokens{end});
                    if ~isempty(regexp(pid, '^\d+$', 'once'))
                        pids{end + 1} = pid; %#ok<AGROW>
                    end
                end
            end
            pids = unique(pids);
            return;
        end
    else
        command = sprintf('lsof -nP -iTCP:%d -sTCP:LISTEN -t', port);
        [~, output] = system(command);
        if isempty(strtrim(output))
            command = sprintf('ss -ltnp "( sport = :%d )" 2>/dev/null', port);
            [~, output] = system(command);
            pid_matches = regexp(output, 'pid=(\d+)', 'tokens');
            for i = 1:numel(pid_matches)
                pids{end + 1} = pid_matches{i}{1}; %#ok<AGROW>
            end
            pids = unique(pids);
            return;
        end
    end

    if isempty(strtrim(output))
        return;
    end

    raw_pids = regexp(output, '\d+', 'match');
    if isempty(raw_pids)
        return;
    end

    pids = unique(raw_pids);
end

function kill_processes(pids)
    if isempty(pids)
        return;
    end

    for i = 1:numel(pids)
        pid = strtrim(pids{i});
        if isempty(pid)
            continue;
        end

        if ispc
            system(sprintf('taskkill /PID %s /F', pid));
        else
            system(sprintf('kill -9 %s', pid));
        end
    end
end

function cleanup_docker_processes(ctx)
    docker = detect_docker(ctx);
    if ~docker.available
        return;
    end

    [status, output] = system(compose_command(docker, 'ps -q fsae-sim'));
    if status == 0 && ~isempty(strtrim(output))
        system(compose_command(docker, ...
            'exec -T fsae-sim sh -lc "pkill -f beam_fem_server || true"'));
    end

    [status, output] = system(sprintf( ...
        'docker ps -q --filter %s --filter %s', ...
        shell_quote(sprintf('label=com.docker.compose.project=%s', ctx.compose_project)), ...
        shell_quote(sprintf('publish=%d', ctx.port))));
    if status ~= 0 || isempty(strtrim(output))
        return;
    end

    container_ids = split_nonempty_lines(output);
    for i = 1:numel(container_ids)
        system(sprintf('docker stop %s', strtrim(container_ids{i})));
    end
end

function handle = start_local_server(binary_path, ctx)
    if ~exist(ctx.log_dir, 'dir')
        mkdir(ctx.log_dir);
    end

    timestamp = datestr(now, 'yyyymmdd_HHMMSSFFF');
    stdout_log = fullfile(ctx.log_dir, sprintf('beam_fem_server_%s.out.log', timestamp));
    stderr_log = fullfile(ctx.log_dir, sprintf('beam_fem_server_%s.err.log', timestamp));
    working_dir = fileparts(binary_path);

    if ispc
        command = sprintf(['powershell -NoProfile -Command ', ...
            '"$ErrorActionPreference = ''Stop''; ', ...
            'try { $p = Start-Process -FilePath %s -ArgumentList ''%d'' ', ...
            '-WorkingDirectory %s -WindowStyle Hidden ', ...
            '-RedirectStandardOutput %s -RedirectStandardError %s -PassThru -ErrorAction Stop; ', ...
            'Write-Output $p.Id } catch { Write-Output $_.Exception.Message; exit 1 }"'], ...
            ps_quote(binary_path), ctx.port, ps_quote(working_dir), ...
            ps_quote(stdout_log), ps_quote(stderr_log));
    else
        command = sprintf('cd %s && nohup %s %d > %s 2> %s < /dev/null & echo $!', ...
            shell_quote(working_dir), shell_quote(binary_path), ctx.port, ...
            shell_quote(stdout_log), shell_quote(stderr_log));
    end

    [status, output] = system(command);
    if status ~= 0
        error('beam_fem_server_manager:LocalStartFailed', ...
            'Failed to launch local FEM server: %s', strtrim(output));
    end

    pid = strtrim(output);
    if isempty(pid) || isempty(regexp(pid, '^\d+$', 'once'))
        error('beam_fem_server_manager:LocalStartFailed', ...
            'Local FEM server did not report a valid process ID: %s', pid);
    end

    handle = make_server_handle('local', 'local-process', pid, ...
        sprintf('local PID %s', pid), stdout_log, stderr_log);
    handle.binary_path = binary_path;
end

function handle = start_docker_server(ctx, docker)
    % Ensure compose service container is running (handles Exited state)
    [status, output] = system(compose_command(docker, 'ps -q fsae-sim'));
    if status ~= 0 || isempty(strtrim(output))
        fprintf('  Starting fsae-sim container via docker compose up...\n');
        [status, cmdout] = system(compose_command(docker, 'up -d fsae-sim'));
        if status ~= 0
            error('beam_fem_server_manager:DockerStartFailed', ...
                'Failed to start fsae-sim container: %s', strtrim(cmdout));
        end
        % Wait for container to be fully ready (longer after Docker Desktop cold start)
        fprintf('  Waiting for container to become ready...\n');
        container_ready = false;
        for wait_i = 1:10
            pause(2);
            [status, output] = system(compose_command(docker, 'ps -q fsae-sim'));
            if status == 0 && ~isempty(strtrim(output))
                container_ready = true;
                break;
            end
        end
        if ~container_ready
            error('beam_fem_server_manager:DockerStartFailed', ...
                'fsae-sim container did not start within 20 seconds. Check docker compose logs.');
        end
    end

    container_id = strtrim(output);
    if ~isempty(container_id)
        lines = split_nonempty_lines(container_id);
        container_id = strtrim(lines{end});
    end

    ensure_fem_binary_in_container(docker);

    % Start server directly (no shell wrapper — more reliable across platforms)
    command = compose_command(docker, ...
        'exec -d fsae-sim /workspace/frame_optimize/bin/beam_fem_server 50051');
    [status, cmdout] = system(command);
    if status ~= 0
        error('beam_fem_server_manager:DockerStartFailed', ...
            'Failed to start server in container: %s', strtrim(cmdout));
    end

    % Verify the server process actually started in the container
    pause(2);
    [~, ps_output] = system(compose_command(docker, ...
        'exec -T fsae-sim sh -c "pgrep -f beam_fem_server || true"'));
    if isempty(strtrim(ps_output))
        % Process not found — capture error by running non-detached
        [~, err_output] = system(compose_command(docker, ...
            'exec -T fsae-sim sh -c "timeout 3 /workspace/frame_optimize/bin/beam_fem_server 50051 2>&1; echo EXIT_CODE=$?"'));
        error('beam_fem_server_manager:DockerStartFailed', ...
            'Server process not found in container after exec.\nDiagnostic output: %s', strtrim(err_output));
    end
    fprintf('  Server process confirmed running (PID: %s)\n', strtrim(ps_output));

    handle = make_server_handle('docker', 'compose-exec', container_id, ...
        'docker compose exec fsae-sim', '', '');
    handle.compose_cmd = docker.compose_cmd;
end

function ensure_fem_binary_in_container(docker)
    % Verify the FEM binary matches the container's architecture.
    % Uses ELF header byte inspection (no 'file' command needed — it may
    % not be installed in minimal containers).
    binary_path = '/workspace/frame_optimize/bin/beam_fem_server';

    % 1. Get container architecture
    [~, arch_raw] = system(compose_command(docker, 'exec -T fsae-sim uname -m'));
    container_arch = strtrim(arch_raw);
    fprintf('  Container architecture: %s\n', container_arch);

    % 2. Read ELF e_machine field (bytes 18-19) using od (always available)
    %    0x3E (3e 00) = x86-64,  0xB7 (b7 00) = AArch64
    [status, elf_raw] = system(compose_command(docker, ...
        sprintf('exec -T fsae-sim sh -c "od -A n -t x1 -j 18 -N 2 %s 2>/dev/null"', binary_path)));
    elf_machine = strtrim(elf_raw);

    % 3. Determine if rebuild is needed
    needs_rebuild = false;
    if status ~= 0 || isempty(elf_machine)
        fprintf('  FEM binary not found or unreadable in container.\n');
        needs_rebuild = true;
    else
        is_x86_binary = ~isempty(regexp(elf_machine, '3e\s+00', 'once'));
        is_arm_binary = ~isempty(regexp(elf_machine, 'b7\s+00', 'once'));
        is_arm_container = strcmp(container_arch, 'aarch64') || strcmp(container_arch, 'arm64');
        is_x86_container = strcmp(container_arch, 'x86_64');

        if is_arm_container && is_x86_binary
            fprintf('  Architecture mismatch: binary is x86-64, container is %s.\n', container_arch);
            needs_rebuild = true;
        elseif is_x86_container && is_arm_binary
            fprintf('  Architecture mismatch: binary is AArch64, container is %s.\n', container_arch);
            needs_rebuild = true;
        elseif ~is_x86_binary && ~is_arm_binary
            fprintf('  Unknown binary architecture (e_machine: %s). Forcing rebuild.\n', elf_machine);
            needs_rebuild = true;
        else
            fprintf('  FEM binary architecture matches container (%s).\n', container_arch);
        end
    end

    if ~needs_rebuild
        return;
    end

    % 4. Rebuild from source inside the container
    fprintf('  Rebuilding FEM binary for %s (this may take a minute)...\n', container_arch);
    build_cmd = compose_command(docker, ...
        'exec -T fsae-sim sh -c "cd /workspace/frame_optimize/cpp && mkdir -p build && cd build && cmake .. -DCMAKE_BUILD_TYPE=Release 2>&1 && cmake --build . -j$(nproc) 2>&1 && cmake --install . --config Release 2>&1"');
    [status, build_output] = system(build_cmd);
    if status ~= 0
        error('beam_fem_server_manager:BuildFailed', ...
            'Failed to build FEM binary in container:\n%s', build_output);
    end

    % 5. Verify rebuilt binary architecture
    [~, verify_raw] = system(compose_command(docker, ...
        sprintf('exec -T fsae-sim sh -c "od -A n -t x1 -j 18 -N 2 %s 2>/dev/null"', binary_path)));
    fprintf('  Rebuilt binary e_machine: %s (expected: b7 00 for aarch64, 3e 00 for x86-64)\n', strtrim(verify_raw));
end

function stop_server(handle)
    ctx = get_runtime_context();
    handle = normalize_server_handle(handle);

    send_shutdown_request(ctx.port);
    pause(0.2);

    if isempty(handle)
        cleanup_existing_server(ctx);
        return;
    end

    if strcmp(handle.mode, 'local')
        kill_processes({handle.id});
        return;
    end

    if strcmp(handle.mode, 'docker')
        docker = detect_docker(ctx);
        if ~docker.available
            return;
        end

        if strcmp(handle.kind, 'compose-exec')
            system(compose_command(docker, ...
                'exec -T fsae-sim sh -lc "pkill -f beam_fem_server || true"'));
        elseif ~isempty(handle.id)
            system(sprintf('docker stop %s', handle.id));
        end
    end
end

function alive = check_server_alive()
    alive = false;
    try
        s = java.net.Socket('localhost', 50051);
        s.setSoTimeout(1000);
        out = java.io.PrintWriter(s.getOutputStream(), true);
        in = java.io.BufferedReader(java.io.InputStreamReader(s.getInputStream()));

        ping_req = struct('command', 'PING');
        out.println(jsonencode(ping_req));
        out.flush();

        resp = char(in.readLine());
        s.close();

        if ~isempty(resp) && ~strcmp(resp, 'null')
            alive = true;
        end
    catch
    end
end

function print_server_logs(handle)
    handle = normalize_server_handle(handle);
    if isempty(handle)
        return;
    end

    if strcmp(handle.mode, 'local')
        print_log_tail(handle.log_file, 'Local FEM stdout');
        print_log_tail(handle.error_log, 'Local FEM stderr');
        return;
    end

    if strcmp(handle.mode, 'docker')
        ctx = get_runtime_context();
        docker = detect_docker(ctx);
        if ~docker.available
            return;
        end

        if strcmp(handle.kind, 'temporary-container') && ~isempty(handle.id)
            [~, output] = system(sprintf('docker logs %s', handle.id));
            if ~isempty(strtrim(output))
                fprintf('Docker FEM logs:\n%s\n', strtrim(output));
            end
        elseif strcmp(handle.kind, 'compose-exec')
            [~, output] = system(compose_command(docker, 'logs --tail 20 fsae-sim'));
            if ~isempty(strtrim(output))
                fprintf('Docker compose logs:\n%s\n', strtrim(output));
            end
        end
    end
end

function print_log_tail(path_value, label)
    if isempty(path_value) || ~isfile(path_value)
        return;
    end

    try
        contents = fileread(path_value);
        lines = split_nonempty_lines(contents);
        if isempty(lines)
            return;
        end

        first_idx = max(1, numel(lines) - 19);
        fprintf('%s (%s):\n', label, path_value);
        for i = first_idx:numel(lines)
            fprintf('  %s\n', lines{i});
        end
    catch ME
        fprintf('Unable to read %s: %s\n', path_value, ME.message);
    end
end

function running = process_is_running(handle, ctx)
    running = true;

    if strcmp(handle.mode, 'local')
        if isempty(handle.id)
            running = false;
            return;
        end

        if ispc
            [status, output] = system(sprintf('tasklist /FI "PID eq %s" /NH', handle.id));
            running = status == 0 && contains(output, handle.id);
        else
            [status, ~] = system(sprintf('ps -p %s > /dev/null 2>&1', handle.id));
            running = status == 0;
        end
        return;
    end

    if strcmp(handle.mode, 'docker') && strcmp(handle.kind, 'temporary-container') && ~isempty(handle.id)
        docker = detect_docker(ctx);
        if ~docker.available
            running = false;
            return;
        end

        [status, output] = system(sprintf('docker ps -q --filter id=%s', handle.id));
        running = status == 0 && ~isempty(strtrim(output));
    end
end

function ctx = get_runtime_context()
    matlab_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(matlab_dir);
    project_root = fileparts(frame_root);

    local_candidates = {};
    incompatible_local_candidates = {};
    binary_names = {'beam_fem_server'};
    incompatible_binary_names = {};
    if ispc
        binary_names = {'beam_fem_server.exe'};
        incompatible_binary_names = {'beam_fem_server'};
    end

    search_dirs = { ...
        fullfile(frame_root, 'bin'), ...
        fullfile(frame_root, 'cpp', 'build'), ...
        fullfile(frame_root, 'cpp', 'build', 'Release'), ...
        fullfile(frame_root, 'cpp', 'build', 'RelWithDebInfo'), ...
        fullfile(frame_root, 'cpp', 'build', 'Debug') ...
    };

    for d = 1:numel(search_dirs)
        for b = 1:numel(binary_names)
            local_candidates{end + 1} = fullfile(search_dirs{d}, binary_names{b}); %#ok<AGROW>
        end
        for b = 1:numel(incompatible_binary_names)
            incompatible_local_candidates{end + 1} = fullfile(search_dirs{d}, incompatible_binary_names{b}); %#ok<AGROW>
        end
    end

    ctx = struct();
    ctx.project_root = project_root;
    ctx.frame_root = frame_root;
    ctx.matlab_dir = matlab_dir;
    ctx.compose_file = fullfile(project_root, 'docker-compose.yml');
    [~, ctx.compose_project] = fileparts(project_root);
    ctx.log_dir = fullfile(frame_root, 'temp');
    ctx.port = get_fem_server_host_port();
    ctx.local_candidates = unique(local_candidates, 'stable');
    ctx.incompatible_local_candidates = unique(incompatible_local_candidates, 'stable');
end

function binary_path = find_local_server_binary(ctx)
    binary_path = '';
    for i = 1:numel(ctx.local_candidates)
        candidate = ctx.local_candidates{i};
        if isfile(candidate) && is_native_binary(candidate)
            binary_path = candidate;
            return;
        end
    end
end

function native = is_native_binary(binary_path)
    % Verify the binary matches the current platform's executable format.
    % Prevents trying to run Linux ELF on macOS or vice versa.
    if ispc
        [~, ~, ext] = fileparts(binary_path);
        native = strcmpi(ext, '.exe');
        return;
    end
    [status, output] = system(sprintf('file %s 2>/dev/null', shell_quote(binary_path)));
    if status ~= 0
        native = false;
        return;
    end
    if ismac
        native = contains(output, 'Mach-O');
    else
        native = contains(output, 'ELF');
    end
end

function paths = find_present_candidates(candidates)
    paths = {};
    for i = 1:numel(candidates)
        if isfile(candidates{i})
            paths{end + 1} = candidates{i}; %#ok<AGROW>
        end
    end
end

function docker = try_autostart_docker_desktop(ctx, reason)
    % Auto-start Docker Desktop on Windows/Mac if daemon is not running,
    % and wait up to 90 seconds for the engine to become ready.
    if ~contains(reason, 'daemon is not running')
        docker = detect_docker(ctx);
        return;
    end

    if ispc
        desktop_exe = 'C:\Program Files\Docker\Docker\Docker Desktop.exe';
        if ~isfile(desktop_exe)
            docker = detect_docker(ctx);
            return;
        end
        launch_cmd = sprintf('start "" "%s"', desktop_exe);
    elseif ismac
        if ~isfolder('/Applications/Docker.app')
            docker = detect_docker(ctx);
            return;
        end
        launch_cmd = 'open -a Docker';
    else
        docker = detect_docker(ctx);
        return;
    end

    fprintf('  Docker Desktop daemon is not running. Launching Docker Desktop...\n');
    system(launch_cmd);

    fprintf('  Waiting for Docker engine to initialize (up to 90s)');
    for attempt = 1:18
        pause(5);
        fprintf('.');
        docker = detect_docker(ctx);
        if docker.available
            fprintf(' ready!\n');
            return;
        end
    end
    fprintf('\n');
    docker = detect_docker(ctx);
end

function docker = detect_docker(ctx)
    docker = struct();
    docker.available = false;
    docker.compose_cmd = '';
    docker.reason = '';

    if ~isfile(ctx.compose_file)
        docker.reason = sprintf('docker-compose.yml was not found at %s', ctx.compose_file);
        return;
    end

    [status, output] = system('docker info 2>&1');
    if status ~= 0
        docker.reason = sprintf('Docker daemon is unavailable: %s', extract_docker_error(output));
        return;
    end

    [status, ~] = system('docker compose version');
    if status == 0
        compose_base = 'docker compose';
    else
        [status, ~] = system('docker-compose version');
        if status == 0
            compose_base = 'docker-compose';
        else
            docker.reason = 'Docker Compose CLI is unavailable.';
            return;
        end
    end

    docker.available = true;
    docker.compose_cmd = sprintf('%s --project-directory %s -f %s', ...
        compose_base, shell_quote(ctx.project_root), shell_quote(ctx.compose_file));
end

function cmd = compose_command(docker, subcommand)
    cmd = sprintf('%s %s', docker.compose_cmd, subcommand);
end

function handle = make_server_handle(mode, kind, id, display, log_file, error_log)
    handle = struct( ...
        'mode', mode, ...
        'kind', kind, ...
        'id', id, ...
        'display', display, ...
        'log_file', log_file, ...
        'error_log', error_log);
end

function handle = normalize_server_handle(handle)
    if isempty(handle)
        return;
    end

    if isstring(handle)
        handle = char(handle);
    end

    if ischar(handle)
        token = strtrim(handle);
        if isempty(token)
            handle = [];
        elseif strcmp(token, 'external')
            handle = make_server_handle('external', 'external', '', 'existing external server', '', '');
        elseif strcmp(token, 'exec')
            handle = make_server_handle('docker', 'compose-exec', '', 'docker compose exec fsae-sim', '', '');
        else
            handle = make_server_handle('docker', 'temporary-container', token, ...
                sprintf('docker container %s', token), '', '');
        end
        return;
    end

    if ~isfield(handle, 'mode')
        handle = [];
        return;
    end

    if ~isfield(handle, 'kind')
        handle.kind = '';
    end
    if ~isfield(handle, 'id')
        handle.id = '';
    end
    if ~isfield(handle, 'display') || isempty(handle.display)
        handle.display = '';
    end
    if ~isfield(handle, 'log_file')
        handle.log_file = '';
    end
    if ~isfield(handle, 'error_log')
        handle.error_log = '';
    end
end

function desc = describe_server_handle(handle)
    handle = normalize_server_handle(handle);
    if isempty(handle)
        desc = 'no server handle';
        return;
    end

    if ~isempty(handle.display)
        desc = handle.display;
        return;
    end

    switch handle.mode
        case 'external'
            desc = 'existing external server';
        case 'local'
            desc = sprintf('local PID %s', handle.id);
        case 'docker'
            if strcmp(handle.kind, 'compose-exec')
                desc = 'docker compose exec fsae-sim';
            else
                desc = sprintf('docker container %s', handle.id);
            end
        otherwise
            desc = 'unknown server handle';
    end
end

function text = format_attempt_errors(messages)
    if isempty(messages)
        text = '';
        return;
    end

    text = sprintf('\n - %s', strjoin(messages, sprintf('\n - ')));
end

function text = join_for_error(values)
    if isempty(values)
        text = '(none)';
    else
        text = strjoin(values, ', ');
    end
end

function msg = extract_docker_error(output)
    % Return a meaningful error description from 'docker info' failure output.
    lines = split_nonempty_lines(output);
    for i = 1:numel(lines)
        l = strtrim(lines{i});
        if contains(l, 'pipe/docker_engine') || contains(l, 'docker daemon is not running') ...
                || contains(l, 'Is the docker daemon running')
            msg = 'Docker Desktop daemon is not running. Start Docker Desktop and wait until the engine finishes initializing.';
            return;
        end
    end
    for i = 1:numel(lines)
        l = strtrim(lines{i});
        if ~isempty(regexp(l, '^ERROR:', 'once')) && ~strcmp(l, 'errors pretty printing info')
            msg = l;
            return;
        end
    end
    msg = trim_single_line(output);
end

function text = trim_single_line(value)
    lines = split_nonempty_lines(value);
    if isempty(lines)
        text = '(no output)';
    else
        text = strtrim(lines{end});
    end
end

function lines = split_nonempty_lines(text)
    if isempty(text)
        lines = {};
        return;
    end

    lines = regexp(text, '\r\n|\n|\r', 'split');
    lines = lines(~cellfun('isempty', lines));
end

function container_id = parse_container_id(output)
    container_id = '';
    lines = split_nonempty_lines(strtrim(output));
    for i = numel(lines):-1:1
        line = strtrim(lines{i});
        if ~isempty(regexp(line, '^[0-9a-fA-F]{12,64}$', 'once'))
            container_id = line;
            break;
        end
    end

    if isempty(container_id) && ~isempty(lines)
        container_id = strtrim(lines{end});
    end

    if numel(container_id) > 12
        container_id = container_id(1:12);
    end
end

function quoted = shell_quote(value)
    quoted = ['"' char(value) '"'];
end

function quoted = ps_quote(value)
    quoted = ['''' strrep(char(value), '''', '''''') ''''];
end
