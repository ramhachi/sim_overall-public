classdef VehicleSimRunner < handle
    % VEHICLESIMRUNNER 永続的なDockerコンテナを使用して高速にシミュレーションを実行するクラス

    properties
        RawContainerName    % ユーザー定義のコンテナ名（docker-compose.yml内）
        ProjectRoot         % プロジェクトのルートディレクトリ
        DockerCmd           % dockerコマンド（docker compose exec ...）
    end

    methods
        function obj = VehicleSimRunner(project_root, container_name)
            % コンストラクタ
            % project_root: docker-compose.ymlがあるディレクトリ
            if nargin < 1
                obj.ProjectRoot = pwd;
            else
                obj.ProjectRoot = project_root;
            end

            % Check if docker-compose.yml exists in ProjectRoot, if not check parent
            if ~exist(fullfile(obj.ProjectRoot, 'docker-compose.yml'), 'file')
                parentDir = fileparts(obj.ProjectRoot);
                if exist(fullfile(parentDir, 'docker-compose.yml'), 'file')
                    obj.ProjectRoot = parentDir;
                end
            end

            if nargin < 2
                obj.RawContainerName = 'fsae-sim'; % docker-compose.ymlのservice名
            else
                obj.RawContainerName = container_name;
            end

            % Dockerコマンドの構築
            % -T: TTYを使用しない（重要：Matlabのsystemコマンドでハングしないため）
            compose_file = fullfile(obj.ProjectRoot, 'docker-compose.yml');
            obj.DockerCmd = sprintf('docker compose -f %s exec -T %s', ...
                obj.shellQuote(compose_file), obj.RawContainerName);

            % コンテナが起動していなければ自動起動
            obj.ensureContainerRunning();
        end

        function [result, success] = run(obj, params_struct, track_name, track_radius, sim_time, varargin)
            % シミュレーション実行
            % params_struct: 車両パラメータの構造体（[]の場合はデフォルトパラメータを使用）
            %
            % Name-value options:
            %   driver_mode      - 'pid' (default) or 'rl'
            %   rl_actor_dir     - path to RL actor export directory
            %   extra_cli_args   - extra CLI arguments passed verbatim
            %   initial_speed    - initial vehicle speed [m/s] (-1 = default)
            %   timeout_sec      - per-sample timeout in seconds; 0 = disabled (default)
            %                      Uses Linux `timeout` inside the container.
            %   params_file_host - explicit host-side path to a pre-written params JSON.
            %                      Caller owns the file lifecycle; runner never deletes it.
            %                      Takes precedence over params_struct when both are given.
            %   sample_log_file  - host path to a JSONL append log; each successful or
            %                      failed sample result is appended as one JSON line.
            %                      Enables resumable execution by the orchestrator.

            if nargin < 3, track_name = 'circle'; end
            if nargin < 4, track_radius = 30.0; end
            if nargin < 5, sim_time = 25.0; end

            p = inputParser;
            addParameter(p, 'driver_mode', 'pid', @(x) ischar(x) || isstring(x));
            addParameter(p, 'rl_actor_dir', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'extra_cli_args', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'initial_speed', -1.0, @isnumeric);
            addParameter(p, 'timeout_sec', 0, @(x) isnumeric(x) && x >= 0);
            addParameter(p, 'params_file_host', '', @(x) ischar(x) || isstring(x));
            addParameter(p, 'sample_log_file', '', @(x) ischar(x) || isstring(x));
            parse(p, varargin{:});

            driver_mode = lower(char(p.Results.driver_mode));
            rl_actor_dir = char(p.Results.rl_actor_dir);
            extra_cli_args = char(p.Results.extra_cli_args);
            initial_speed = p.Results.initial_speed;
            timeout_sec = p.Results.timeout_sec;
            params_file_host_explicit = char(p.Results.params_file_host);
            sample_log_file = char(p.Results.sample_log_file);

            if ~ismember(driver_mode, {'pid', 'rl'})
                error('driver_mode must be ''pid'' or ''rl''.');
            end
            driver_args = obj.buildDriverArgs(driver_mode, rl_actor_dir);

            % initial_speed オプション
            if initial_speed >= 0.0
                initial_speed_arg = sprintf('--initial-speed %.4f', initial_speed);
            else
                initial_speed_arg = '';
            end

            % ---- パラメータファイルの解決 ----------------------------------------
            % Priority 1: explicit host path provided by orchestrator (never deleted).
            % Priority 2: params_struct → auto-generated temp file (caller-managed delete
            %             for backward compat; UUID-based name to avoid collisions).
            temp_path_host = '';  % non-empty only when we own the temp file

            if ~isempty(params_file_host_explicit)
                % Orchestrator-managed file: map host path to container path.
                params_file_container = obj.hostToContainerPath(params_file_host_explicit);
                params_file_arg = sprintf('--params_file ''%s''', params_file_container);

            elseif ~isempty(params_struct) && isstruct(params_struct)
                % Auto-generate from struct. Use UUID to avoid timestamp collisions.
                runtime_dir_host = fullfile(obj.ProjectRoot, 'output', 'runtime');
                if ~exist(runtime_dir_host, 'dir')
                    mkdir(runtime_dir_host);
                end
                uid = strrep(char(java.util.UUID.randomUUID()), '-', '');
                temp_fname = sprintf('params_%s.json', uid);
                temp_path_host = fullfile(runtime_dir_host, temp_fname);
                params_file_container = sprintf('/workspace/output/runtime/%s', temp_fname);

                json_str = jsonencode(params_struct);
                fid = fopen(temp_path_host, 'w');
                if fid == -1
                    error('VehicleSimRunner:fileError', ...
                        'Failed to create temp param file: %s', temp_path_host);
                end
                fwrite(fid, json_str, 'char');
                fclose(fid);
                params_file_arg = sprintf('--params_file ''%s''', params_file_container);
            else
                params_file_arg = '';
            end

            % ---- タイムアウトプレフィックス ----------------------------------------
            % Uses Linux `timeout` inside the container (reliable cross-platform).
            if timeout_sec > 0
                timeout_prefix = sprintf('timeout %d ', floor(timeout_sec));
            else
                timeout_prefix = '';
            end

            % コマンド構築
            cmd = sprintf('%s bash -c "cd build && %s./cpp/laptime_sim --track %s --radius %.2f --time %.2f %s %s %s --output_json %s"', ...
                obj.DockerCmd, timeout_prefix, track_name, track_radius, sim_time, ...
                driver_args, initial_speed_arg, extra_cli_args, params_file_arg);

            % 実行
            [status, cmdout] = system(cmd);

            % 一時ファイル削除（auto-generated temp files のみ、明示的パスは触らない）
            if ~isempty(temp_path_host) && isfile(temp_path_host)
                delete(temp_path_host);
            end

            % JSONパース
            try
                result = obj.parseJsonResult(cmdout);
                success = (status == 0) && ~obj.isFailedResult(result);
            catch e
                fprintf('Failed to parse result: %s\n', e.message);
                fprintf('Raw output: %s\n', cmdout);
                result = [];
                success = false;
            end

            if ~success
                fprintf('Simulation failed with status %d\n', status);
                if ~isempty(result) && isstruct(result) && isfield(result, 'failure_code')
                    fprintf('Failure code: %s\n', char(string(result.failure_code)));
                end
                if ~isempty(result) && isstruct(result) && isfield(result, 'failure_message')
                    fprintf('Failure message: %s\n', char(string(result.failure_message)));
                end
                fprintf('Output: %s\n', cmdout);
            end

            % DEBUG行を表示
            debug_lines = regexp(cmdout, 'DEBUG:.*', 'match');
            if ~isempty(debug_lines)
                for i = 1:length(debug_lines)
                    fprintf('%s\n', debug_lines{i});
                end
            end

            % アペンド専用サンプルログ（orchestratorによる再開可能実行のため）
            if ~isempty(sample_log_file)
                obj.appendSampleLog(sample_log_file, result, success, params_file_host_explicit);
            end
        end
    end

    methods (Access = private)
        function driver_args = buildDriverArgs(obj, driver_mode, rl_actor_dir)
            if strcmp(driver_mode, 'pid')
                driver_args = '--driver pid';
                return;
            end

            if isempty(rl_actor_dir)
                rl_actor_dir_host = resolve_default_rl_actor_export_dir(obj.ProjectRoot);
                if isempty(rl_actor_dir_host)
                    rl_actor_dir_host = fullfile(obj.ProjectRoot, 'matlab', 'rl_driver', 'rl_actor_export');
                end
                requested_actor_dir = rl_actor_dir_host;
            else
                rl_actor_dir_host = rl_actor_dir;
                requested_actor_dir = rl_actor_dir;
                is_absolute = startsWith(rl_actor_dir_host, filesep) || ...
                    ~isempty(regexp(rl_actor_dir_host, '^[A-Za-z]:[\\/]', 'once'));
                if ~isfolder(rl_actor_dir_host) && ~is_absolute
                    rl_actor_dir_host = fullfile(obj.ProjectRoot, rl_actor_dir_host);
                end
            end

            if ~isfolder(rl_actor_dir_host)
                error('RL actor directory not found: %s', requested_actor_dir);
            end

            rl_actor_dir_host = char(java.io.File(rl_actor_dir_host).getCanonicalPath());
            project_root_abs = char(java.io.File(obj.ProjectRoot).getCanonicalPath());
            if ~startsWith(rl_actor_dir_host, project_root_abs)
                error('rl_actor_dir must be under project root: %s', project_root_abs);
            end

            required_files = {'rl_actor_weights.txt', 'rl_normalization_params.txt', 'rl_actor_test_data.csv'};
            for rf = 1:numel(required_files)
                if ~isfile(fullfile(rl_actor_dir_host, required_files{rf}))
                    error('Missing RL export file: %s', fullfile(rl_actor_dir_host, required_files{rf}));
                end
            end

            rel_actor_path = erase(rl_actor_dir_host, [project_root_abs filesep]);
            rel_actor_path = strrep(rel_actor_path, '\', '/');
            rl_actor_dir_container = ['/workspace/' rel_actor_path];
            driver_args = sprintf('--driver rl --rl-actor-dir ''%s''', rl_actor_dir_container);
        end

        function ensureContainerRunning(obj)
            % コンテナが起動しているか確認し、起動していなければ起動する
            % Hard-fail: any inability to reach the container is a fatal preflight error.
            [status, ~] = system(sprintf('%s echo ok 2>&1', obj.DockerCmd));
            if status ~= 0
                fprintf('Docker container not running. Starting with docker compose up -d...\n');
                compose_file = fullfile(obj.ProjectRoot, 'docker-compose.yml');
                [st, out] = system(sprintf('docker compose -f %s up -d 2>&1', ...
                    obj.shellQuote(compose_file)));
                if st ~= 0
                    error('VehicleSimRunner:dockerStartFailed', ...
                        'Failed to start Docker container: %s', out);
                end
                % コンテナ準備完了まで待機（最大30秒）
                for i = 1:15
                    pause(2);
                    [status2, ~] = system(sprintf('%s echo ok 2>&1', obj.DockerCmd));
                    if status2 == 0
                        fprintf('Docker container is ready.\n');
                        return;
                    end
                end
                error('VehicleSimRunner:dockerNotReady', ...
                    'Docker container did not become ready within 30 seconds.');
            end
        end

        function result = parseJsonResult(~, cmdout)
            % Try explicit delimiters first (%%JSON_BEGIN%% / %%JSON_END%%).
            % This is robust against mixed stdout output from the container.
            % Falls back to greedy brace extraction for backward compatibility with
            % simulator binaries that pre-date the delimiter protocol.
            begin_tok = '%%JSON_BEGIN%%';
            end_tok   = '%%JSON_END%%';
            b_idx = strfind(cmdout, begin_tok);
            e_idx = strfind(cmdout, end_tok);
            if ~isempty(b_idx) && ~isempty(e_idx)
                json_start   = b_idx(end) + length(begin_tok);
                json_end_pos = e_idx(end) - 1;
                json_content = strtrim(cmdout(json_start:json_end_pos));
                result = jsondecode(json_content);
                return;
            end

            % Fallback: greedy first-{ to last-} (legacy behaviour).
            json_start = strfind(cmdout, '{');
            json_end   = strfind(cmdout, '}');
            if isempty(json_start) || isempty(json_end)
                error('VehicleSimRunner:noJson', 'No JSON found in output');
            end
            json_content = cmdout(json_start(1):json_end(end));
            result = jsondecode(json_content);
        end

        function failed = isFailedResult(~, result)
            if ~isstruct(result)
                failed = false;
                return;
            end
            % Explicit failure status reported by the simulator
            if isfield(result, 'status') && strcmpi(char(string(result.status)), 'failed')
                failed = true;
                return;
            end
            failed = false;
        end

        function container_path = hostToContainerPath(obj, host_path)
            % Convert an absolute host path under ProjectRoot to the container-side
            % /workspace/... equivalent.  Errors hard if the path escapes ProjectRoot.
            host_abs    = char(java.io.File(host_path).getAbsolutePath());
            root_abs    = char(java.io.File(obj.ProjectRoot).getAbsolutePath());
            if ~startsWith(host_abs, root_abs)
                error('VehicleSimRunner:invalidPath', ...
                    'params_file_host must be under ProjectRoot (%s), got: %s', ...
                    root_abs, host_path);
            end
            rel = host_abs(length(root_abs)+2:end);          % strip root + separator
            container_path = ['/workspace/' strrep(rel, '\', '/')];
        end

        function appendSampleLog(~, log_file, result, success, params_file_host)
            % Append one JSON line to log_file (append-only; creates file if absent).
            % The orchestrator reads this file to implement resumable execution.
            try
                entry = struct();
                entry.timestamp = datestr(now, 'yyyy-mm-ddTHH:MM:SS');
                entry.success   = success;
                if ~isempty(params_file_host)
                    entry.params_file = params_file_host;
                end
                if ~isempty(result) && isstruct(result)
                    entry.result = result;
                end
                fid = fopen(log_file, 'a');
                if fid ~= -1
                    fprintf(fid, '%s\n', jsonencode(entry));
                    fclose(fid);
                end
            catch log_err
                fprintf('[VehicleSimRunner] Warning: failed to write sample log: %s\n', ...
                    log_err.message);
            end
        end

        function quoted = shellQuote(~, value)
            escaped = strrep(char(value), char(39), [char(39) '"' char(39) '"' char(39)]);
            quoted = [char(39) escaped char(39)];
        end
    end
end
