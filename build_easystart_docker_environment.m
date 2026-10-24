function success = build_easystart_docker_environment()
%BUILD_EASYSTART_DOCKER_ENVIRONMENT Start the Docker service and build the simulator.

success = false;

fprintf('\n🛠️  環境構築を開始します...\n');

project_root = fileparts(mfilename('fullpath'));
current_dir = pwd;
previous_docker_platform = getenv('DOCKER_DEFAULT_PLATFORM');
cleanup = onCleanup(@() restore_environment(current_dir, previous_docker_platform)); %#ok<NASGU>

cd(project_root);

is_windows = ispc();
profile = resolve_easystart_docker_build_profile();
host_ui_port = select_host_port('FSAE_HOST_UI_PORT', 8080);
host_rl_port = select_host_port('FSAE_HOST_RL_PORT', 22345);
host_fem_port = select_host_port('FSAE_HOST_FEM_PORT', 50051);

if is_windows
    cmd_docker_check = 'docker info >nul 2>&1';
else
    cmd_docker_check = 'docker info > /dev/null 2>&1';
end

[status, ~] = system(cmd_docker_check);
if status ~= 0
    fprintf('\n⛔ エラー: Dockerが起動していません！\n');
    fprintf('Docker Desktopを起動してから、もう一度実行してください。\n');
    return;
end

if ~isempty(profile.docker_platform)
    setenv('DOCKER_DEFAULT_PLATFORM', profile.docker_platform);
end

fprintf('  - Docker target platform: %s\n', display_value(profile.docker_platform, 'default'));
fprintf('  - CPU最適化プロファイル: %s (%s)\n', ...
    profile.cpu_optimization_profile, display_value(profile.raw_host_arch, profile.host_arch));
fprintf('  - CMake generator: %s\n', profile.generator);
fprintf('  - Host ports: UI=%d, RL=%d, FEM=%d\n', host_ui_port, host_rl_port, host_fem_port);

fprintf('1. Dockerコンテナを起動・更新しています...\n');
[status, output] = system('docker compose up -d --build --remove-orphans');
if status ~= 0
    fprintf('\n⛔ エラー: Dockerコンテナの起動に失敗しました。\n');
    fprintf('詳細:\n%s\n', output);
    fprintf('\nトラブルシューティング:\n');
    fprintf('  - Docker Desktopが起動していることを確認してください\n');
    fprintf('  - ターミナルで以下を実行して詳細を確認: docker compose logs\n');
    return;
end

fprintf('コンテナの準備を待っています...\n');
if ~wait_for_service_ready('fsae-sim', 20)
    fprintf('\n⛔ エラー: fsae-sim コンテナの準備が完了しませんでした。\n');
    fprintf('ターミナルで以下を実行して状態を確認してください:\n');
    fprintf('  docker compose ps\n');
    fprintf('  docker compose logs fsae-sim\n');
    return;
end

fprintf('\n2. C++コードをコンパイルしています...\n');
fprintf('ビルド中... (初回は数分かかります)\n');

build_targets = strjoin(profile.build_targets, ' ');
build_cmd = sprintf([ ...
    'docker compose exec -T fsae-sim bash /workspace/scripts/easystart_container_build.sh ', ...
    '%s %s %s %s'], ...
    profile.build_signature, profile.generator, profile.cpu_optimization_profile, build_targets);

[status, build_output] = system(build_cmd);
if status == 0
    fprintf('\n✅  環境構築とビルドが完了しました！準備OKです。\n');
    fprintf('  - Active build profile: %s\n', profile.build_signature);
    success = true;
else
    if status == 2
        fprintf('\n⛔ エラー: CMake設定に失敗しました。\n');
        fprintf('出力:\n%s\n', build_output);
        fprintf('\n詳細なエラー情報を確認するには、ターミナルで以下を実行してください:\n');
        fprintf('  docker compose exec fsae-sim bash\n');
        fprintf('  cd /workspace/build && cmake -S /workspace -B . -G %s -DCMAKE_BUILD_TYPE=Release -DFSAE_CPU_OPTIMIZATION_PROFILE=%s\n', ...
            profile.generator, profile.cpu_optimization_profile);
    else
        fprintf('\n⛔ エラー: ビルドに失敗しました。\n');
        fprintf('ビルド出力:\n%s\n', build_output);
        fprintf('\n詳細なエラー情報を確認するには、ターミナルで以下を実行してください:\n');
        fprintf('  docker compose exec fsae-sim bash\n');
        fprintf('  cd /workspace/build && cmake --build . --verbose\n');
    end
end
end

function ready = wait_for_service_ready(service_name, max_attempts)
ready = false;

for attempt = 1:max_attempts
    [status, ~] = system(sprintf('docker compose exec -T %s bash -lc "true"', service_name));
    if status == 0
        ready = true;
        return;
    end
    pause(1);
end
end

function restore_environment(original_dir, previous_docker_platform)
cd(original_dir);

if isempty(previous_docker_platform)
    setenv('DOCKER_DEFAULT_PLATFORM', '');
else
    setenv('DOCKER_DEFAULT_PLATFORM', previous_docker_platform);
end
end

function value = display_value(primary_value, fallback_value)
value = primary_value;
if isempty(value)
    value = fallback_value;
end
end

function port = select_host_port(env_name, preferred_port)
existing_value = strtrim(getenv(env_name));
if ~isempty(existing_value)
    parsed = str2double(existing_value);
    if isfinite(parsed) && parsed > 0 && mod(parsed, 1) == 0
        port = parsed;
        setenv(env_name, num2str(port));
        return;
    end
end

port = find_available_tcp_port(preferred_port);
setenv(env_name, num2str(port));
end

function port = find_available_tcp_port(preferred_port)
if can_bind_tcp_port(preferred_port)
    port = preferred_port;
    return;
end

server_socket = [];
cleanup_socket = [];
try
    server_socket = java.net.ServerSocket(0);
    cleanup_socket = onCleanup(@() server_socket.close()); %#ok<NASGU>
    port = server_socket.getLocalPort();
catch
    if ~isempty(server_socket)
        server_socket.close();
    end
    error('Failed to allocate a free host TCP port.');
end
end

function available = can_bind_tcp_port(port)
available = false;
server_socket = [];

try
    server_socket = java.net.ServerSocket(port);
    server_socket.setReuseAddress(true);
    available = true;
catch
    available = false;
end

if ~isempty(server_socket)
    server_socket.close();
end
end
