function startRLServer(projectRoot)
    % STARTRLSERVER Docker内でRL学習用サーバーを起動
    %
    % startRLServer()                        % カレントディレクトリ基準
    % startRLServer('/path/to/sim_overall')  % プロジェクトルート指定
    %
    % 前提: docker compose up -d でコンテナが起動済みであること
    %       コンテナ内でC++がビルド済みであること
    
    if nargin < 1
        % MATLABファイルからプロジェクトルートを推定
        thisDir = fileparts(mfilename('fullpath'));
        projectRoot = fileparts(fileparts(thisDir));  % matlab/rl_driver → sim_overall
    end
    
    composeFile = fullfile(projectRoot, 'docker-compose.yml');
    if ~exist(composeFile, 'file')
        error('docker-compose.yml not found at: %s', composeFile);
    end

    hostRlPort = get_rl_server_host_port();
    pidFile = '/tmp/rl_server.pid';
    
    %% コンテナ起動確認
    fprintf('Checking Docker container status...\n');
    [status, output] = system(sprintf( ...
        'docker compose -f "%s" ps --format "{{.State}}" fsae-sim', composeFile));
    
    if status ~= 0 || isempty(strfind(strtrim(output), 'running'))
        fprintf('Starting Docker container...\n');
        system(sprintf('docker compose -f "%s" up -d', composeFile));
        pause(2);
    end
    
    %% ビルド確認
    fprintf('Building RL server in container...\n');
    % ホストOSに依存しないよう、コンテナ内で CMake 経由のビルドを使用
    buildCmd = sprintf( ...
        'docker compose -f "%s" exec -T fsae-sim bash -c "cd /workspace/build && cmake .. -DCMAKE_BUILD_TYPE=Release -Wno-dev > /dev/null 2>&1 && cmake --build . --target fsae_rl_server --parallel 2>&1"', ...
        composeFile);
    [status, output] = system(buildCmd);
    if status ~= 0
        fprintf('Build output:\n%s\n', output);
        error('Failed to build fsae_rl_server');
    end
    fprintf('Build successful.\n');
    
    %% 既存サーバーを停止
    % PID file を優先し、残骸がある場合のみ ps -C で PID を列挙して個別終了する
    killFmt = ['docker compose -f "%s" exec -T fsae-sim bash -lc ' ...
               '''if [ -f ' pidFile ' ]; then ' ...
               'pid=$(cat ' pidFile ' 2>/dev/null); ' ...
               'if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then kill "$pid" 2>/dev/null; sleep 1; if kill -0 "$pid" 2>/dev/null; then kill -9 "$pid" 2>/dev/null; fi; fi; ' ...
               'rm -f ' pidFile '; ' ...
               'fi; ' ...
               'for pid in $(ps -C fsae_rl_server -o pid= 2>/dev/null); do ' ...
               'kill "$pid" 2>/dev/null || true; sleep 0.2; if kill -0 "$pid" 2>/dev/null; then kill -9 "$pid" 2>/dev/null || true; fi; ' ...
               'done; ' ...
               'sleep 1; true'''];
    system(sprintf(killFmt, composeFile));
    pause(2);

    %% サーバー起動（バックグラウンド）
    % exec -d はプロセスを維持しないため nohup + & を使用
    fprintf('Starting RL server on container port 12345 (host port %d)...\n', hostRlPort);
    startCmd = sprintf( ...
        ['docker compose -f "%s" exec -T fsae-sim bash -lc ' ...
         '''rm -f ' pidFile '; ' ...
         'nohup /workspace/build/cpp/fsae_rl_server 12345 > /tmp/rl_server.log 2>&1 & ' ...
         'echo $! > ' pidFile ''''], ...
        composeFile);
    [status, output] = system(startCmd);
    if status ~= 0
        fprintf('Start output:\n%s\n', output);
        error('Failed to start RL server');
    end
    pause(1);  % バインド完了を待つ

    %% 接続テスト（最大15秒リトライ）
    fprintf('Waiting for RL server to be ready');
    maxRetries = 15;
    connected = false;
    for attempt = 1:maxRetries
        pause(1);
        fprintf('.');
        try
            client = FSAESimClient('localhost', hostRlPort);
            client.connect();
            trackLen = client.init('circle');
            client.disconnect();
            fprintf('\nRL Server ready. Test connection OK (circle track = %.1fm)\n', trackLen);
            connected = true;
            break;
        catch
            % まだ起動中 → リトライ
        end
    end
    fprintf('\n');
    if ~connected
        % サーバーログを表示してデバッグ情報を提供
        [~, logOut] = system(sprintf( ...
            'docker compose -f "%s" exec -T fsae-sim bash -c "tail -20 /tmp/rl_server.log 2>/dev/null || echo no_log"', ...
            composeFile));
        fprintf('RL server log:\n%s\n', logOut);
        error('RL server did not become ready within %d seconds.', maxRetries);
    end
end
