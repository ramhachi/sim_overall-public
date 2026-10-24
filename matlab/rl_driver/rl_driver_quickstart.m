%% RL_DRIVER_QUICKSTART.m
% RLドライバー学習の全コマンドをまとめたクイックスタートガイド
%
% ■ 使い方
%   このファイルを開いて、実行したいセクションを選んでCtrl+Enter（Section実行）

%% ========== 0. 初期設定（毎回まず実行）==========
RL_DIR = fileparts(mfilename('fullpath'));
addpath(RL_DIR);
addpath(fileparts(RL_DIR));
fprintf('✓ パス設定完了: %s\n', RL_DIR);

%% ========== 1. Dockerビルド＋サーバー起動 ==========
% C++をDockerコンテナ内でビルドし、既定のhostポート22345でTCPサーバーを起動する
startRLServer();

%% ========== 2. 接続テスト ==========
client = FSAESimClient('localhost', get_rl_server_host_port());
client.connect();
trackLen = client.init('circle');
fprintf('✓ 接続OK: circleトラック長 = %.1fm\n', trackLen);
obs = client.reset();
[obs2, info] = client.step(0.0, 0.5, 0.0);
fprintf('✓ STEPテスト: vx=%.2f m/s, s=%.2f m\n', info(1), info(4));

% オートクロスも確認
trackLen2 = client.init('autocross');
fprintf('✓ autocrossトラック長 = %.1fm\n', trackLen2);
client.disconnect();

%% ========== 3a. 円形コース特化学習（推奨: まずこれから）==========
trainCircleSimple();

%% ========== 3b. 単純学習（1つのトラック）==========
trainRLDriver('circle');

%% ========== 3c. カリキュラム学習 ==========
% 直線→円→スキッドパッド→オートクロス の段階学習
% + パラメータランダム化（Domain Randomization）
trainWithCurriculum('autocross', 'RandInterval', 100);

%% ========== 3d. ランダム化なしでカリキュラム学習 ==========
trainWithCurriculum('autocross', 'DomainRand', false, 'RandInterval', 100);

%% ========== 3e. ランダム化の強度を調整 ==========
% RandStrength: 0.0（パラメータ固定）〜 1.0（全範囲からランダム）
% RandInterval: Nエピソードごとにパラメータを変更
trainWithCurriculum('autocross', 'RandStrength', 0.5, 'RandInterval', 100);

%% ========== 3f. 特定パラメータで学習 ==========
artifact = load_vehicle_artifact('../../optimized_autocross.json');
trainRLDriver('autocross', artifact.params);

%% ========== 3g. 端末停止後の phase 3/4 recovery 再開 ==========
% 既存 deployment があれば保持し、なければ安全側の fallback を公開してから
% 最新 checkpoint / progress に基づいて recovery run を再開する
resumePhase34RecoveryAfterRestart();

%% ========== 3h. recovery 完了後の ace-driver 継続学習 ==========
% 直近の recovery final_agent から、より高速な autocross 指向へ継続学習する
trainAceDriverFromRecovery();

%% ========== 3i. 端末停止後の ace-driver 継続再開 ==========
% ace-driver run の progress を優先し、progress.json が無い中断 run でも
% 既存 AgentXX.mat checkpoint から安全に再開する
resumeAceDriverAfterRestart();

%% ========== 3j. ace final から phase4 追加学習 ==========
% 完了済み ace final_agent から、新しい run ディレクトリで phase4 を追加学習する
trainAceDriverPhase4Extension();

%% ========== 3k. 端末停止後の ace phase4 追加学習再開 ==========
% ace final から開始した phase4 追加学習 run を安全に開始/再開する
resumeAceDriverPhase4ExtensionAfterRestart();

%% ========== 4. 学習済みエージェントで評価 ==========
evaluateRLDriver('rl_results/curriculum/final_agent.mat', 'autocross');

%% ========== 4f. RL laptime_sim ベンチマーク ==========
results = benchmarkRLLaptimeSim();
disp(results);

%% ========== 4b. 受け入れ基準チェック（3トラック）==========
% 完走率・ラップタイム・コース逸脱率をまとめて判定
validateRLReadiness('rl_results/curriculum/final_agent.mat', ...
    'Tracks', {'autocross', 'skidpad', 'circle'}, ...
    'Episodes', 5, ...
    'SaveJson', 'rl_results/curriculum/readiness_report.json');

%% ========== 4c. 受け入れゲート一括実行（推奨）==========
% 1) C++用エクスポート 2) MATLAB↔C++一致性チェック 3) 受け入れ判定
gate = runRLReadinessGate('rl_results/curriculum/final_agent.mat', ...
    'ActorExportDir', 'rl_actor_export', ...
    'Tracks', {'autocross', 'skidpad', 'circle'}, ...
    'Episodes', 5);
disp(gate.summary.overall_pass);
disp(gate.report_path);

%% ========== 4d. 最新 recovery final_agent の受け入れゲート ==========
gate = runRLReadinessGate( ...
    'rl_results/curriculum_torque_frame_fresh_v1/phase34_recovery_from_phase3_final/final_agent.mat', ...
    'ActorExportDir', 'rl_results/curriculum_torque_frame_fresh_v1/phase34_recovery_from_phase3_final/final_actor_export', ...
    'Tracks', {'autocross', 'skidpad', 'circle'}, ...
    'Episodes', 5, ...
    'SaveDir', 'rl_results/curriculum_torque_frame_fresh_v1/phase34_recovery_from_phase3_final', ...
    'PIDBaselineLapTime', getDefaultRLPIDBaselineLapTimes());
disp(gate.summary.overall_pass);
disp(gate.report_path);

%% ========== 5. C++用エクスポート（学習後）==========
agentData = load('rl_results/curriculum/final_agent.mat');
deployVehicleParams = resolve_rl_vehicle_params();  % 非デフォルト車両で使う場合はここを上書き
exportActorForCpp(agentData.agent, 'rl_actor_export', deployVehicleParams);

%% ========== 6. サーバー管理コマンド ==========
PROJECT_ROOT = fileparts(fileparts(RL_DIR));
COMPOSE = sprintf('docker compose -f "%s/docker-compose.yml"', PROJECT_ROOT);
EXEC    = sprintf('%s exec -T fsae-sim bash -c', COMPOSE);

% サーバー状態確認
[~, o] = system(sprintf('%s "ps aux | grep fsae_rl_server | grep -v grep"', EXEC));
fprintf('サーバープロセス:\n%s\n', o);

% サーバー停止
system(sprintf('%s "pkill -f fsae_rl_server; true"', EXEC));
fprintf('✓ サーバー停止\n');

% サーバー再起動
startRLServer();
