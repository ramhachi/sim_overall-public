% profile_rl_bridge.m
% RLサーバーとMATLAB環境のパフォーマンスを計測するスクリプト

addpath('matlab/rl_driver');

fprintf('=== Profiling TCP Client ===\n');
client = FSAESimClient('localhost', get_rl_server_host_port());
client.connect();
client.init('circle');

% Reset performance
N_RESET = 100;
tic;
for i = 1:N_RESET
    client.reset();
end
t_reset = toc / N_RESET;

% Step performance
N_STEP = 1000;
client.reset();
tic;
for i = 1:N_STEP
    client.step(0, 0.5, 0);
end
t_step = toc / N_STEP;

client.disconnect();

fprintf('TCP Client Avg Reset: %.4f ms\n', t_reset * 1000);
fprintf('TCP Client Avg Step: %.4f ms\n', t_step * 1000);

fprintf('\n=== Profiling RL Environment ===\n');
env = FSAEEnvironment('circle');

% Env Reset
tic;
for i = 1:N_RESET
    reset(env);
end
t_env_reset = toc / N_RESET;

% Env Step
reset(env);
tic;
for i = 1:N_STEP
    step(env, [0; 0.5; 0]);
end
t_env_step = toc / N_STEP;

fprintf('RL Env Avg Reset: %.4f ms\n', t_env_reset * 1000);
fprintf('RL Env Avg Step: %.4f ms\n', t_env_step * 1000);

% 1エピソード(3000ステップ)の推定時間
est_ep_time = t_env_reset + (t_env_step * 3000);
fprintf('\nEstimated Time for 1 Episode (3000 steps): %.2f seconds\n', est_ep_time);
