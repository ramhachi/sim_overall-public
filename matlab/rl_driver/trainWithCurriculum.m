function trainWithCurriculum(finalTrackName, varargin)
    % TRAINWITHCURRICULUM カリキュラム学習 + パラメータランダム化
    %
    % trainWithCurriculum()                   % デフォルト: autocross最終
    % trainWithCurriculum('autocross')
    % trainWithCurriculum('autocross', 'DomainRand', true)
    % trainWithCurriculum('autocross', 'DomainRand', true, 'RandInterval', 100)
    %
    % カリキュラム:
    %   Phase 1: 直線（直進・加減速学習）
    %   Phase 2: 定円R=30m（定常旋回学習）
    %   Phase 3: スキッドパッド（円旋回のコース追従）
    %   Phase 4: オートクロス（複合コース全体走行）
    %
    % Domain Randomization:
    %   optimizer-aligned な車両パラメータ群を、
    %   100エピソード単位で切り替えながら汎化性能を向上。
    ensure_rl_driver_paths();

    if nargin < 1, finalTrackName = 'autocross'; end
    
    %% オプション解析
    p = inputParser;
    addParameter(p, 'DomainRand', true);        % パラメータランダム化
    addParameter(p, 'RandInterval', 100);       % 何エピソードごとにパラメータ変更
    addParameter(p, 'RandStrength', 0.3);       % ランダム化の強度 (0=なし, 1=全範囲)
    addParameter(p, 'ResumeAgent', []);          % 既存エージェントから再開
    addParameter(p, 'SaveBaseDir', fullfile('rl_results', 'curriculum'));
    addParameter(p, 'PhaseEpisodes', [1000, 3000, 3000, 8000]);
    addParameter(p, 'PhaseSteps', [3000, 3000, 4000, 8000]);
    addParameter(p, 'EnvOverrides', struct());
    addParameter(p, 'PhaseTrackPools', {});
    addParameter(p, 'PhaseRewardOverrides', {});
    addParameter(p, 'StartPhase', []);
    addParameter(p, 'CompletedEpisodesOffset', []);
    addParameter(p, 'SaveAgentCriteria', 'EpisodeCount');
    addParameter(p, 'SaveAgentValue', []);
    addParameter(p, 'ScoreAveragingWindowLength', 100);
    addParameter(p, 'StopTrainingCriteria', 'AverageReward');
    addParameter(p, 'StopTrainingValue', 1e9);
    addParameter(p, 'FinalActorExportDir', '');
    addParameter(p, 'PublishFinalToDefault', false);
    addParameter(p, 'PublishVehicleParams', struct());
    addParameter(p, 'Plots', 'training-progress');
    addParameter(p, 'Verbose', true);
    parse(p, varargin{:});
    opts = p.Results;

    if numel(opts.PhaseEpisodes) ~= 4 || numel(opts.PhaseSteps) ~= 4
        error('PhaseEpisodes and PhaseSteps must each contain 4 entries.');
    end
    
    %% カリキュラム定義
    defaultTrackPools = getDefaultCurriculumTrackPools();
    defaultRewardOverrides = getDefaultCurriculumRewardOverrides(opts.PhaseSteps);
    phaseTrackPools = resolvePhaseTrackPools(defaultTrackPools, opts.PhaseTrackPools);
    phaseRewardOverrides = resolvePhaseRewardOverrides(defaultRewardOverrides, opts.PhaseRewardOverrides);

    % {代表トラック名, トラックプール, 報酬重みオーバーライド, 最大エピソード数, MaxStepsPerEpisode}
    curriculum = {
        {'straight',  phaseTrackPools{1}, ...
            phaseRewardOverrides{1}, ...
            opts.PhaseEpisodes(1), opts.PhaseSteps(1)}
        {'circle',    phaseTrackPools{2}, ...
            phaseRewardOverrides{2}, ...
            opts.PhaseEpisodes(2), opts.PhaseSteps(2)}
        {'skidpad',   phaseTrackPools{3}, ...
            phaseRewardOverrides{3}, ...
            opts.PhaseEpisodes(3), opts.PhaseSteps(3)}
        {'autocross', phaseTrackPools{4}, ...
            phaseRewardOverrides{4}, ...
            opts.PhaseEpisodes(4), opts.PhaseSteps(4)}
    };
    
    % 最終コースがautocross以外の場合、最終段を差し替え
    if ~strcmp(finalTrackName, 'autocross')
        curriculum{4} = {finalTrackName, {finalTrackName}, ...
            phaseRewardOverrides{4}, ...
            opts.PhaseEpisodes(4), opts.PhaseSteps(4)};
    end
    
    % 重複除去（例: 最終がskidpadならPhase3と4が被るので3を削除）
    trackNames = cellfun(@(c) c{1}, curriculum, 'UniformOutput', false);
    [~, uniqueIdx] = unique(trackNames, 'stable');
    curriculum = curriculum(uniqueIdx);
    
    %% パラメータランダム化の範囲 (canonical vehicle design schema 準拠)
    paramBounds = getVehicleParamBounds();
    
    %% 結果保存ディレクトリ
    saveBaseDir = opts.SaveBaseDir;
    if ~exist(saveBaseDir, 'dir')
        mkdir(saveBaseDir);
    end
    
    agent = opts.ResumeAgent;

    if ~isempty(opts.StartPhase) && isempty(agent)
        error('StartPhase requires ResumeAgent so later phases do not restart from a fresh agent.');
    end
    
    % フェーズ進捗の確認（再開時にスキップするフェーズを特定）
    startPhase = 1;
    progressFile = fullfile(saveBaseDir, 'progress.json');
    completedEpisodes = 0;
    if ~isempty(agent) && isfile(progressFile) && isempty(opts.StartPhase)
        prog = readProgressFile(progressFile);
        startPhase = prog.last_completed_phase + 1;
        if isfield(prog, 'completed_episodes')
            completedEpisodes = prog.completed_episodes;
        else
            completedEpisodes = sum(cellfun(@(c) c{4}, curriculum(1:min(prog.last_completed_phase, numel(curriculum)))));
        end
        fprintf('  → Phase %d から再開（1〜%d は完了済み: %s）\n', ...
            startPhase, startPhase - 1, prog.track);
    end
    if ~isempty(opts.StartPhase)
        if opts.StartPhase < 1 || opts.StartPhase > length(curriculum)
            error('StartPhase must be within [1, %d].', length(curriculum));
        end
        startPhase = opts.StartPhase;
        if isempty(opts.CompletedEpisodesOffset)
            completedEpisodes = 0;
        else
            completedEpisodes = opts.CompletedEpisodesOffset;
        end
        fprintf('  → StartPhase override: Phase %d から開始（completed_episodes=%d）\n', ...
            startPhase, completedEpisodes);
    end
    if startPhase > length(curriculum)
        fprintf('全フェーズ完了済みです。エクスポートのみ実行します。\n');
    end

    for phase = startPhase:length(curriculum)
        trackName      = curriculum{phase}{1};
        trackPool      = curriculum{phase}{2};
        rewardOverride = curriculum{phase}{3};
        maxEpisodes    = curriculum{phase}{4};
        maxSteps       = curriculum{phase}{5};
        
        fprintf('\n========================================\n');
        fprintf('  Curriculum Phase %d/%d: %s\n', phase, length(curriculum), trackName);
        fprintf('  Max Episodes: %d, Max Steps: %d\n', maxEpisodes, maxSteps);
        fprintf('  Domain Randomization: %s (strength=%.1f, interval=%d)\n', ...
            mat2str(opts.DomainRand), opts.RandStrength, opts.RandInterval);
        fprintf('========================================\n');
        
        % 環境構築（ベースパラメータで初期化）
        env = FSAEEnvironment(trackName);
        env.MaxEpisodeSteps = maxSteps;
        env.EpisodeIndexOffset = completedEpisodes;
        env.TrackPool = trackPool;
        applyRLEnvOverrides(env, opts.EnvOverrides, 'Env override');
        if numel(trackPool) > 1
            fprintf('  Track pool: %s\n', strjoin(trackPool, ', '));
        end
        
        % 報酬重みオーバーライド
        fields = fieldnames(rewardOverride);
        for f = 1:length(fields)
            if isprop(env, fields{f})
                env.(fields{f}) = rewardOverride.(fields{f});
                fprintf('  Reward override: %s = %.2f\n', fields{f}, rewardOverride.(fields{f}));
            end
        end
        
        % 初回はエージェント新規作成
        if isempty(agent)
            agent = createSACAgent(env);
        else
            assertAgentObservationDimension(agent, getObservationInfo(env).Dimension(1), 'ResumeAgent');
        end
        
        % 保存ディレクトリ
        saveDir = fullfile(saveBaseDir, sprintf('phase%d_%s', phase, trackName));
        if ~exist(saveDir, 'dir'), mkdir(saveDir); end
        
        % Domain Randomization 設定（環境クラス側で処理）
        if opts.DomainRand
            env.DomainRandEnabled = true;
            env.DomainRandInterval = opts.RandInterval;
            env.DomainRandStrength = opts.RandStrength;
            env.DomainRandBounds = paramBounds;
            fprintf('  DR enabled: strength=%.1f, interval=%d\n', ...
                opts.RandStrength, opts.RandInterval);
        end
        
        % 標準学習（DomainRandは環境のreset()内で自動実行される）
        trainOpts = rlTrainingOptions;
        trainOpts.MaxEpisodes = maxEpisodes;
        trainOpts.MaxStepsPerEpisode = maxSteps;
        trainOpts.SaveAgentDirectory = saveDir;
        trainOpts.SaveAgentCriteria = opts.SaveAgentCriteria;
        trainOpts.StopTrainingCriteria = opts.StopTrainingCriteria;
        trainOpts.StopTrainingValue = opts.StopTrainingValue;
        if isempty(opts.SaveAgentValue)
            switch opts.SaveAgentCriteria
                case 'EpisodeCount'
                    trainOpts.SaveAgentValue = max(1, floor(maxEpisodes / 10));
                otherwise
                    error('SaveAgentValue must be specified for SaveAgentCriteria=%s.', opts.SaveAgentCriteria);
            end
        else
            trainOpts.SaveAgentValue = opts.SaveAgentValue;
        end
        trainOpts.Plots = opts.Plots;
        trainOpts.Verbose = opts.Verbose;
        trainOpts.ScoreAveragingWindowLength = opts.ScoreAveragingWindowLength;
        
        trainingStats = train(agent, env, trainOpts);
        
        % フェーズ結果保存
        save(fullfile(saveDir, 'agent.mat'), 'agent');
        save(fullfile(saveDir, 'training_stats.mat'), 'trainingStats');
        completedEpisodes = completedEpisodes + numel(trainingStats.EpisodeReward);
        
        % 進捗ファイル更新（次回再開位置を記録）
        prog_data = struct('last_completed_phase', phase, 'track', trackName, ...
            'completed_episodes', completedEpisodes, ...
            'timestamp', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
        writeProgressFile(progressFile, prog_data);
        
        fprintf('Phase %d complete. Agent saved to %s\n', phase, saveDir);
        
        % 環境を明示的に切断（C++サーバーは1クライアント専用のため、
        % 次フェーズ接続前に確実に解放する必要がある）
        delete(env);
        clear env;
        pause(0.5);  % サーバー側のaccept()再準備待ち
    end
    
    %% 最終エクスポート
    finalAgentPath = fullfile(saveBaseDir, 'final_agent.mat');
    save(finalAgentPath, 'agent');
    
    % 学習完了時は run-local な出力先へエクスポートする。
    % default runtime path への切り替えは、readiness を通した候補だけを
    % 明示的に publish して行う。
    if isempty(opts.FinalActorExportDir)
        exportDir = fullfile(saveBaseDir, 'final_actor_export');
    else
        exportDir = opts.FinalActorExportDir;
    end
    fprintf('\n📤 学習結果Actorをエクスポートします: %s\n', exportDir);
    try
        exportActorForCpp(agent, exportDir, opts.PublishVehicleParams);
        fprintf('✅ エクスポート完了\n');
    catch ME_exp
        fprintf('⚠️ エクスポート失敗: %s\n', ME_exp.message);
        fprintf('   手動でエクスポートしてください: exportActorForCpp(agent, ''%s'', resolve_rl_vehicle_params())\n', exportDir);
    end

    if opts.PublishFinalToDefault
        fprintf('📌 final_agent を default RL Actor path に公開します\n');
        publishRLDeploymentCandidate(finalAgentPath, ...
            'ActorExportDir', fullfile(fileparts(mfilename('fullpath')), 'rl_actor_export'), ...
            'VehicleParams', opts.PublishVehicleParams, ...
            'Metadata', struct('source_run_dir', saveBaseDir, 'published_from_final_agent', true));
    end
    
    fprintf('\n=== Curriculum Training Complete ===\n');
    fprintf('Final agent: %s\n', finalAgentPath);
end

function trackPools = getDefaultCurriculumTrackPools()
trackPools = {
    {'straight', 'acceleration'}
    {'circle', 'circle_cw', 'circle_R20', 'circle_cw_R20', 'circle_R40', 'circle_cw_R40'}
    {'skidpad', 'skidpad', 'skidpad', 'figure_eight', 'figure_eight', ...
        'circle', 'circle_cw', 'circle_R20', 'circle_cw_R20'}
    {'autocross', 'autocross', 'autocross', 'autocross', 'autocross', ...
        'skidpad', 'skidpad', 'figure_eight', ...
        'circle', 'circle_cw', 'circle_R20', 'circle_cw_R20', 'straight_corner'}
    };
end

function rewardOverrides = getDefaultCurriculumRewardOverrides(phaseSteps)
rewardOverrides = {
    struct('W_speed', 0.5, 'W_boundary', 30, 'W_centering', 1.5, 'W_heading', 1.0)
    struct('W_boundary', 35, 'W_centering', 2.5, 'W_heading', 2.0)
    struct('W_boundary', 40, 'W_centering', 3.0, 'W_heading', 2.5)
    struct('W_boundary', 45, 'W_centering', 3.5, 'W_heading', 3.0, 'W_alive', 0.1)
    };
end

function phaseTrackPools = resolvePhaseTrackPools(defaultTrackPools, overrideTrackPools)
phaseTrackPools = defaultTrackPools;
if isempty(overrideTrackPools)
    return;
end
if ~iscell(overrideTrackPools) || numel(overrideTrackPools) ~= numel(defaultTrackPools)
    error('PhaseTrackPools must be a %dx1 or 1x%d cell array.', numel(defaultTrackPools), numel(defaultTrackPools));
end
for i = 1:numel(defaultTrackPools)
    if isempty(overrideTrackPools{i})
        continue;
    end
    phaseTrackPools{i} = normalizeTrackPool(overrideTrackPools{i}, i);
end
end

function phaseRewardOverrides = resolvePhaseRewardOverrides(defaultRewardOverrides, overrideRewardOverrides)
phaseRewardOverrides = defaultRewardOverrides;
if isempty(overrideRewardOverrides)
    return;
end
if ~iscell(overrideRewardOverrides) || numel(overrideRewardOverrides) ~= numel(defaultRewardOverrides)
    error('PhaseRewardOverrides must be a %dx1 or 1x%d cell array.', numel(defaultRewardOverrides), numel(defaultRewardOverrides));
end
for i = 1:numel(defaultRewardOverrides)
    if isempty(overrideRewardOverrides{i})
        continue;
    end
    if ~isstruct(overrideRewardOverrides{i})
        error('PhaseRewardOverrides{%d} must be a struct.', i);
    end
    phaseRewardOverrides{i} = merge_nested_structs(defaultRewardOverrides{i}, overrideRewardOverrides{i});
end
end

function trackPool = normalizeTrackPool(value, phaseIndex)
if ischar(value) || (isstring(value) && isscalar(value))
    trackPool = {char(value)};
elseif isstring(value)
    trackPool = cellstr(value(:)');
elseif iscell(value)
    trackPool = cell(1, numel(value));
    for i = 1:numel(value)
        if ~(ischar(value{i}) || (isstring(value{i}) && isscalar(value{i})))
            error('PhaseTrackPools{%d} entries must be char or string scalars.', phaseIndex);
        end
        trackPool{i} = char(value{i});
    end
else
    error('PhaseTrackPools{%d} must be a track name or cell array of track names.', phaseIndex);
end

if isempty(trackPool)
    error('PhaseTrackPools{%d} must not be empty.', phaseIndex);
end
end

function prog = readProgressFile(progressFile)
prog = jsondecode(fileread(progressFile));
requiredFields = {'last_completed_phase', 'track'};
for i = 1:numel(requiredFields)
    if ~isfield(prog, requiredFields{i})
        error('Progress file is missing required field "%s": %s', requiredFields{i}, progressFile);
    end
end
end

function writeProgressFile(progressFile, progData)
progressDir = fileparts(progressFile);
if ~isempty(progressDir) && ~exist(progressDir, 'dir')
    mkdir(progressDir);
end

tmpPath = [progressFile '.tmp'];
fid = fopen(tmpPath, 'w');
if fid == -1
    error('Failed to open progress temp file for writing: %s', tmpPath);
end

cleanup = onCleanup(@() closeProgressFile(fid));
fprintf(fid, '%s', jsonencode(progData));
clear cleanup;

[ok, msg, msgid] = movefile(tmpPath, progressFile, 'f');
if ~ok
    if isfile(tmpPath)
        delete(tmpPath);
    end
    error('Failed to update progress file %s (%s: %s).', progressFile, msgid, msg);
end
end

function closeProgressFile(fid)
if fid ~= -1
    fclose(fid);
end
end
