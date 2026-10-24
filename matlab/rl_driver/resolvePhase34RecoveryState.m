function state = resolvePhase34RecoveryState(varargin)
% RESOLVEPHASE34RECOVERYSTATE Resolve the safest continuation point for phase 3/4 recovery.

p = inputParser;
addParameter(p, 'ResumeAgentPath', '');
addParameter(p, 'SaveBaseDir', '');
addParameter(p, 'PhaseEpisodes', [1, 1, 240, 480]);
addParameter(p, 'CompletedEpisodesOffset', []);
addParameter(p, 'DefaultStartPhase', 3);
addParameter(p, 'InitialMode', 'fresh_recovery');
addParameter(p, 'InitialMessage', 'Starting a fresh phase 3/4 recovery run.');
parse(p, varargin{:});
opts = p.Results;

if isempty(opts.ResumeAgentPath)
    error('ResumeAgentPath is required.');
end
if isempty(opts.SaveBaseDir)
    error('SaveBaseDir is required.');
end
if numel(opts.PhaseEpisodes) ~= 4
    error('PhaseEpisodes must contain 4 entries.');
end
if ~isnumeric(opts.DefaultStartPhase) || ~isscalar(opts.DefaultStartPhase) || ...
        opts.DefaultStartPhase < 1 || opts.DefaultStartPhase > numel(opts.PhaseEpisodes)
    error('DefaultStartPhase must be a scalar within [1, %d].', numel(opts.PhaseEpisodes));
end

state = struct();
state.resume_agent_path = char(opts.ResumeAgentPath);
state.start_phase = double(opts.DefaultStartPhase);
state.phase_episodes = opts.PhaseEpisodes;
state.completed_episodes_offset = 0;
state.current_phase_completed_episodes = 0;
state.phase_resume_state_path = '';
state.mode = char(opts.InitialMode);
state.message = char(opts.InitialMessage);

progressPath = fullfile(opts.SaveBaseDir, 'progress.json');
if isfile(progressPath)
    progress = jsondecode(fileread(progressPath));
    lastCompletedPhase = getNumericField(progress, 'last_completed_phase', 0);
    completedEpisodes = getNumericField(progress, 'completed_episodes', 0);
    if lastCompletedPhase >= numel(opts.PhaseEpisodes)
        error(['Recovery run at %s is already complete. ' ...
            'Use a new SaveBaseDir if you want to extend phase 4 further.'], opts.SaveBaseDir);
    end
    activePhase = min(lastCompletedPhase + 1, numel(opts.PhaseEpisodes));

    [activePhaseDir, activePhaseName] = findPhaseDir(opts.SaveBaseDir, activePhase);
    resumeState = getPhaseResumeState(activePhaseDir);
    if resumeState.has_state
        [latestCheckpointPath, latestCheckpointEpisode] = findLatestCheckpointAfter(activePhaseDir, resumeState.session_started_datenum);
        totalPhaseEpisodesCompleted = resumeState.phase_episode_offset;
        if ~isempty(latestCheckpointPath)
            totalPhaseEpisodesCompleted = totalPhaseEpisodesCompleted + latestCheckpointEpisode;
        else
            latestCheckpointPath = resumeState.resume_source_checkpoint;
        end
    else
        [latestCheckpointPath, latestCheckpointEpisode] = findLatestCheckpoint(activePhaseDir);
        totalPhaseEpisodesCompleted = latestCheckpointEpisode;
    end
    if ~isempty(latestCheckpointPath)
        remainingEpisodes = opts.PhaseEpisodes(activePhase) - totalPhaseEpisodesCompleted;
        if remainingEpisodes > 0
            phaseEpisodes = opts.PhaseEpisodes;
            phaseEpisodes(activePhase) = remainingEpisodes;

            state.resume_agent_path = latestCheckpointPath;
            state.start_phase = activePhase;
            state.phase_episodes = phaseEpisodes;
            state.completed_episodes_offset = completedEpisodes + totalPhaseEpisodesCompleted;
            state.current_phase_completed_episodes = totalPhaseEpisodesCompleted;
            state.phase_resume_state_path = fullfile(activePhaseDir, 'resume_state.json');
            state.mode = 'resume_partial_phase';
            state.message = sprintf([ ...
                'Resuming interrupted %s from %s (%d episodes already completed in this phase, %d remaining).'], ...
                activePhaseName, latestCheckpointPath, totalPhaseEpisodesCompleted, remainingEpisodes);
            return;
        end
    end

    if lastCompletedPhase >= 1 && lastCompletedPhase < numel(opts.PhaseEpisodes)
        [completedPhaseDir, completedPhaseName] = findPhaseDir(opts.SaveBaseDir, lastCompletedPhase);
        completedAgentPath = fullfile(completedPhaseDir, 'agent.mat');
        if isfile(completedAgentPath)
            state.resume_agent_path = completedAgentPath;
            state.start_phase = lastCompletedPhase + 1;
            state.phase_episodes = opts.PhaseEpisodes;
            state.completed_episodes_offset = completedEpisodes;
            state.mode = 'resume_next_phase';
            state.message = sprintf([ ...
                'Resuming recovery after completed %s using %s.'], ...
                completedPhaseName, completedAgentPath);
            return;
        end
    end
else
    % Some interrupted runs never flushed a root progress.json; recover from
    % existing phase directories/checkpoints instead of restarting from scratch.
    inferredState = inferStateWithoutProgressFile(opts);
    if inferredState.has_state
        state = inferredState.state;
        return;
    end
end

if isempty(opts.CompletedEpisodesOffset)
    state.completed_episodes_offset = inferCompletedEpisodesOffsetFromCheckpoint(state.resume_agent_path);
else
    state.completed_episodes_offset = opts.CompletedEpisodesOffset;
end
state.message = sprintf('%s Source checkpoint: %s', state.message, state.resume_agent_path);
end

function inferred = inferStateWithoutProgressFile(opts)
inferred = struct('has_state', false, 'state', struct());
if isempty(opts.SaveBaseDir) || ~isfolder(opts.SaveBaseDir)
    return;
end

existingPhases = findExistingPhaseIndices(opts.SaveBaseDir);
if isempty(existingPhases)
    return;
end

activePhase = max(existingPhases);
[activePhaseDir, activePhaseName] = findPhaseDir(opts.SaveBaseDir, activePhase);
if isempty(activePhaseDir)
    return;
end

baseCompletedEpisodes = resolveBaseCompletedEpisodesOffset(opts.ResumeAgentPath, opts.CompletedEpisodesOffset);
priorPhaseEpisodes = 0;
for phase = 1:activePhase-1
    [phaseDir, ~] = findPhaseDir(opts.SaveBaseDir, phase);
    priorPhaseEpisodes = priorPhaseEpisodes + getCompletedEpisodesFromPhaseDir(phaseDir);
end

resumeState = getPhaseResumeState(activePhaseDir);
completedInActivePhase = getCompletedEpisodesFromPhaseDir(activePhaseDir);
latestCheckpointPath = '';
latestCheckpointEpisode = 0;
if resumeState.has_state
    [latestCheckpointPath, latestCheckpointEpisode] = findLatestCheckpointAfter(activePhaseDir, resumeState.session_started_datenum);
    completedInActivePhase = max(completedInActivePhase, resumeState.phase_episode_offset);
    if ~isempty(latestCheckpointPath)
        completedInActivePhase = max(completedInActivePhase, resumeState.phase_episode_offset + latestCheckpointEpisode);
    else
        latestCheckpointPath = resumeState.resume_source_checkpoint;
    end
else
    [latestCheckpointPath, latestCheckpointEpisode] = findLatestCheckpoint(activePhaseDir);
    completedInActivePhase = max(completedInActivePhase, latestCheckpointEpisode);
end

phaseEpisodes = opts.PhaseEpisodes;
phaseCompleted = completedInActivePhase >= phaseEpisodes(activePhase) || isfile(fullfile(activePhaseDir, 'agent.mat'));
completedEpisodesOffset = baseCompletedEpisodes + priorPhaseEpisodes;

if ~phaseCompleted && ~isempty(latestCheckpointPath)
    phaseEpisodes(activePhase) = max(0, phaseEpisodes(activePhase) - completedInActivePhase);
    inferred.state = struct( ...
        'resume_agent_path', latestCheckpointPath, ...
        'start_phase', activePhase, ...
        'phase_episodes', phaseEpisodes, ...
        'completed_episodes_offset', completedEpisodesOffset + completedInActivePhase, ...
        'current_phase_completed_episodes', completedInActivePhase, ...
        'phase_resume_state_path', fullfile(activePhaseDir, 'resume_state.json'), ...
        'mode', 'resume_partial_phase', ...
        'message', sprintf(['Resuming interrupted %s from %s without progress.json ' ...
            '(%d episodes already completed in this phase, %d remaining).'], ...
            activePhaseName, latestCheckpointPath, completedInActivePhase, phaseEpisodes(activePhase)));
    inferred.has_state = true;
    return;
end

if phaseCompleted && activePhase < numel(opts.PhaseEpisodes)
    completedAgentPath = fullfile(activePhaseDir, 'agent.mat');
    if ~isfile(completedAgentPath)
        completedAgentPath = latestCheckpointPath;
    end
    if ~isempty(completedAgentPath)
        inferred.state = struct( ...
            'resume_agent_path', completedAgentPath, ...
            'start_phase', activePhase + 1, ...
            'phase_episodes', opts.PhaseEpisodes, ...
            'completed_episodes_offset', completedEpisodesOffset + completedInActivePhase, ...
            'current_phase_completed_episodes', completedInActivePhase, ...
            'phase_resume_state_path', '', ...
            'mode', 'resume_next_phase', ...
            'message', sprintf(['Resuming after completed %s using %s without progress.json.'], ...
                activePhaseName, completedAgentPath));
        inferred.has_state = true;
    end
end
end

function completedEpisodes = resolveBaseCompletedEpisodesOffset(resumeAgentPath, completedEpisodesOffset)
if isempty(completedEpisodesOffset)
    completedEpisodes = inferCompletedEpisodesOffsetFromCheckpoint(resumeAgentPath);
else
    completedEpisodes = completedEpisodesOffset;
end
end

function value = getNumericField(s, fieldName, defaultValue)
value = defaultValue;
if ~isstruct(s) || ~isfield(s, fieldName)
    return;
end
candidate = s.(fieldName);
if isnumeric(candidate) && isscalar(candidate) && isfinite(candidate)
    value = double(candidate);
end
end

function [phaseDir, phaseName] = findPhaseDir(baseDir, phaseIndex)
phaseDir = '';
phaseName = sprintf('phase%d', phaseIndex);
if isempty(baseDir) || ~isfolder(baseDir)
    return;
end

matches = dir(fullfile(baseDir, sprintf('phase%d_*', phaseIndex)));
matches = matches([matches.isdir]);
if isempty(matches)
    return;
end

phaseDir = fullfile(baseDir, matches(1).name);
phaseName = matches(1).name;
end

function phaseIndices = findExistingPhaseIndices(baseDir)
phaseIndices = [];
if isempty(baseDir) || ~isfolder(baseDir)
    return;
end

matches = dir(fullfile(baseDir, 'phase*_*'));
matches = matches([matches.isdir]);
for i = 1:numel(matches)
    tokens = regexp(matches(i).name, '^phase(\d+)_', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end
    phaseIndex = str2double(tokens{1});
    if isfinite(phaseIndex)
        phaseIndices(end + 1) = phaseIndex; %#ok<AGROW>
    end
end
phaseIndices = unique(sort(phaseIndices));
end

function [checkpointPath, checkpointEpisode] = findLatestCheckpoint(phaseDir)
checkpointPath = '';
checkpointEpisode = 0;
if isempty(phaseDir) || ~isfolder(phaseDir)
    return;
end

matches = dir(fullfile(phaseDir, 'Agent*.mat'));
for i = 1:numel(matches)
    tokens = regexp(matches(i).name, '^Agent(\d+)\.mat$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end
    episode = str2double(tokens{1});
    if isfinite(episode) && episode > checkpointEpisode
        checkpointEpisode = episode;
        checkpointPath = fullfile(phaseDir, matches(i).name);
    end
end
end

function [checkpointPath, checkpointEpisode] = findLatestCheckpointAfter(phaseDir, minDatenum)
checkpointPath = '';
checkpointEpisode = 0;
if isempty(phaseDir) || ~isfolder(phaseDir)
    return;
end

matches = dir(fullfile(phaseDir, 'Agent*.mat'));
for i = 1:numel(matches)
    if matches(i).datenum <= minDatenum
        continue;
    end
    tokens = regexp(matches(i).name, '^Agent(\d+)\.mat$', 'tokens', 'once');
    if isempty(tokens)
        continue;
    end
    episode = str2double(tokens{1});
    if isfinite(episode) && episode > checkpointEpisode
        checkpointEpisode = episode;
        checkpointPath = fullfile(phaseDir, matches(i).name);
    end
end
end

function completedEpisodes = inferCompletedEpisodesOffsetFromCheckpoint(agentPath)
completedEpisodes = 0;
if ~(ischar(agentPath) || isstring(agentPath))
    return;
end

[phaseIndex, phaseDir, runDir] = getPhaseContext(char(agentPath));
if isempty(phaseIndex)
    return;
end

progressPath = fullfile(runDir, 'progress.json');
if isfile(progressPath)
    progress = jsondecode(fileread(progressPath));
    totalCompleted = getNumericField(progress, 'completed_episodes', NaN);
    if isfinite(totalCompleted)
        laterEpisodes = 0;
        for laterPhase = phaseIndex + 1:4
            [laterPhaseDir, ~] = findPhaseDir(runDir, laterPhase);
            laterEpisodes = laterEpisodes + getCompletedEpisodesFromPhaseDir(laterPhaseDir);
        end
        completedEpisodes = max(0, totalCompleted - laterEpisodes);
        if completedEpisodes > 0
            return;
        end
    end
end

completedEpisodes = getCompletedEpisodesFromPhaseDir(phaseDir);
end

function completedEpisodes = getCompletedEpisodesFromPhaseDir(phaseDir)
completedEpisodes = 0;
if isempty(phaseDir) || ~isfolder(phaseDir)
    return;
end

statsPath = fullfile(phaseDir, 'training_stats.mat');
if isfile(statsPath)
    data = load(statsPath);
    candidateNames = fieldnames(data);
    for i = 1:numel(candidateNames)
        candidate = data.(candidateNames{i});
        if isstruct(candidate) && isfield(candidate, 'EpisodeReward')
            completedEpisodes = numel(candidate.EpisodeReward);
            if completedEpisodes > 0
                return;
            end
        end
    end
end

[~, completedEpisodes] = findLatestCheckpoint(phaseDir);
end

function resumeState = getPhaseResumeState(phaseDir)
resumeState = struct( ...
    'has_state', false, ...
    'phase_episode_offset', 0, ...
    'session_started_datenum', -inf, ...
    'resume_source_checkpoint', '');
if isempty(phaseDir) || ~isfolder(phaseDir)
    return;
end

statePath = fullfile(phaseDir, 'resume_state.json');
if ~isfile(statePath)
    return;
end

state = jsondecode(fileread(statePath));
resumeState.has_state = true;
resumeState.phase_episode_offset = getNumericField(state, 'phase_episode_offset', 0);
resumeState.session_started_datenum = getNumericField(state, 'session_started_datenum', -inf);
if isfield(state, 'resume_source_checkpoint') && ...
        (ischar(state.resume_source_checkpoint) || (isstring(state.resume_source_checkpoint) && isscalar(state.resume_source_checkpoint)))
    resumeState.resume_source_checkpoint = char(state.resume_source_checkpoint);
end
end

function [phaseIndex, phaseDir, runDir] = getPhaseContext(agentPath)
phaseIndex = [];
phaseDir = fileparts(agentPath);
runDir = fileparts(phaseDir);
if isempty(phaseDir)
    return;
end

[~, phaseName] = fileparts(phaseDir);
tokens = regexp(phaseName, '^phase(\d+)_', 'tokens', 'once');
if isempty(tokens)
    return;
end

phaseIndex = str2double(tokens{1});
if ~isfinite(phaseIndex)
    phaseIndex = [];
end
end
