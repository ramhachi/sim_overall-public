function resumePhase34RecoveryAfterRestart()
% RESUMEPHASE34RECOVERYAFTERRESTART
% 1) keep the current deployment candidate if one already exists
% 2) otherwise publish the current safest fallback deployment candidate
% 3) resume or start the long mixed-track phase 3/4 recovery run

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
deploymentAgentPath = fullfile(projectRoot, 'rl_results', 'deployment', 'deployment_agent.mat');
fallbackAgentPath = fullfile(projectRoot, 'rl_results', ...
    'curriculum_torque_frame_fresh_v1', ...
    'phase2o_circle_preview_always_on', ...
    'Agent19.mat');

fprintf('=== Phase 3/4 recovery bootstrap ===\n');
if isfile(deploymentAgentPath)
    fprintf('Keeping existing deployment candidate: %s\n', deploymentAgentPath);
else
    fprintf('Publishing fallback deployment candidate: %s\n', fallbackAgentPath);
    publishRLDeploymentCandidate(fallbackAgentPath, ...
        'Metadata', struct( ...
            'published_reason', 'safe_fallback_before_phase34_recovery', ...
            'published_by', mfilename));
end

fprintf('Starting long mixed-track phase 3/4 recovery run...\n');
trainPhase34Recovery( ...
    'PhaseEpisodes', [1, 1, 480, 1440], ...
    'ScoreAveragingWindowLength', 30, ...
    'SaveAgentValue', 20);
end
