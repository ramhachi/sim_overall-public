function resumeAceDriverAfterRestart()
% RESUMEACEDRIVERAFTERRESTART
% 1) keep the current deployment candidate if one already exists
% 2) otherwise publish the current safest fallback deployment candidate
% 3) resume or start the long ace-driver continuation run

projectRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
deploymentAgentPath = fullfile(projectRoot, 'rl_results', 'deployment', 'deployment_agent.mat');
fallbackAgentPath = fullfile(projectRoot, 'rl_results', ...
    'curriculum_torque_frame_fresh_v1', ...
    'phase2o_circle_preview_always_on', ...
    'Agent19.mat');

fprintf('=== Ace-driver continuation bootstrap ===\n');
if isfile(deploymentAgentPath)
    fprintf('Keeping existing deployment candidate: %s\n', deploymentAgentPath);
else
    fprintf('Publishing fallback deployment candidate: %s\n', fallbackAgentPath);
    publishRLDeploymentCandidate(fallbackAgentPath, ...
        'Metadata', struct( ...
            'published_reason', 'safe_fallback_before_ace_driver_continuation', ...
            'published_by', mfilename));
end

fprintf('Starting ace-driver continuation run...\n');
trainAceDriverFromRecovery( ...
    'PhaseEpisodes', [1, 1, 360, 2160], ...
    'ScoreAveragingWindowLength', 40, ...
    'SaveAgentValue', 20);
end
