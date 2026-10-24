function actorDir = resolve_default_rl_actor_export_dir(projectRoot)
% RESOLVE_DEFAULT_RL_ACTOR_EXPORT_DIR Pick the best available RL actor export.

if nargin < 1 || isempty(projectRoot)
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
end

projectRoot = char(java.io.File(projectRoot).getCanonicalPath());
requiredFiles = {'rl_actor_weights.txt', 'rl_normalization_params.txt', 'rl_actor_test_data.csv'};

preferredCandidates = { ...
    fullfile(projectRoot, 'rl_results', 'curriculum_torque_frame_fresh_v1', ...
        'phase4_ace_extension_from_ace_final', 'final_actor_export') ...
    fullfile(projectRoot, 'rl_results', 'curriculum_torque_frame_fresh_v1', ...
        'phase34_ace_driver_from_phase3_final', 'final_actor_export') ...
    fullfile(projectRoot, 'rl_results', 'curriculum_torque_frame_fresh_v1', ...
        'phase34_recovery_from_phase3_final', 'final_actor_export') ...
    fullfile(projectRoot, 'matlab', 'rl_driver', 'rl_actor_export') ...
    };

actorDir = '';
for i = 1:numel(preferredCandidates)
    candidate = preferredCandidates{i};
    if hasRequiredFiles(candidate, requiredFiles)
        actorDir = candidate;
        return;
    end
end

finalExportCandidates = dir(fullfile(projectRoot, 'rl_results', '**', 'final_actor_export'));
if isempty(finalExportCandidates)
    return;
end

[~, order] = sort([finalExportCandidates.datenum], 'descend');
finalExportCandidates = finalExportCandidates(order);
for i = 1:numel(finalExportCandidates)
    candidate = fullfile(finalExportCandidates(i).folder, finalExportCandidates(i).name);
    if hasRequiredFiles(candidate, requiredFiles)
        actorDir = candidate;
        return;
    end
end
end

function tf = hasRequiredFiles(candidateDir, requiredFiles)
tf = isfolder(candidateDir);
if ~tf
    return;
end

for i = 1:numel(requiredFiles)
    if ~isfile(fullfile(candidateDir, requiredFiles{i}))
        tf = false;
        return;
    end
end
end
