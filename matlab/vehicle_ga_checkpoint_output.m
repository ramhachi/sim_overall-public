function [state, options, optchanged] = vehicle_ga_checkpoint_output(options, state, flag, context)
% VEHICLE_GA_CHECKPOINT_OUTPUT Persist completed GA generations atomically.

optchanged = false;
if ~ismember(flag, {'init', 'iter', 'done'})
    return;
end

checkpoint_dir = fullfile(context.run_dir, 'checkpoint');
if ~isfolder(checkpoint_dir)
    mkdir(checkpoint_dir);
end
checkpoint = struct( ...
    'schema_version', 'vehicle_ga_checkpoint/v1', ...
    'generation', state.Generation, ...
    'population', state.Population, ...
    'scores', state.Score, ...
    'best_history', state.Best, ...
    'rng_state', rng, ...
    'elapsed_time_sec', context.campaign_elapsed_before_sec + ...
        toc(context.segment_started_tic), ...
    'compatibility', context.compatibility, ...
    'saved_at', char(datetime('now', 'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX')));

mat_staging = fullfile(checkpoint_dir, 'latest.mat.writing');
mat_path = fullfile(checkpoint_dir, 'latest.mat');
save(mat_staging, 'checkpoint', '-v7');
movefile(mat_staging, mat_path, 'f');

finite_scores = state.Score(isfinite(state.Score));
best_cost = NaN;
if ~isempty(finite_scores)
    best_cost = min(finite_scores);
end
summary = struct( ...
    'schema_version', checkpoint.schema_version, ...
    'generation', state.Generation, ...
    'population_count', size(state.Population, 1), ...
    'best_cost', best_cost, ...
    'flag', flag, ...
    'saved_at', checkpoint.saved_at);
write_json_atomic(fullfile(checkpoint_dir, 'latest.json'), summary);
end

function write_json_atomic(path, value)
staging = [path, '.writing'];
fid = fopen(staging, 'w');
if fid == -1
    error('vehicle_ga_checkpoint_output:writeFailed', 'Cannot write %s.', staging);
end
cleanup = onCleanup(@() close_if_open(fid));
fwrite(fid, jsonencode(value, 'PrettyPrint', true), 'char');
fclose(fid);
clear cleanup;
movefile(staging, path, 'f');
end

function close_if_open(fid)
try
    fclose(fid);
catch
end
end
