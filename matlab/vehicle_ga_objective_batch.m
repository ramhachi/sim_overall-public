function costs = vehicle_ga_objective_batch(active_population, context)
% VEHICLE_GA_OBJECTIVE_BATCH Evaluate one GA batch using the process pool.

persistent batch_counter last_run_dir
if isempty(batch_counter) || isempty(last_run_dir) || ~strcmp(last_run_dir, context.run_dir)
    existing_batches = dir(fullfile(context.run_dir, 'evaluations', 'batch_*'));
    batch_counter = numel(existing_batches);
    last_run_dir = context.run_dir;
end
batch_counter = batch_counter + 1;

if isvector(active_population)
    active_population = reshape(active_population, 1, []);
end
population_count = size(active_population, 1);
full_population = repmat(context.active_schema.full_center, population_count, 1);
full_population(:, context.active_schema.indices) = active_population;
damper_f = find(strcmp(context.compatibility.variable_names, 'DamperF'), 1);
damper_r = find(strcmp(context.compatibility.variable_names, 'DamperR'), 1);
full_population(:, damper_r) = full_population(:, damper_f);

batch_id = sprintf('batch_%06d', batch_counter);
batch_dir = fullfile(context.run_dir, 'evaluations', batch_id);
if ~isfolder(batch_dir)
    mkdir(batch_dir);
end

records = cell(population_count, 1);
parfor i = 1:population_count
    candidate_id = sprintf('%s_member_%04d', batch_id, i);
    records{i} = evaluate_vehicle_ga_candidate( ...
        full_population(i, :), context, candidate_id, batch_dir);
end

costs = zeros(population_count, 1);
log_path = fullfile(context.run_dir, 'evaluations.jsonl');
fid = fopen(log_path, 'a');
if fid == -1
    error('vehicle_ga_objective_batch:logOpenFailed', 'Cannot append %s.', log_path);
end
cleanup = onCleanup(@() close_if_open(fid));
for i = 1:population_count
    costs(i) = records{i}.cost;
    fprintf(fid, '%s\n', jsonencode(records{i}));
end
fclose(fid);
clear cleanup;
end

function close_if_open(fid)
try
    fclose(fid);
catch
end
end
