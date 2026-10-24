function benchmark_inverse_design
% BENCHMARK_INVERSE_DESIGN Compare cached vs uncached objective evaluations.

this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir);
addpath(root_dir);
addpath(fullfile(root_dir, '..', 'matlab'));

json_file = resolve_default_vehicle_json(fullfile(root_dir, '..'), ...
    'RequireExportReady', true, ...
    'RequireSuspensionHardpoints', true, ...
    'RequiredHardpointCorners', {'FL'});
if isempty(json_file) || ~isfile(json_file)
    fprintf('SKIP: no vehicle artifact with FL hardpoints available.\n');
    return;
end

results = struct('use_cache', {false, true}, 'elapsed_s', {0, 0}, ...
    'cache_entries', {0, 0});
for i = 1:2
    tmp = [tempname, '.json'];
    copyfile(json_file, tmp);
    main_a_arm_optimizer(tmp, struct('n_starts', 2, 'rng_seed', 7, ...
        'use_cache', results(i).use_cache));
    data = load_vehicle_artifact(tmp);
    diagnostics = data.params.suspension_geometry.front.optimizer_diagnostics;
    results(i).elapsed_s = diagnostics.elapsed_s;
    results(i).cache_entries = diagnostics.evaluation_cache_entries;
    delete(tmp);
end
speedup = results(1).elapsed_s / max(results(2).elapsed_s, eps);
fprintf('cache=off: %.2f s (%d entries)\n', results(1).elapsed_s, results(1).cache_entries);
fprintf('cache=on : %.2f s (%d entries)\n', results(2).elapsed_s, results(2).cache_entries);
fprintf('speedup  : %.2fx\n', speedup);
end
