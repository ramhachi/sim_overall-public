function test_optimizer_reproducibility
% TEST_OPTIMIZER_REPRODUCIBILITY - same seed must produce the same design vector.

this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir);
addpath(root_dir);
addpath(fullfile(root_dir, '..', 'matlab'));
addpath(fullfile(root_dir, '..', 'matlab', 'steering_hardpoint_design'));

% The A-arm stage creates the hardpoints, so any export-ready artifact works.
json_file = resolve_default_vehicle_json(fullfile(root_dir, '..'), 'RequireExportReady', true);
if isempty(json_file) || ~isfile(json_file)
    fprintf('SKIP: no export-ready vehicle artifact available.\n');
    return;
end

tmp_files = cell(1, 2);
design_vectors = zeros(2, 10);
try
    for k = 1:2
        tmp_files{k} = [tempname, '.json'];
        copyfile(json_file, tmp_files{k});
        main_a_arm_optimizer(tmp_files{k}, struct('rng_seed', 7, 'n_starts', 3, 'allow_poor_fit', true));
        data = load_vehicle_artifact(tmp_files{k});
        design_vectors(k, :) = data.params.suspension_geometry.front.design_vector;
        assert(data.params.suspension_geometry.front.random_seed == 7);
        assert(data.params.suspension_geometry.front.n_starts == 3);
        assert(ischar(data.params.suspension_geometry.front.code_version));
        diagnostics = data.params.suspension_geometry.front.optimizer_diagnostics;
        assert(isfield(diagnostics, 'evaluation_cache_entries'));
        assert(diagnostics.evaluation_cache_enabled);
        contract = data.params.suspension_geometry.front.target_contract;
        assert(strcmp(contract.weight_policy, 'tolerance_normalized'));
        assert(isfield(contract, 'tolerances'));
        assert(isfield(contract, 'targets'));
        % #39: targets carry source tags; kc.provenance summarises corners.
        sg_targets = data.params.suspension_geometry.front.targets;
        assert(ischar(sg_targets.sources.kpi_deg) && islogical(sg_targets.provisional.kpi_deg));
        assert(strcmp(data.params.suspension.kc.provenance.schema_version, 'kc_provenance/v1'));
        assert(~data.params.suspension.kc.provenance.production_ready);
        if k == 1
            corners = data.params.suspension.kc.corners;
            assert(~isfield(corners.FL.hardpoints, 'tie_rod_inner'));
            assert(strcmp(corners.FL.evaluation_scope.toe, 'not_evaluated'));
            assert(~isfield(corners.FL.kinematics, 'toe_vs_travel'));
        end
    end
    delta = max(abs(design_vectors(1, :) - design_vectors(2, :)));
    assert(delta < 1e-12, sprintf('same-seed design vectors differ by %.3e', delta));

    main_steering_optimizer(tmp_files{1}, struct('rng_seed', 7, 'n_starts', 2, 'allow_poor_fit', true));
    steered = load_vehicle_artifact(tmp_files{1});
    corner = steered.params.suspension.kc.corners.FL;
    assert(isfield(corner.hardpoints, 'tie_rod_inner'));
    assert(isfield(corner.kinematics, 'toe_vs_travel'));
    assert(strcmp(corner.evaluation_scope.tie_rod, 'evaluated'));
    assert(any(strcmp(corner.quality.verdict, {'valid', 'provisional'})));
    steering = steered.params.suspension_geometry.front_steering;
    assert(strcmp(steering.bump_steer_model, 'closed_link_3d'));
    assert(isfield(steering.achieved, 'bump_steer_penalty'));
    assert(isfield(steering.achieved, 'bump_steer_proxy_penalty'));
    assert(isfield(steering, 'feasibility'));
    assert(isfield(steering.feasibility, 'ackermann_error_rad'));
    assert(isfield(steering.feasibility, 'rack_travel_ok'));
    assert(isfield(steering.feasibility, 'tie_rod_length_ok'));
    assert(isfield(steering.feasibility, 'rod_end_angle_ok'));
    assert(numel(steering.bump_steer_travel_samples_m) >= 11);
    % #42: the whole corner was re-solved with the tie rod.
    assert(strcmp(corner.source.tie_rod_model, 'evaluated'));
    assert(isfield(corner, 'sample_values') && isfield(corner.sample_values, 'toe_rad'));
    assert(strcmp(corner.source.script, 'matlab/steering_hardpoint_design/main_steering_optimizer.m'));

    % #42: rear stage adds a toe link and exports toe_vs_travel / roll steer.
    main_rear_a_arm_optimizer(tmp_files{1}, struct('rng_seed', 7, 'n_starts', 2, 'allow_poor_fit', true));
    rear = load_vehicle_artifact(tmp_files{1});
    rl = rear.params.suspension.kc.corners.RL;
    assert(isfield(rl.hardpoints, 'tie_rod_inner'));
    assert(isfield(rl.kinematics, 'toe_vs_travel') && isfield(rl.kinematics, 'roll_steer_gain'));
    assert(strcmp(rl.evaluation_scope.toe, 'evaluated'));
    assert(strcmp(rl.evaluation_scope.steering, 'not_applicable'));
    % Zero-bump-steer construction keeps rear bump steer small (< 0.5 deg over the full range).
    assert(max(abs(rl.sample_values.toe_rad - rl.static_alignment.toe)) < deg2rad(0.5));
    fprintf('PASS: same seed produced identical design vectors and steering K&C was re-evaluated.\n');
catch ME
    for k = 1:numel(tmp_files)
        if ~isempty(tmp_files{k}) && isfile(tmp_files{k})
            delete(tmp_files{k});
        end
    end
    rethrow(ME);
end

for k = 1:numel(tmp_files)
    if isfile(tmp_files{k})
        delete(tmp_files{k});
    end
end
end
