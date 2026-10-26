classdef test_vehicle_ga_platform < matlab.unittest.TestCase
    % Contract tests for the EasyStart LPV full-vehicle GA platform.

    properties (TestParameter)
        InvalidMetric = {'max_lateral_error', 'rms_heading_error', ...
            'mean_abs_sw_error', 'steering_saturation_ratio', ...
            'max_track_limit_excess', 'mpc_avg_solve_time_ms', ...
            'mpc_max_solve_time_ms'}
    end

    methods (Test)
        function defaultCampaignIsFiveHourLpvRun(testCase)
            config = vehicle_ga_default_config(test_vehicle_ga_platform.projectRoot());

            testCase.verifyEqual(config.wall_time_sec, 18000);
            testCase.verifyEqual(config.ga_max_time_sec, 17700);
            testCase.verifyEqual(config.population_size, 80);
            testCase.verifyEqual(config.max_generations, 55);
            testCase.verifyEqual(config.parallel_workers, 10);
            testCase.verifyEqual(config.driver_mode, 'lpv_lqr');
            testCase.verifyEqual(config.lpv_model, 'steering6');
            testCase.verifyFalse(config.drs_enabled);
            testCase.verifySubstring(config.output_root, ...
                fullfile('output', 'vehicle_ga_runs'));
        end

        function activeSchemaRemovesFixedAndDegenerateVariables(testCase)
            schema = build_full_vehicle_design_schema();
            active = vehicle_ga_active_schema(schema);

            testCase.verifyNumElements(active.indices, 29);
            testCase.verifyTrue(ismember('DamperF', active.variable_names));
            testCase.verifyFalse(ismember('DamperR', active.variable_names));
            % K&C LUTs own camber; camber-gain variables would be inert.
            testCase.verifyFalse(any(ismember({'CamberGainF', 'CamberGainR', ...
                'POU_Camber_C1_F', 'POU_Camber_C2_F'}, active.variable_names)));
            % Free-lunch variables: the optimizer would just pin them to a bound.
            testCase.verifyFalse(any(ismember({'Mass', 'EngineTorqueScale', 'SteeringStiffness', ...
                'POU_Jack_C1_F', 'POU_Jack_C2_F'}, active.variable_names)));
            testCase.verifyTrue(all(ismember(test_vehicle_ga_platform.fixedTireNames(), ...
                active.fixed_variable_names)));
        end

        function legacySeedsAreImportedByName(testCase)
            root = test_vehicle_ga_platform.projectRoot();
            config = vehicle_ga_default_config(root);
            schema = build_full_vehicle_design_schema();
            [seeds, report] = collect_vehicle_ga_seeds(config.seed_paths, schema);
            statuses = string({report.status});

            testCase.verifySize(seeds, [5, 46]);
            testCase.verifyGreaterThanOrEqual(nnz(statuses == "accepted_exact"), 2);
            testCase.verifyGreaterThanOrEqual(nnz(statuses == "accepted_migrated"), 3);
            testCase.verifyGreaterThanOrEqual(min(seeds, [], 'all'), ...
                min(schema.variable_bounds(:, 1)));
            testCase.verifyTrue(all(seeds >= schema.variable_bounds(:, 1)' & ...
                seeds <= schema.variable_bounds(:, 2)', 'all'));
        end

        function validResultPassesGate(testCase)
            result = test_vehicle_ga_platform.validResult();
            [cost, gate] = score_vehicle_ga_result(result, 0, 1006.2611818354848);

            testCase.verifyTrue(gate.passed);
            testCase.verifyEqual(cost, result.lap_time);
            testCase.verifyEmpty(gate.violations);
        end

        function nonfiniteMetricIsRejected(testCase, InvalidMetric)
            result = test_vehicle_ga_platform.withMetric(InvalidMetric, NaN);
            [cost, gate] = score_vehicle_ga_result(result, 0, 1006.2611818354848);

            testCase.verifyFalse(gate.passed);
            testCase.verifyEqual(cost, 1e9);
        end

        function failedProcessGetsDistancePenalty(testCase)
            result = struct('distance_traveled', 900.0);
            [cost, gate] = score_vehicle_ga_result(result, 124, 1000.0);

            testCase.verifyFalse(gate.passed);
            testCase.verifyEqual(cost, 1.1e6);
            testCase.verifyEqual(gate.violations, {'simulation_failed'});
        end

        function parallelSearchIgnoresOnlyMaxRuntimeOutlier(testCase)
            result = test_vehicle_ga_platform.withMetric('mpc_max_solve_time_ms', 12.0);
            [cost, gate] = score_vehicle_ga_result(result, 0, 1000.0, false);

            testCase.verifyTrue(gate.passed);
            testCase.verifyEqual(cost, result.lap_time);
        end

        function drsConfigurationFailsClosed(testCase)
            config = struct('drs_enabled', true);

            testCase.verifyError(@() run_full_vehicle_ga(config), ...
                'run_full_vehicle_ga:unsupportedDrs');
        end

        function checkpointIsWrittenAtomically(testCase)
            runDir = tempname;
            cleanup = onCleanup(@() test_vehicle_ga_platform.removeTree(runDir));
            mkdir(runDir);
            context = struct( ...
                'run_dir', runDir, ...
                'compatibility', struct('version', 1), ...
                'campaign_elapsed_before_sec', 12.0, ...
                'segment_started_tic', tic);
            state = test_vehicle_ga_platform.checkpointState();
            [~, ~, changed] = vehicle_ga_checkpoint_output(struct(), state, 'iter', context);

            testCase.verifyFalse(changed);
            testCase.verifyTrue(isfile(fullfile(runDir, 'checkpoint', 'latest.mat')));
            testCase.verifyTrue(isfile(fullfile(runDir, 'checkpoint', 'latest.json')));
            testCase.verifyFalse(isfile(fullfile(runDir, 'checkpoint', 'latest.mat.writing')));
            checkpoint = load(fullfile(runDir, 'checkpoint', 'latest.mat'), 'checkpoint');
            testCase.verifyGreaterThanOrEqual(checkpoint.checkpoint.elapsed_time_sec, 12.0);
            clear cleanup;
        end
    end

    methods (Static, Access = private)
        function root = projectRoot()
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
        end

        function names = fixedTireNames()
            names = {'TireFzNom', 'TireRelaxationLx', 'TireRelaxationLy', ...
                'TireCx', 'TireCy', 'TireVerticalK', 'TireVerticalC'};
        end

        function result = validResult()
            result = struct( ...
                'status', 'success', ...
                'lap_time', 72.0, ...
                'lap_completed', true, ...
                'degraded_mode', false, ...
                'tire_model_type', 'pinn', ...
                'tire_runtime', 'pinn_onnx', ...
                'velocity_profile_source', 'generated', ...
                'max_track_limit_excess', 0.0, ...
                'max_lateral_error', 0.3, ...
                'rms_heading_error', 2.0, ...
                'mean_abs_sw_error', 0.05, ...
                'steering_saturation_ratio', 0.0, ...
                'mpc_fail_count', 0, ...
                'mpc_avg_solve_time_ms', 0.01, ...
                'mpc_max_solve_time_ms', 0.2);
        end

        function result = withMetric(name, value)
            result = test_vehicle_ga_platform.validResult();
            result.(name) = value;
        end

        function state = checkpointState()
            state = struct( ...
                'Generation', 2, ...
                'Population', [1, 2; 3, 4], ...
                'Score', [4; 2], ...
                'Best', [5; 2]);
        end

        function removeTree(path)
            if isfolder(path)
                rmdir(path, 's');
            end
        end
    end
end
