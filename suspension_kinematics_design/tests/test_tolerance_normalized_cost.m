classdef test_tolerance_normalized_cost < matlab.unittest.TestCase
    methods (Test)
        function toleranceNormalizesResidual(tc)
            targets = make_targets();
            metrics = exact_metrics(targets);
            metrics.roll_center_z = targets.roll_center_z + targets.tolerances.roll_center_z;

            [cost, terms] = evaluate_inverse_cost(metrics, targets, 'front');

            tc.verifyEqual(terms.roll_center_z, 1.0, 'AbsTol', 1e-12);
            tc.verifyEqual(cost, targets.weights.roll_center_z, 'AbsTol', 1e-12);
        end

        function weightChangeFlipsOnlyIntendedRanking(tc)
            targets = make_targets();
            candidate_a = exact_metrics(targets);
            candidate_a.roll_center_z = targets.roll_center_z + targets.tolerances.roll_center_z;
            candidate_b = exact_metrics(targets);
            candidate_b.kpi_deg = targets.kpi_deg + targets.tolerances.kpi_deg;

            targets.weights.roll_center_z = 1.0;
            targets.weights.kpi = 1.0;
            cost_a = evaluate_inverse_cost(candidate_a, targets, 'front');
            cost_b = evaluate_inverse_cost(candidate_b, targets, 'front');
            tc.verifyEqual(cost_a, cost_b, 'AbsTol', 1e-12);

            targets.weights.roll_center_z = 10.0;
            cost_a = evaluate_inverse_cost(candidate_a, targets, 'front');
            cost_b = evaluate_inverse_cost(candidate_b, targets, 'front');
            tc.verifyGreaterThan(cost_a, cost_b);
            tc.verifyEqual(cost_a, 10.0, 'AbsTol', 1e-12);
            tc.verifyEqual(cost_b, 1.0, 'AbsTol', 1e-12);
        end

        function objectiveWeightOverrideIsPersistedInTargets(tc)
            params = create_vehicle_baseline_template();
            params.suspension.objective_weights = struct('camber_gain', 7.0);
            targets = build_inverse_targets(params, 'front');

            tc.verifyEqual(targets.weights.camber_gain, 7.0);
            tc.verifyEqual(targets.weight_policy, 'tolerance_normalized');
        end
    end
end

function targets = make_targets()
params = create_vehicle_baseline_template();
targets = build_inverse_targets(params, 'front');
targets.weights.roll_center_z = 1.0;
targets.weights.kpi = 1.0;
end

function metrics = exact_metrics(targets)
metrics = struct();
metrics.roll_center_z = targets.roll_center_z;
metrics.kpi_deg = targets.kpi_deg;
metrics.caster_deg = targets.caster_deg;
metrics.scrub_radius_m = targets.scrub_radius_m;
metrics.mechanical_trail_m = targets.mechanical_trail_m;
metrics.camber_gain_rad_m = targets.camber_vs_travel(2);
metrics.roll_camber_gain = targets.roll_camber_gain;
metrics.roll_center_height_per_travel = targets.roll_center_height_per_travel;
metrics.roll_center_height_per_roll = targets.roll_center_height_per_roll;
metrics.jacking_force_per_fy = targets.jacking_force_per_fy;
metrics.anti_brake_pct = targets.anti_brake_pct;
metrics.anti_drive_pct = targets.anti_drive_pct;
metrics.compliance = struct( ...
    'camber_per_load', targets.camber_per_load, ...
    'toe_per_load', targets.toe_per_load);
metrics.IC_y = -1.0;
metrics.converged = true;
end
