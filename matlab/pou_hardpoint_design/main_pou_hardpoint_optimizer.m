function results = main_pou_hardpoint_optimizer(params_json_file, options)
% MAIN_POU_HARDPOINT_OPTIMIZER - Inverse kinematics solver for POU system
%
% Optimizes M-pivot and bellcrank hardpoint coordinates to match polynomial
% POU coefficients from vehicle optimization.
%
% INPUTS:
%   params_json_file - Path to optimized vehicle parameters JSON
%   options          - (optional) Struct with fields:
%     .wheel_index      - 0=FL, 1=FR, 2=RL, 3=RR (default: 0)
%     .n_multistart     - Number of random initial guesses (default: 5)
%     .delta_range_deg  - Steering angle range [deg] (default: [-30, 30])
%     .n_delta_points   - Number of points in steering sweep (default: 61)
%     .verbose          - Print progress (default: true)
%     .allow_partial_solution - Accept non-converged fmincon output (default: false)
%
% OUTPUT:
%   results - Struct containing:
%     .M_pivot_chassis      - Optimal M-pivot coordinates (chassis frame) [m]
%     .bellcrank_pivot      - Optimal bellcrank pivot coordinates [m]
%     .polynomial_error_mm  - RMS fit error [mm]
%     .max_rod_angle_deg    - Maximum rod-end angle [deg]
%     .installation_ratio   - Mechanical advantage ratio [-]
%     .packaging_valid      - Boolean packaging check result
%     .violations           - Cell array of packaging warnings
%     .optimization_exitflag - fmincon exit status
%     .optimization_time_s   - Solve time [s]

    % === PARSE INPUTS ===
    if nargin < 2
        options = struct();
    end

    % Default options
    if ~isfield(options, 'wheel_index')
        options.wheel_index = 0;
    end
    if ~isfield(options, 'n_multistart')
        options.n_multistart = 5;
    end
    if ~isfield(options, 'delta_range_deg')
        options.delta_range_deg = [-30, 30];
    end
    if ~isfield(options, 'n_delta_points')
        options.n_delta_points = 61;
    end
    if ~isfield(options, 'verbose')
        options.verbose = true;
    end
    if ~isfield(options, 'allow_partial_solution')
        options.allow_partial_solution = false;
    end
    if ~isfield(options, 'allow_missing_tie_rod')
        options.allow_missing_tie_rod = false;
    end

    helper_dir = fileparts(fileparts(mfilename('fullpath')));
    if exist('load_vehicle_artifact', 'file') ~= 2
        addpath(helper_dir);
    end

    % === LOAD VEHICLE PARAMETERS ===
    if options.verbose
        fprintf('=== POU Hardpoint Inverse Kinematics Optimizer ===\n');
        fprintf('Loading vehicle parameters from: %s\n', params_json_file);
    end

    json_data = load_vehicle_artifact(params_json_file);

    % Navigate JSON structure (handle both flat and nested formats)
    params = json_data.params;
    if ~isfield(params, 'suspension')
        error('JSON file does not contain suspension parameters');
    end
    susp = params.suspension;

    % Extract POU polynomial coefficients (wheel_index)
    idx = options.wheel_index + 1;  % MATLAB 1-based indexing

    if isfield(susp, 'pou_jack_c1')
        target_poly.jack_c1 = susp.pou_jack_c1(idx);
        target_poly.jack_c2 = susp.pou_jack_c2(idx);
        target_poly.camber_c1 = susp.pou_camber_c1(idx);
        target_poly.camber_c2 = susp.pou_camber_c2(idx);
    else
        error('POU:MissingPolynomialTargets', ...
            ['Vehicle artifact does not contain pou_jack_c1/pou_jack_c2/pou_camber_c1/pou_camber_c2. ', ...
             'Run the upstream vehicle/suspension artifact generation before POU optimization.']);
    end

    if options.verbose
        wheel_names = {'FL', 'FR', 'RL', 'RR'};
        fprintf('Wheel: %s\n', wheel_names{idx});
        fprintf('Target Polynomial Coefficients:\n');
        fprintf('  Jack-up:  c1=%.4f m/rad, c2=%.4f m/rad²\n', ...
            target_poly.jack_c1, target_poly.jack_c2);
        fprintf('  Camber:   c1=%.4f rad/rad, c2=%.4f rad/rad²\n', ...
            target_poly.camber_c1, target_poly.camber_c2);
    end

    % === DEFINE SUSPENSION GEOMETRY ===
    geom = define_suspension_geometry(options.wheel_index, json_data, options.allow_missing_tie_rod);

    % Steering angle range
    delta_range = linspace(deg2rad(options.delta_range_deg(1)), ...
                           deg2rad(options.delta_range_deg(2)), ...
                           options.n_delta_points)';

    % === OPTIMIZATION SETUP ===
    % Design variables: [Xm, Ym, Zm, Xb, Yb, Zb]
    lb = [geom.M_pivot_bounds.lower; geom.bellcrank_bounds.lower];
    ub = [geom.M_pivot_bounds.upper; geom.bellcrank_bounds.upper];

    % Debug: Check bounds validity
    if any(lb >= ub)
        fprintf('WARNING: Invalid bounds detected!\n');
        for i = 1:length(lb)
            if lb(i) >= ub(i)
                var_names = {'Xm', 'Ym', 'Zm', 'Xb', 'Yb', 'Zb'};
                fprintf('  %s: lb=%.4f >= ub=%.4f\n', var_names{i}, lb(i), ub(i));
            end
        end
        error('Lower bounds must be strictly less than upper bounds');
    end

    % Objective function
    obj_fun = @(x) objective_polynomial_fit(x, geom, target_poly, delta_range);

    % Constraint function
    con_fun = @(x) constraint_rod_misalignment(x, geom, delta_range);

    % Solver options
    if options.verbose
        display_mode = 'iter';
    else
        display_mode = 'none';
    end

    fmincon_options = optimoptions('fmincon', ...
        'Algorithm', 'sqp', ...
        'Display', display_mode, ...
        'MaxIterations', 15000, ...
        'MaxFunctionEvaluations', 300000, ...
        'OptimalityTolerance', 1e-6, ...
        'ConstraintTolerance', 2e-3, ...  % Relaxed from 1e-4
        'StepTolerance', 1e-15, ...       % Allow tiny steps
        'FiniteDifferenceType', 'central', ...
        'UseParallel', false, ...
        'ScaleProblem', true);

    % === MULTI-START OPTIMIZATION ===
    if isfield(options, 'override_x') && ~isempty(options.override_x)
        if options.verbose
            fprintf('\n--- Mirror Override Mode (Skipping Optimization) ---\n');
        end
        best_x = options.override_x;
        best_cost = obj_fun(best_x);
        [c_override, ~] = con_fun(best_x);
        if all(c_override <= fmincon_options.ConstraintTolerance)
            best_exitflag = 1;
        else
            best_exitflag = -1;
            fprintf('WARNING: Mirror override violates %d constraint(s); result marked non-converged.\n', ...
                sum(c_override > fmincon_options.ConstraintTolerance));
        end
        solve_time = 0;
    else
        if options.verbose
            fprintf('\nRunning optimization with %d initial guesses...\n', options.n_multistart);
        end

        best_cost = inf;
        best_x = [];
        best_exitflag = -1;

        tic;
        % Define initial guess strategies
        strategies = {'aggressive', 'nominal', 'conservative', 'random', 'random'};

        for i = 1:options.n_multistart
            % Generate initial guess using different strategies
            if i <= length(strategies)
                strategy = strategies{i};
            else
                strategy = 'random';
            end

            x0 = generate_initial_guess(geom, target_poly, strategy);

            if options.verbose
                fprintf('\n--- Initial Guess %d/%d (%s strategy) ---\n', i, options.n_multistart, strategy);
            end

            % Run optimization
            [x_opt, cost_opt, exitflag] = fmincon(obj_fun, x0, [], [], [], [], ...
                lb, ub, con_fun, fmincon_options);

            % Update best solution
            if ~isnan(cost_opt) && ~isinf(cost_opt) && isscalar(cost_opt) && isscalar(exitflag)
                if exitflag >= 1 && cost_opt < best_cost
                    best_cost = cost_opt;
                    best_x = x_opt;
                    best_exitflag = exitflag;

                    if options.verbose
                        fprintf('  → New best solution found! Cost: %.2f\n', cost_opt);
                    end
                elseif options.allow_partial_solution && exitflag == 0 && (isempty(best_x) || cost_opt < best_cost * 1.5)
                    % Accept non-converged solution if it's the only one or reasonably good
                    if options.verbose
                        fprintf('  → Accepting partial solution (exitflag=%d, cost=%.2f)\n', exitflag, cost_opt);
                    end
                    if isempty(best_x) || cost_opt < best_cost
                        best_cost = cost_opt;
                        best_x = x_opt;
                        best_exitflag = exitflag;
                    end
                end
            end
        end
        solve_time = toc;
    end

    % === RESULTS ===
    if isempty(best_x)
        error('POU:NoConvergedSolution', ...
            'Optimization did not converge. Set options.allow_partial_solution=true only for diagnostics.');
    end

    % Extract optimal design variables
    M_pivot_upright = best_x(1:3);
    bellcrank_pivot = best_x(4:6);

    % Compute M-pivot in chassis frame (at zero steering)
    upright_zero = solve_upright_position_3d(0.0, geom);
    M_pivot_chassis = upright_zero.R_upright * M_pivot_upright + upright_zero.t_upright;

    % Compute final curves and metrics
    [jack_curve, camber_curve, pushrod_lengths, damper_lengths] = ...
        forward_kinematics(best_x, geom, delta_range);

    % Polynomial targets
    jack_target = target_poly.jack_c1 * delta_range + target_poly.jack_c2 * delta_range.^2;
    camber_target = target_poly.camber_c1 * delta_range + target_poly.camber_c2 * delta_range.^2;

    % Errors
    jack_error_mm = (jack_curve - jack_target) * 1000;
    camber_error_deg = rad2deg(camber_curve - camber_target);
    polynomial_error_mm = sqrt(mean(jack_error_mm.^2));

    % Rod-end angle: c(1) = max(rod_angles) - max_allowed, so max(rod_angles) = c(1) + max_allowed
    [c_constraints, ~] = constraint_rod_misalignment(best_x, geom, delta_range);
    max_rod_angle_deg = rad2deg(c_constraints(1) + geom.rod_end_angle_max);

    idx_zero = find(abs(delta_range) < 1e-6, 1);

    % Dense closed-chain LUT for the runtime (#44): travel x steer.
    if isfield(options, 'travel_range_m') && ~isempty(options.travel_range_m)
        travel_range = options.travel_range_m;
    else
        travel_range = 0.01;
    end
    if isfield(options, 'rack_range_m') && ~isempty(options.rack_range_m)
        rack_range = options.rack_range_m;
    else
        rack_range = 0.01;
    end
    if isfield(options, 'n_steer_lut') && ~isempty(options.n_steer_lut)
        n_steer_lut = options.n_steer_lut;
    else
        n_steer_lut = 21;
    end
    travel_lut = linspace(-travel_range, travel_range, 13)';
    rack_lut = linspace(-rack_range, rack_range, 13)';
    raw_lut = solve_pou_closed_chain(geom, best_x, struct( ...
        'travel_m', travel_lut, 'rack_m', rack_lut, 'residual_tolerance', 1e-4));
    max_steer = max(abs(raw_lut.steer_rad(isfinite(raw_lut.steer_rad))));
    if isempty(max_steer) || ~isfinite(max_steer) || max_steer < 1e-6
        max_steer = max(abs(rack_lut)) * 4.0;
    end
    steer_grid = linspace(-max_steer, max_steer, n_steer_lut);
    mr_grid = nan(numel(travel_lut), numel(steer_grid));
    jack_grid = nan(size(mr_grid));
    camber_grid = nan(size(mr_grid));
    damper_grid = nan(size(mr_grid));
    arb_grid = nan(size(mr_grid));
    for it = 1:numel(travel_lut)
        row_steer = raw_lut.steer_rad(it, :);
        if any(~isfinite(row_steer))
            continue;
        end
        if max(row_steer) - min(row_steer) < 1e-9
            mr_grid(it, :) = mean(raw_lut.mr(it, :), 'omitnan');
            jack_grid(it, :) = mean(raw_lut.jack_m(it, :), 'omitnan');
            camber_grid(it, :) = mean(raw_lut.camber_rad(it, :), 'omitnan');
            damper_grid(it, :) = mean(raw_lut.damper_length_m(it, :), 'omitnan');
            continue;
        end
        % Remove duplicate steer samples before interpolation.
        [row_steer, iu] = unique(row_steer);
        row_mr_all = raw_lut.mr(it, :); row_jack_all = raw_lut.jack_m(it, :);
        row_camber_all = raw_lut.camber_rad(it, :); row_damper_all = raw_lut.damper_length_m(it, :);
        row_arb_all = raw_lut.arb_mr(it, :);
        if all(diff(row_steer) <= 0)
            row_steer = fliplr(row_steer);
            row_mr = fliplr(row_mr_all(iu));
            row_jack = fliplr(row_jack_all(iu));
            row_camber = fliplr(row_camber_all(iu));
            row_damper = fliplr(row_damper_all(iu));
            row_arb = fliplr(row_arb_all(iu));
        else
            row_mr = row_mr_all(iu);
            row_jack = row_jack_all(iu);
            row_camber = row_camber_all(iu);
            row_damper = row_damper_all(iu);
            row_arb = row_arb_all(iu);
        end
        mr_grid(it, :) = interp1(row_steer, row_mr, steer_grid, 'linear', 'extrap');
        jack_grid(it, :) = interp1(row_steer, row_jack, steer_grid, 'linear', 'extrap');
        camber_grid(it, :) = interp1(row_steer, row_camber, steer_grid, 'linear', 'extrap');
        damper_grid(it, :) = interp1(row_steer, row_damper, steer_grid, 'linear', 'extrap');
        if any(isfinite(row_arb))
            arb_grid(it, :) = interp1(row_steer, row_arb, steer_grid, 'linear', 'extrap');
        end
    end
    lut = struct();
    lut.schema_version = 'pou_lut/v1';
    lut.travel_m = travel_lut(:)';
    lut.steer_rad = steer_grid;
    lut.mr = mr_grid;
    lut.jack_m = jack_grid;
    lut.camber_rad = camber_grid;
    lut.damper_length_m = damper_grid;
    lut.arb_mr = arb_grid;
    lut.rack_m = rack_lut(:)';
    lut.layout = 'rows=travel_m, cols=steer_rad';
    lut.bellcrank = raw_lut.bellcrank;
    lut.converged = all(isfinite(mr_grid(:))) && all(isfinite(jack_grid(:)));
    results.pou_lut = lut;
    results.bellcrank = raw_lut.bellcrank;

    % Installation ratio at the nominal operating point (average of +/-1 mm
    % because the central-difference MR is exactly zero at the symmetric centre).
    mr_mid = mean(mr_grid, 2, 'omitnan');
    installation_ratio = mean(abs(mr_mid(isfinite(mr_mid))));
    if ~isfinite(installation_ratio)
        valid = mr_grid(isfinite(mr_grid));
        installation_ratio = median(valid);
    end

    % Packaging validation
    [packaging_valid, violations] = validate_packaging(best_x, geom, delta_range);

    % === POPULATE RESULTS STRUCT ===
    results.M_pivot_upright = M_pivot_upright;
    results.M_pivot_chassis = M_pivot_chassis;
    results.bellcrank_pivot = bellcrank_pivot;
    results.polynomial_error_mm = polynomial_error_mm;
    results.max_jack_error_mm = max(abs(jack_error_mm));
    results.max_camber_error_deg = max(abs(camber_error_deg));
    results.max_rod_angle_deg = max_rod_angle_deg;
    results.installation_ratio = installation_ratio;
    results.packaging_valid = packaging_valid;
    results.violations = violations;
    results.optimization_exitflag = best_exitflag;
    results.optimization_time_s = solve_time;
    results.pushrod_length_nominal_m = pushrod_lengths(idx_zero);
    results.damper_length_nominal_m = damper_lengths(idx_zero);

    % Store curves for plotting
    results.delta_range_deg = rad2deg(delta_range);
    results.jack_curve_mm = jack_curve * 1000;
    results.jack_target_mm = jack_target * 1000;
    results.camber_curve_deg = rad2deg(camber_curve);
    results.camber_target_deg = rad2deg(camber_target);

    % === PRINT SUMMARY ===
    if options.verbose
        fprintf('\n========== OPTIMIZATION RESULTS ==========\n');
        fprintf('Exit Status: %d (1=converged, 2=feasible)\n', best_exitflag);
        fprintf('Solve Time: %.2f seconds\n', solve_time);
        fprintf('\n--- Hardpoint Coordinates ---\n');
        fprintf('M-pivot (upright frame): [%.4f, %.4f, %.4f] m\n', M_pivot_upright);
        fprintf('M-pivot (chassis frame): [%.4f, %.4f, %.4f] m\n', M_pivot_chassis);
        fprintf('Bellcrank pivot:         [%.4f, %.4f, %.4f] m\n', bellcrank_pivot);
        fprintf('\n--- Performance Metrics ---\n');
        fprintf('Polynomial Fit Error (RMS): %.2f mm\n', polynomial_error_mm);
        fprintf('Max Jack Error:             %.2f mm\n', max(abs(jack_error_mm)));
        fprintf('Max Camber Error:           %.2f deg\n', max(abs(camber_error_deg)));
        fprintf('Max Rod-End Angle:          %.2f deg (limit: %.0f deg)\n', ...
            max_rod_angle_deg, rad2deg(geom.rod_end_angle_max));
        fprintf('Installation Ratio:         %.3f (target: 0.15-0.25)\n', installation_ratio);
        fprintf('Pushrod Length (nominal):   %.3f m\n', pushrod_lengths(idx_zero));
        fprintf('\n--- Packaging Validation ---\n');
        if packaging_valid
            fprintf('Status: ✓ PASS (all constraints satisfied)\n');
        else
            fprintf('Status: ✗ FAIL (%d violations)\n', length(violations));
        end
        fprintf('==========================================\n\n');
    end
end
