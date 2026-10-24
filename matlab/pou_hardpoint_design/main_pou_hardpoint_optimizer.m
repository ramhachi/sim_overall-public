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
    [jack_curve, camber_curve, pushrod_lengths] = ...
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

    % Installation ratio (at zero steering)
    idx_zero = find(abs(delta_range) < 1e-6, 1);
    OM = M_pivot_chassis - geom.kingpin_axis.origin;
    proj = dot(OM, geom.kingpin_axis.direction) * geom.kingpin_axis.direction;
    perp_dist = norm(OM - proj);
    installation_ratio = perp_dist / pushrod_lengths(idx_zero);

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
