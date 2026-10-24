function all_results = run_all_wheels(params_json_file, options)
% RUN_ALL_WHEELS - Batch POU hardpoint optimization for all steered wheels
%
% Runs the POU inverse kinematics optimizer for front wheels (FL, FR).
% Rear wheels are skipped by default since they have no steering.
%
% INPUTS:
%   params_json_file - Path to optimized vehicle parameters JSON
%   options          - (optional) Struct with fields:
%     .wheels         - Array of wheel indices to optimize (default: [0, 1])
%                       0=FL, 1=FR, 2=RL, 3=RR
%     .n_multistart   - Number of random initial guesses (default: 5)
%     .delta_range_deg - Steering angle range [deg] (default: [-30, 30])
%     .n_delta_points  - Number of points in sweep (default: 61)
%     .output_dir      - Export directory (default: 'output')
%     .export_csv      - Export SolidWorks CSV (default: true)
%     .export_json     - Export JSON (default: true)
%     .export_report   - Export text report (default: true)
%     .update_params   - Update vehicle params JSON (default: false)
%     .max_fit_error_mm - Maximum acceptable jack RMS error [mm] (default: 2.0)
%     .verbose         - Print progress (default: true)
%
% OUTPUT:
%   all_results - Struct with fields FL, FR, etc. containing per-wheel results

    if nargin < 2
        options = struct();
    end

    % Default options
    if ~isfield(options, 'wheels')
        options.wheels = [0, 1];  % Front wheels only (have steering)
    end
    if ~isfield(options, 'n_multistart')
        options.n_multistart = 100;
    end
    if ~isfield(options, 'delta_range_deg')
        options.delta_range_deg = [-30, 30];
    end
    if ~isfield(options, 'n_delta_points')
        options.n_delta_points = 61;
    end
    if ~isfield(options, 'output_dir')
        options.output_dir = 'output';
    end
    if ~isfield(options, 'export_csv')
        options.export_csv = true;
    end
    if ~isfield(options, 'export_json')
        options.export_json = true;
    end
    if ~isfield(options, 'export_report')
        options.export_report = true;
    end
    if ~isfield(options, 'update_params')
        options.update_params = false;
    end
    if ~isfield(options, 'max_fit_error_mm')
        options.max_fit_error_mm = 2.0;
    end
    if ~isfield(options, 'verbose')
        options.verbose = true;
    end
    if ~isfield(options, 'allow_missing_tie_rod')
        options.allow_missing_tie_rod = false;
    end

    % Add paths
    this_dir = fileparts(mfilename('fullpath'));
    addpath(fileparts(this_dir));
    addpath(fullfile(this_dir, 'geometry'));
    addpath(fullfile(this_dir, 'optimization'));
    addpath(fullfile(this_dir, 'visualization'));
    addpath(fullfile(this_dir, 'export'));

    wheel_names = {'FL', 'FR', 'RL', 'RR'};
    all_results = struct();
    total_time = tic;

    if options.verbose
        fprintf('\n');
        fprintf('================================================================\n');
        fprintf(' POU Hardpoint Batch Optimization\n');
        fprintf('================================================================\n');
        fprintf('Params file: %s\n', params_json_file);
        fprintf('Wheels: ');
        for i = 1:length(options.wheels)
            fprintf('%s ', wheel_names{options.wheels(i) + 1});
        end
        fprintf('\n');
        fprintf('Multi-start: %d\n', options.n_multistart);
        fprintf('Output dir: %s\n', options.output_dir);
        fprintf('================================================================\n\n');
    end

    % Create output directory
    if ~exist(options.output_dir, 'dir')
        mkdir(options.output_dir);
    end

    % JSON output path for combined results
    json_output = fullfile(options.output_dir, 'pou_hardpoints.json');

    n_success = 0;
    n_fail = 0;

    for i = 1:length(options.wheels)
        wi = options.wheels(i);
        wn = wheel_names{wi + 1};

        if options.verbose
            fprintf('────────────────────────────────────────\n');
            fprintf(' Wheel %d/%d: %s\n', i, length(options.wheels), wn);
            fprintf('────────────────────────────────────────\n\n');
        end

        % Configure per-wheel options
        wheel_opts = struct();
        wheel_opts.wheel_index = wi;
        wheel_opts.n_multistart = options.n_multistart;
        wheel_opts.delta_range_deg = options.delta_range_deg;
        wheel_opts.n_delta_points = options.n_delta_points;
        wheel_opts.verbose = options.verbose;
        wheel_opts.allow_missing_tie_rod = options.allow_missing_tie_rod;

        % --- ENFORCE LEFT-RIGHT SYMMETRY ---
        if wi == 1 && isfield(all_results, 'FL') && ~isfield(all_results.FL, 'error')
            if options.verbose
                fprintf('  Enforcing symmetry: Mirroring FL geometry for FR.\n');
            end
            fl_res = all_results.FL;
            
            % Generate FR base geometry to extract coordinate transformation
            vehicle_artifact = load_vehicle_artifact(params_json_file);
            fr_geom = define_suspension_geometry(1, vehicle_artifact, options.allow_missing_tie_rod);
            
            % Mirror FL M-pivot to FR (flip Y axis)
            M_pivot_chassis_fr = fl_res.M_pivot_chassis;
            M_pivot_chassis_fr(2) = -M_pivot_chassis_fr(2);
            
            % Mirror FL Bellcrank to FR (flip Y axis)
            bell_chassis_fr = fl_res.bellcrank_pivot;
            bell_chassis_fr(2) = -bell_chassis_fr(2);
            
            upright_zero_fr = solve_upright_position_3d(0.0, fr_geom);
            M_pivot_upright_fr = upright_zero_fr.R_upright' * (M_pivot_chassis_fr - upright_zero_fr.t_upright);
            
            wheel_opts.override_x = [M_pivot_upright_fr; bell_chassis_fr];
        end

        % Run optimization
        try
            results = main_pou_hardpoint_optimizer(params_json_file, wheel_opts);

            % Get geometry for export
            vehicle_artifact = load_vehicle_artifact(params_json_file);
            geom = define_suspension_geometry(wi, vehicle_artifact, options.allow_missing_tie_rod);

            % Store results
            all_results.(wn) = results;
            all_results.(wn).geom = geom;

            % Export
            if options.export_csv
                export_hardpoints_csv(results, geom, options.output_dir);
            end
            if options.export_json
                export_hardpoints_json(results, geom, json_output);
            end
            if options.export_report
                export_summary_report(results, geom, options.output_dir);
            end
            if options.update_params
                update_vehicle_params_json(results, geom, params_json_file);
            end

            if results.optimization_exitflag >= 1 && isfield(results, 'packaging_valid') && results.packaging_valid ...
                    && results.polynomial_error_mm <= options.max_fit_error_mm
                n_success = n_success + 1;
            else
                n_fail = n_fail + 1;
            end

        catch ME
            fprintf('ERROR optimizing wheel %s: %s\n', wn, ME.message);
            all_results.(wn) = struct('error', ME.message);
            n_fail = n_fail + 1;
        end

        fprintf('\n');
    end

    elapsed = toc(total_time);

    % Summary
    if options.verbose
        fprintf('================================================================\n');
        fprintf(' Batch Optimization Complete\n');
        fprintf('================================================================\n');
        fprintf('Total time: %.1f s\n', elapsed);
        fprintf('Success: %d/%d, Failed: %d/%d\n', ...
            n_success, length(options.wheels), n_fail, length(options.wheels));
        fprintf('\n');

        % Per-wheel summary table
        fprintf('%-6s  %-10s  %8s  %8s  %8s  %6s\n', ...
            'Wheel', 'Status', 'RMS[mm]', 'MaxRod', 'IR', 'Pkg');
        fprintf('------  ----------  --------  --------  --------  ------\n');

        for i = 1:length(options.wheels)
            wi = options.wheels(i);
            wn = wheel_names{wi + 1};
            r = all_results.(wn);

            if isfield(r, 'error')
                fprintf('%-6s  %-10s  %8s  %8s  %8s  %6s\n', wn, 'ERROR', '-', '-', '-', '-');
            else
                if r.optimization_exitflag >= 1
                    status = 'CONVERGED';
                else
                    status = 'PARTIAL';
                end
                if r.polynomial_error_mm > options.max_fit_error_mm
                    status = 'FIT-FAIL';
                end
                if r.packaging_valid
                    pkg = 'PASS';
                else
                    pkg = 'FAIL';
                end
                fprintf('%-6s  %-10s  %8.2f  %8.2f  %8.3f  %6s\n', ...
                    wn, status, r.polynomial_error_mm, r.max_rod_angle_deg, ...
                    r.installation_ratio, pkg);
            end
        end

        fprintf('\nOutput files in: %s/\n', options.output_dir);
        fprintf('================================================================\n\n');
    end
end
