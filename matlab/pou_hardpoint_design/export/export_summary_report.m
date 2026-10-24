function report_path = export_summary_report(results, geom, output_dir)
% EXPORT_SUMMARY_REPORT - Generate human-readable POU optimization report
%
% INPUTS:
%   results    - Results struct from main_pou_hardpoint_optimizer
%   geom       - Suspension geometry struct
%   output_dir - Output directory (default: current directory)
%
% OUTPUT:
%   report_path - Path to generated report file

    if nargin < 3 || isempty(output_dir)
        output_dir = '.';
    end

    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    wheel_names = {'FL', 'FR', 'RL', 'RR'};
    wn = wheel_names{geom.wheel_index + 1};

    report_path = fullfile(output_dir, sprintf('pou_report_%s.txt', wn));

    fid = fopen(report_path, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', report_path);
    end

    fprintf(fid, '================================================================\n');
    fprintf(fid, ' POU Hardpoint Optimization Report\n');
    fprintf(fid, ' Wheel: %s\n', wn);
    fprintf(fid, ' Date: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, '================================================================\n\n');

    % Optimization status
    fprintf(fid, '--- Optimization Status ---\n');
    if results.optimization_exitflag >= 1
        fprintf(fid, 'Status:    CONVERGED (exitflag=%d)\n', results.optimization_exitflag);
    elseif results.optimization_exitflag == 0
        fprintf(fid, 'Status:    PARTIAL (max iterations reached)\n');
    else
        fprintf(fid, 'Status:    FAILED (exitflag=%d)\n', results.optimization_exitflag);
    end
    fprintf(fid, 'Solve Time: %.2f s\n\n', results.optimization_time_s);

    % Hardpoint coordinates
    fprintf(fid, '--- Hardpoint Coordinates (Chassis Frame) ---\n');
    fprintf(fid, '  M-pivot (upright):  [%8.4f, %8.4f, %8.4f] m\n', results.M_pivot_upright);
    fprintf(fid, '  M-pivot (chassis):  [%8.4f, %8.4f, %8.4f] m\n', results.M_pivot_chassis);
    fprintf(fid, '  Bellcrank pivot:    [%8.4f, %8.4f, %8.4f] m\n', results.bellcrank_pivot);
    fprintf(fid, '\n');

    % A-arm hardpoints (full DW model)
    fprintf(fid, '--- A-Arm Hardpoints (Reference) ---\n');
    fprintf(fid, '  Upper front chassis: [%8.4f, %8.4f, %8.4f] m\n', geom.upper_arm_front_chassis);
    fprintf(fid, '  Upper rear chassis:  [%8.4f, %8.4f, %8.4f] m\n', geom.upper_arm_rear_chassis);
    fprintf(fid, '  Upper upright:       [%8.4f, %8.4f, %8.4f] m\n', geom.upper_arm_upright);
    fprintf(fid, '  Lower front chassis: [%8.4f, %8.4f, %8.4f] m\n', geom.lower_arm_front_chassis);
    fprintf(fid, '  Lower rear chassis:  [%8.4f, %8.4f, %8.4f] m\n', geom.lower_arm_rear_chassis);
    fprintf(fid, '  Lower upright:       [%8.4f, %8.4f, %8.4f] m\n', geom.lower_arm_upright);
    fprintf(fid, '  Tie rod chassis:     [%8.4f, %8.4f, %8.4f] m\n', geom.tie_rod_chassis);
    fprintf(fid, '  Tie rod upright:     [%8.4f, %8.4f, %8.4f] m\n', geom.tie_rod_upright);
    fprintf(fid, '  Kingpin origin:      [%8.4f, %8.4f, %8.4f] m\n', geom.kingpin_axis.origin);
    fprintf(fid, '  Kingpin direction:   [%8.4f, %8.4f, %8.4f]\n', geom.kingpin_axis.direction);
    fprintf(fid, '\n');

    % Performance metrics
    fprintf(fid, '--- Performance Metrics ---\n');
    fprintf(fid, '  Polynomial Fit RMS:  %6.2f mm', results.polynomial_error_mm);
    if results.polynomial_error_mm < 2.0
        fprintf(fid, '  [PASS < 2mm]\n');
    else
        fprintf(fid, '  [FAIL > 2mm]\n');
    end

    fprintf(fid, '  Max Jack Error:      %6.2f mm\n', results.max_jack_error_mm);
    fprintf(fid, '  Max Camber Error:    %6.2f deg\n', results.max_camber_error_deg);

    fprintf(fid, '  Max Rod-End Angle:   %6.2f deg', results.max_rod_angle_deg);
    if results.max_rod_angle_deg < rad2deg(geom.rod_end_angle_max)
        fprintf(fid, '  [PASS < %.0f deg]\n', rad2deg(geom.rod_end_angle_max));
    else
        fprintf(fid, '  [FAIL > %.0f deg]\n', rad2deg(geom.rod_end_angle_max));
    end

    fprintf(fid, '  Installation Ratio:  %6.3f', results.installation_ratio);
    if results.installation_ratio >= 0.05 && results.installation_ratio <= 0.50
        fprintf(fid, '     [PASS 0.05-0.50]\n');
    else
        fprintf(fid, '     [OUT OF RANGE]\n');
    end

    fprintf(fid, '  Pushrod Length:      %6.3f m\n', results.pushrod_length_nominal_m);
    fprintf(fid, '\n');

    % Packaging
    fprintf(fid, '--- Packaging Validation ---\n');
    if results.packaging_valid
        fprintf(fid, '  Status: PASS (all constraints satisfied)\n');
    else
        fprintf(fid, '  Status: FAIL\n');
        for i = 1:length(results.violations)
            fprintf(fid, '  Violation %d: %s\n', i, results.violations{i});
        end
    end
    fprintf(fid, '\n');

    % Geometry assumptions
    fprintf(fid, '--- Geometry Assumptions ---\n');
    fprintf(fid, '  Rim radius:          %.3f m (%.1f in)\n', geom.rim_radius, geom.rim_radius/0.0254);
    fprintf(fid, '  Rim clearance min:   %.1f mm\n', geom.rim_clearance_min * 1000);
    fprintf(fid, '  Pushrod length min:  %.3f m\n', geom.pushrod_length_min);
    fprintf(fid, '  Pushrod length max:  %.3f m\n', geom.pushrod_length_max);
    fprintf(fid, '  Rod-end angle max:   %.1f deg\n', rad2deg(geom.rod_end_angle_max));
    fprintf(fid, '\n');

    fprintf(fid, '================================================================\n');
    fprintf(fid, ' NOTE: A-arm hardpoints are PLACEHOLDER values.\n');
    fprintf(fid, ' Update define_suspension_geometry.m with actual CAD data\n');
    fprintf(fid, ' before using these results for manufacturing.\n');
    fprintf(fid, '\n');
    fprintf(fid, ' SCOPE NOTE:\n');
    fprintf(fid, ' This tool uses a reduced kinematic surrogate (camber/jacking proxy\n');
    fprintf(fid, ' polynomials) to compute kinematics and compliance trends. It is\n');
    fprintf(fid, ' a design surrogate for sensitivity/optimization and NOT a full\n');
    fprintf(fid, ' multi-body simulation. Verify physical layout in CAD/ADAMS.\n');
    fprintf(fid, '================================================================\n');

    fclose(fid);
    fprintf('Report exported: %s\n', report_path);
end
