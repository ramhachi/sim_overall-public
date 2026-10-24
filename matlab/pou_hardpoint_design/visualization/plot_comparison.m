function fig = plot_comparison(results)
% PLOT_COMPARISON - Visualize geometric vs polynomial POU curves
%
% INPUT:
%   results - Output struct from main_pou_hardpoint_optimizer()
%
% OUTPUT:
%   fig - Figure handle
%
% PLOTS:
%   1. Jack-up displacement comparison
%   2. Camber angle comparison
%   3. Error distribution histograms
%   4. 3D hardpoint visualization

    % Create figure with subplots
    fig = figure('Position', [100, 100, 1200, 800], 'Name', 'POU Hardpoint Optimization Results');

    % === SUBPLOT 1: Jack-up Displacement ===
    subplot(2, 3, 1);
    hold on; grid on;

    plot(results.delta_range_deg, results.jack_target_mm, 'b--', 'LineWidth', 2, 'DisplayName', 'Polynomial Target');
    plot(results.delta_range_deg, results.jack_curve_mm, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Geometric (Optimized)');

    xlabel('Steering Angle [deg]');
    ylabel('Jack-up Displacement [mm]');
    title('Jack-up: Geometric vs Polynomial');
    legend('Location', 'best');
    xlim([min(results.delta_range_deg), max(results.delta_range_deg)]);

    % === SUBPLOT 2: Camber Angle ===
    subplot(2, 3, 2);
    hold on; grid on;

    plot(results.delta_range_deg, results.camber_target_deg, 'b--', 'LineWidth', 2, 'DisplayName', 'Polynomial Target');
    plot(results.delta_range_deg, results.camber_curve_deg, 'r-', 'LineWidth', 1.5, 'DisplayName', 'Geometric (Optimized)');

    xlabel('Steering Angle [deg]');
    ylabel('Camber Angle [deg]');
    title('Camber: Geometric vs Polynomial');
    legend('Location', 'best');
    xlim([min(results.delta_range_deg), max(results.delta_range_deg)]);

    % === SUBPLOT 3: Jack-up Error ===
    subplot(2, 3, 4);
    jack_error = results.jack_curve_mm - results.jack_target_mm;
    % Shade the error region
    fill([results.delta_range_deg; flipud(results.delta_range_deg)], ...
         [jack_error; zeros(size(jack_error))], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on; grid on;
    plot(results.delta_range_deg, jack_error, 'k-', 'LineWidth', 1.5);
    yline(0, 'b--', 'LineWidth', 1.5);
    xlabel('Steering Angle [deg]');
    ylabel('Jack-up Error [mm]');
    title(sprintf('Jack-up Error (RMS: %.2f mm)', results.polynomial_error_mm));
    xlim([min(results.delta_range_deg), max(results.delta_range_deg)]);
    max_j_err = max(abs(jack_error));
    if max_j_err < 1; max_j_err = 1; end
    ylim([-max_j_err*1.2, max_j_err*1.2]);

    % === SUBPLOT 4: Camber Error ===
    subplot(2, 3, 5);
    camber_error = results.camber_curve_deg - results.camber_target_deg;
    % Shade the error region
    fill([results.delta_range_deg; flipud(results.delta_range_deg)], ...
         [camber_error; zeros(size(camber_error))], 'r', 'FaceAlpha', 0.2, 'EdgeColor', 'none');
    hold on; grid on;
    plot(results.delta_range_deg, camber_error, 'k-', 'LineWidth', 1.5);
    yline(0, 'b--', 'LineWidth', 1.5);
    xlabel('Steering Angle [deg]');
    ylabel('Camber Error [deg]');
    title(sprintf('Camber Error (Max: %.2f deg)', results.max_camber_error_deg));
    xlim([min(results.delta_range_deg), max(results.delta_range_deg)]);
    max_c_err = max(abs(camber_error));
    if max_c_err < 1; max_c_err = 1; end
    ylim([-max_c_err*1.2, max_c_err*1.2]);

    % === SUBPLOT 5: Error Histogram ===
    subplot(2, 3, 3);
    histogram(jack_error, 20, 'FaceColor', 'b', 'FaceAlpha', 0.7);
    hold on;
    xline(0, 'r--', 'LineWidth', 2);
    xlabel('Jack-up Error [mm]');
    ylabel('Frequency');
    title('Error Distribution');
    grid on;

    % === SUBPLOT 6: Summary Table ===
    subplot(2, 3, 6);
    axis off;

    % Create text summary
    if results.packaging_valid
        packaging_str = 'YES';
    else
        packaging_str = 'NO';
    end

    summary_text = {
        '\bf{Optimization Results}';
        '';
        sprintf('M-pivot (chassis): [%.3f, %.3f, %.3f] m', results.M_pivot_chassis);
        sprintf('Bellcrank pivot:   [%.3f, %.3f, %.3f] m', results.bellcrank_pivot);
        '';
        '\bf{Performance Metrics}';
        sprintf('Polynomial Error (RMS): %.2f mm', results.polynomial_error_mm);
        sprintf('Max Jack Error:         %.2f mm', results.max_jack_error_mm);
        sprintf('Max Camber Error:       %.2f deg', results.max_camber_error_deg);
        sprintf('Max Rod-End Angle:      %.2f deg', results.max_rod_angle_deg);
        sprintf('Installation Ratio:     %.3f', results.installation_ratio);
        sprintf('Pushrod Length:         %.3f m', results.pushrod_length_nominal_m);
        '';
        '\bf{Validation}';
        sprintf('Packaging Valid: %s', packaging_str);
        sprintf('Exit Flag:       %d', results.optimization_exitflag);
        sprintf('Solve Time:      %.1f s', results.optimization_time_s);
    };

    % Display text
    text(0.1, 0.9, summary_text, 'VerticalAlignment', 'top', 'FontSize', 9, ...
        'FontName', 'FixedWidth', 'Interpreter', 'tex');

    % Overall title
    sgtitle('POU Hardpoint Optimization - Geometric vs Polynomial Comparison', ...
        'FontSize', 14, 'FontWeight', 'bold');
end
