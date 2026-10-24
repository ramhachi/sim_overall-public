function [valid, violations] = validate_packaging(design_vars, geom, delta_range)
% VALIDATE_PACKAGING - Post-optimization packaging validation
%
% INPUTS:
%   design_vars - [Xm, Ym, Zm, Xb, Yb, Zb] - 6×1 optimized design variables [m]
%   geom        - Suspension geometry struct
%   delta_range - N×1 steering angle array [rad]
%
% OUTPUTS:
%   valid      - Boolean (true if all checks pass)
%   violations - Cell array of warning messages
%
% CHECKS:
%   1. M-pivot clearance to wheel rim (> 5mm)
%   2. M-pivot clearance to brake caliper (> 5mm)
%   3. Bellcrank position (must be inboard of wheel)
%   4. Bellcrank envelope (ground clearance and chassis bounds)

    violations = {};

    % Extract design variables
    M_pivot_upright = design_vars(1:3);
    bellcrank_pivot = design_vars(4:6);

    % === CHECK 1: Rim Clearance ===
    % M-pivot is on the upright. Check radial distance from the wheel spin
    % axis (along Y through the wheel center) against the inner rim radius.
    % For 10" rim: outer radius=127mm, inner barrel clearance radius~100mm
    % (accounts for bead seat, spoke clearance zone)
    inner_rim_clearance_radius = geom.rim_radius;  % Full rim radius
    z_wheel = geom.z_wheel;
    radial_dist = sqrt((M_pivot_upright(1) - geom.x_wheel)^2 ...
                     + (M_pivot_upright(3) - z_wheel)^2);
    min_rim_clearance = inner_rim_clearance_radius - radial_dist;

    if min_rim_clearance < geom.rim_clearance_min
        violations{end+1} = sprintf('Rim clearance violation: %.1f mm (available: %.1f mm, required: %.1f mm)', ...
            min_rim_clearance * 1000, inner_rim_clearance_radius * 1000, geom.rim_clearance_min * 1000);
    end

    % === CHECK 2: Caliper Clearance ===
    % Check M-pivot distance to brake caliper envelope
    min_caliper_clearance = inf;

    for i = 1:length(delta_range)
        delta = delta_range(i);
        upright_state = solve_upright_position_3d(delta, geom);

        M_pivot_chassis = upright_state.R_upright * M_pivot_upright + ...
            upright_state.t_upright;

        % Distance to caliper center
        dist_to_caliper = norm(M_pivot_chassis - geom.caliper.center);
        clearance = dist_to_caliper - geom.caliper.radius;
        min_caliper_clearance = min(min_caliper_clearance, clearance);
    end

    if min_caliper_clearance < geom.rim_clearance_min
        violations{end+1} = sprintf('Caliper clearance violation: %.1f mm (required: %.1f mm)', ...
            min_caliper_clearance * 1000, geom.rim_clearance_min * 1000);
    end

    % === CHECK 3: Bellcrank Position ===
    % Bellcrank must be on chassis side (inboard of wheel)
    % Use geom.y_wheel directly (the authoritative value, already flipped for right side)
    y_wheel_bc = geom.y_wheel;
    y_bellcrank = bellcrank_pivot(2);

    % Check if bellcrank is inboard (sign depends on left/right wheel)
    if geom.is_left
        % Left wheel: y_wheel < 0, bellcrank Y should be less negative (closer to centerline)
        if y_bellcrank < y_wheel_bc
            violations{end+1} = sprintf('Bellcrank outboard of wheel (Y: %.3f m, wheel Y: %.3f m)', y_bellcrank, y_wheel_bc);
        end
    else
        % Right wheel: y_wheel > 0, bellcrank Y should be less positive
        if y_bellcrank > y_wheel_bc
            violations{end+1} = sprintf('Bellcrank outboard of wheel (Y: %.3f m, wheel Y: %.3f m)', y_bellcrank, y_wheel_bc);
        end
    end

    % === CHECK 4: Bellcrank Envelope ===
    % SAE Z convention: Z is DOWN. Ground is Z = 0 and the chassis sits at
    % negative Z. The bellcrank must clear the ground and stay within the
    % chassis bounds used during optimization.
    z_bellcrank = bellcrank_pivot(3);
    ground_clearance_min = 0.05;
    if z_bellcrank > -ground_clearance_min
        violations{end+1} = sprintf('Bellcrank too close to ground (Z: %.3f m, ground at Z=0)', z_bellcrank);
    end
    if isfield(geom, 'bellcrank_bounds')
        if z_bellcrank < geom.bellcrank_bounds.lower(3)
            violations{end+1} = sprintf('Bellcrank above chassis envelope (Z: %.3f m, limit: %.3f m)', ...
                z_bellcrank, geom.bellcrank_bounds.lower(3));
        end
        if z_bellcrank > geom.bellcrank_bounds.upper(3)
            violations{end+1} = sprintf('Bellcrank below lower A-arm (Z: %.3f m, limit: %.3f m)', ...
                z_bellcrank, geom.bellcrank_bounds.upper(3));
        end
    end

    % === OVERALL VALIDATION ===
    valid = isempty(violations);

    % Print violations if any
    if ~valid
        fprintf('\n=== PACKAGING VIOLATIONS ===\n');
        for i = 1:length(violations)
            fprintf('  %d. %s\n', i, violations{i});
        end
        fprintf('============================\n\n');
    end
end
