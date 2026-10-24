function [c, ceq] = constraint_rod_misalignment(design_vars, geom, delta_range)
% CONSTRAINT_ROD_MISALIGNMENT - Nonlinear constraint on pushrod geometry
%
% INPUTS:
%   design_vars - [Xm, Ym, Zm, Xb, Yb, Zb] - 6x1 design variables [m]
%   geom        - Suspension geometry struct
%   delta_range - Nx1 steering angle array [rad]
%
% OUTPUTS:
%   c   - Inequality constraints (c <= 0 for feasibility)
%   ceq - Equality constraints (empty)
%
% CONSTRAINTS:
%   1. Rod-end misalignment angle < max_angle
%   2. Pushrod length within bounds
%   3. Installation ratio within bounds

    M_pivot_upright = design_vars(1:3);
    bellcrank_pivot = design_vars(4:6);

    N = length(delta_range);
    rod_angles = zeros(N, 1);
    pushrod_lengths = zeros(N, 1);

    % Nominal pushrod direction (zero steer)
    upright_nominal = solve_upright_position_3d(0.0, geom);
    M_pivot_nominal = upright_nominal.R_upright * M_pivot_upright + upright_nominal.t_upright;
    pushrod_vec_nominal = bellcrank_pivot - M_pivot_nominal;
    pushrod_dir_nominal = pushrod_vec_nominal / norm(pushrod_vec_nominal);

    for i = 1:N
        delta = delta_range(i);
        upright_state = solve_upright_position_3d(delta, geom);

        % M-pivot in chassis frame
        M_pivot_chassis = upright_state.R_upright * M_pivot_upright + upright_state.t_upright;

        % Pushrod vector and length
        pushrod_vec = bellcrank_pivot - M_pivot_chassis;
        pushrod_lengths(i) = norm(pushrod_vec);
        pushrod_dir = pushrod_vec / pushrod_lengths(i);

        % Rod-end misalignment = angle from nominal direction
        cos_angle = max(-1, min(1, dot(pushrod_dir, pushrod_dir_nominal)));
        rod_angles(i) = acos(cos_angle);
    end

    % === INEQUALITY CONSTRAINTS (c <= 0) ===
    c = zeros(6, 1);

    % 1. Rod-end angle
    c(1) = max(rod_angles) - geom.rod_end_angle_max;

    % 2. Pushrod length bounds
    c(2) = geom.pushrod_length_min - min(pushrod_lengths);
    c(3) = max(pushrod_lengths) - geom.pushrod_length_max;

    % 3. Installation ratio (at zero steer)
    % Use derived kingpin axis from solver
    kp_origin = geom.kingpin_axis.origin;
    kp_dir = geom.kingpin_axis.direction;
    OM = M_pivot_nominal - kp_origin;
    proj = dot(OM, kp_dir) * kp_dir;
    perp_dist = norm(OM - proj);

    idx_zero = find(abs(delta_range) < 1e-6, 1);
    if ~isempty(idx_zero)
        IR = perp_dist / pushrod_lengths(idx_zero);
    else
        % Use nominal pushrod length
        IR = perp_dist / norm(pushrod_vec_nominal);
    end

    c(4) = 0.03 - IR;    
    c(5) = IR - 1.50;    % relaxed from 0.80 to allow convergence for aggressive targets

    % 4. Maximum distance from Wheel Center to upright pushrod node (M-pivot)
    % Use the midpoint of the two ball joints as the "wheel center" proxy for this check.
    % W_C in SAE Z-down: the hub center is roughly at the midpoint of upper/lower ball joints.
    hub_center = (geom.upper_arm_upright + geom.lower_arm_upright) / 2;
    W_C = [hub_center(1); hub_center(2); hub_center(3)];
    dist_to_wc = norm(M_pivot_nominal - W_C);
    if isfield(geom, 'max_upright_radius')
        max_rad = geom.max_upright_radius;
    else
        max_rad = 0.20;
    end
    c(6) = dist_to_wc - max_rad;

    % 5. M-pivot must be inboard of the tire center plane by a minimum clearance
    if isfield(geom, 'min_inboard_clearance')
        inb_clear = geom.min_inboard_clearance;
    else
        inb_clear = 0.05;
    end
    % abs(Y) of M-pivot must be smaller than abs(Y) of wheel minus clearance
    c(7) = abs(M_pivot_nominal(2)) - (abs(geom.y_wheel) - inb_clear);

    % 6. Bellcrank pivot must be sufficiently far from all A-arm chassis attachment points
    % This prevents the bellcrank from being buried inside the A-arm fork structure.
    if isfield(geom, 'min_bellcrank_arm_clearance')
        bc_arm_clear = geom.min_bellcrank_arm_clearance;
    else
        bc_arm_clear = 0.10;  % Default: 10cm minimum clearance
    end
    arm_chassis_pts = [geom.upper_arm_front_chassis, ...
                       geom.upper_arm_rear_chassis, ...
                       geom.lower_arm_front_chassis, ...
                       geom.lower_arm_rear_chassis];  % 3×4
    min_dist_to_arm = min(vecnorm(arm_chassis_pts - bellcrank_pivot, 2, 1));
    c(8) = bc_arm_clear - min_dist_to_arm;  % <= 0 when clearance satisfied

    ceq = [];
end
