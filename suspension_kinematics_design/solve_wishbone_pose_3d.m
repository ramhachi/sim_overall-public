function pose = solve_wishbone_pose_3d(geom, targetWheelZ, tieRodChassis, ...
        initialGuess, residualTolerance, tieRodLength, useTieRod)
% SOLVE_WISHBONE_POSE_3D Solve one corner pose at a wheel-centre height.
%
% Public shared solver used by evaluate_double_wishbone_3d and the POU
% pushrod/bellcrank closed chain (#44). geom uses the evaluate_* fields
% (U_U, L_U, W_C, tie_rod_*), not the POU hardpoint names.
%
% Inputs:
%   geom             - corner geometry (U_U, L_U, W_C, axes, tie rod)
%   targetWheelZ     - wheel-centre Z in the chassis frame [m, SAE Z down]
%   tieRodChassis    - optional rack-side tie-rod point override [3x1]
%   initialGuess     - optional fsolve seed (3x1 with tie rod, 2x1 without)
%   residualTolerance- convergence threshold (default 1e-6)
%   tieRodLength     - optional fixed tie-rod length (default from geom)
%
% Output: pose with U_U, L_U, R_upright, t_upright, W_C, TR_U, steer_angle.

    if nargin < 3
        tieRodChassis = [];
    end
    if nargin < 4
        initialGuess = [];
    end
    if nargin < 5 || isempty(residualTolerance)
        residualTolerance = 1e-6;
    end
    if nargin < 6
        tieRodLength = [];
    end
    if nargin < 7 || isempty(useTieRod)
        useTieRod = isfield(geom, 'tie_rod_upright') && ~isempty(geom.tie_rod_upright) ...
            && isfield(geom, 'tie_rod_chassis') && ~isempty(geom.tie_rod_chassis);
    end

    if ~isempty(tieRodChassis)
        geom.tie_rod_chassis = tieRodChassis(:);
    end

    U0 = geom.U_U;
    L0 = geom.L_U;
    W0 = geom.W_C;
    L_kingpin = norm(U0 - L0);
    options = optimoptions('fsolve', 'Display', 'off', ...
        'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12);

    if useTieRod
        TR0 = geom.tie_rod_upright;
        if isempty(tieRodLength)
            tieLength = norm(geom.tie_rod_chassis - TR0);
        else
            tieLength = tieRodLength;
        end
        guess = normalize_initial_guess(initialGuess, 3);
        [x, fval, exitflag, output] = fsolve(@(vars) travel_residual_3(vars, geom, ...
            U0, L0, W0, TR0, L_kingpin, tieLength, targetWheelZ), guess, options);
        theta_u = x(1);
        theta_l = x(2);
        phi = x(3);
    else
        guess = normalize_initial_guess(initialGuess, 2);
        [x, fval, exitflag, output] = fsolve(@(vars) travel_residual_2(vars, geom, ...
            U0, L0, W0, L_kingpin, targetWheelZ), guess, options);
        theta_u = x(1);
        theta_l = x(2);
        phi = 0.0;
        TR0 = [];
    end

    [U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, ...
        theta_u, theta_l, phi);
    pose = struct();
    pose.U_U = U_U;
    pose.L_U = L_U;
    pose.R_upright = R_upright;
    pose.t_upright = t_upright;
    pose.W_C = R_upright * W0 + t_upright;
    pose.solver_vars = x(:);
    pose.fval = fval(:);
    pose.raw_residual_norm = norm(fval, inf);
    if numel(fval) == 3
        residual_scale = [max(L_kingpin^2, 1e-6); 1.0; max(tieLength^2, 1e-6)];
    else
        residual_scale = [max(L_kingpin^2, 1e-6); 1.0];
    end
    pose.residual_scale = residual_scale(:);
    pose.residual_norm = max(abs(fval(:) ./ residual_scale));
    pose.iterations = output.iterations;
    pose.exitflag = exitflag;
    pose.converged = exitflag > 0 && all(isfinite(x)) && ...
        pose.residual_norm <= residualTolerance;
    if ~isempty(TR0)
        pose.TR_U = R_upright * TR0 + t_upright;
    else
        pose.TR_U = [];
    end
    pose.steer_angle = compute_steer_from_pose(geom, pose);
end

function guess = normalize_initial_guess(initialGuess, n)
    if isempty(initialGuess) || numel(initialGuess) ~= n || ~all(isfinite(initialGuess(:)))
        guess = zeros(n, 1);
    else
        guess = initialGuess(:);
    end
end

function steerAngle = compute_steer_from_pose(geom, pose)
    spindleNominal = get_spindle_nominal(geom);
    spindleNow = pose.R_upright * spindleNominal;
    v0 = spindleNominal(1:2);
    v = spindleNow(1:2);
    steerAngle = atan2(v0(1) * v(2) - v0(2) * v(1), dot(v0, v));
end

function spindleNominal = get_spindle_nominal(geom)
    if isfield(geom, 'Spindle_Dir') && ~isempty(geom.Spindle_Dir)
        spindleNominal = geom.Spindle_Dir / max(norm(geom.Spindle_Dir), 1e-9);
    elseif geom.is_left
        spindleNominal = [0; -1; 0];
    else
        spindleNominal = [0; 1; 0];
    end
end

function res = travel_residual_2(vars, geom, U0, L0, W0, L_kingpin, targetWheelZ)
    [U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, ...
        vars(1), vars(2), 0.0);
    W_C = R_upright * W0 + t_upright;
    res = [
        dot(U_U - L_U, U_U - L_U) - L_kingpin^2;
        W_C(3) - targetWheelZ
    ];
end

function res = travel_residual_3(vars, geom, U0, L0, W0, TR0, ...
        L_kingpin, tieLength, targetWheelZ)
    [U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, ...
        vars(1), vars(2), vars(3));
    W_C = R_upright * W0 + t_upright;
    TR_U = R_upright * TR0 + t_upright;
    res = [
        dot(U_U - L_U, U_U - L_U) - L_kingpin^2;
        W_C(3) - targetWheelZ;
        dot(geom.tie_rod_chassis - TR_U, geom.tie_rod_chassis - TR_U) - tieLength^2
    ];
end

function [U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, ...
        theta_u, theta_l, phi)
    U_U = rotate_point_around_axis(U0, geom.upper_arm_axis, geom.upper_arm_axis_origin, theta_u);
    L_U = rotate_point_around_axis(L0, geom.lower_arm_axis, geom.lower_arm_axis_origin, theta_l);
    kp_ref = geom.kingpin_axis.direction;
    kp_new = (U_U - L_U) / max(norm(U_U - L_U), 1e-9);
    R_align = rotation_between_vectors(kp_ref, kp_new);
    R_spin = axis_angle_rotation(kp_new, phi);
    R_upright = R_spin * R_align;
    mid_ref = (U0 + L0) / 2;
    mid_cur = (U_U + L_U) / 2;
    t_upright = mid_cur - R_upright * mid_ref;
end

function p_rot = rotate_point_around_axis(p, axis, axis_origin, angle)
    v = p - axis_origin;
    p_rot = axis_origin + axis_angle_rotation(axis, angle) * v;
end

function R = rotation_between_vectors(a, b)
    a = a / max(norm(a), 1e-9);
    b = b / max(norm(b), 1e-9);
    v = cross(a, b);
    s = norm(v);
    c = dot(a, b);
    if s < 1e-12
        if c > 0
            R = eye(3);
        else
            perp = get_perpendicular(a);
            R = axis_angle_rotation(perp, pi);
        end
    else
        vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
        R = eye(3) + vx + vx * vx * (1 - c) / (s * s);
    end
end

function perp = get_perpendicular(v)
    if abs(v(1)) < 0.9
        perp = cross(v, [1; 0; 0]);
    else
        perp = cross(v, [0; 1; 0]);
    end
    perp = perp / max(norm(perp), 1e-9);
end

function R = axis_angle_rotation(axis, angle)
    axis = axis / max(norm(axis), 1e-9);
    x = axis(1); y = axis(2); z = axis(3);
    c = cos(angle); s = sin(angle); C = 1 - c;
    R = [ ...
        x*x*C + c,     x*y*C - z*s, x*z*C + y*s; ...
        y*x*C + z*s,   y*y*C + c,   y*z*C - x*s; ...
        z*x*C - y*s,   z*y*C + x*s, z*z*C + c];
end
