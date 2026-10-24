function upright_state = solve_upright_position_3d(delta, geom)
% SOLVE_UPRIGHT_POSITION_3D - Pure Kingpin Kinematics
%
% Applies pure steering rotation around the derived kingpin axis.
% Assuming fixed heave (A-arms don't rotate), the upright rigidly
% revolves around the axis formed by U_U and L_U.

    U0 = geom.upper_arm_upright;
    L0 = geom.lower_arm_upright;
    TR0 = geom.tie_rod_upright;
    kp_dir = geom.kingpin_axis.direction;
    kp_origin = geom.kingpin_axis.origin;

    % Pure kingpin rotation
    R_kp = axis_angle_rotation(kp_dir, delta);

    % Upright transform
    R_up = R_kp;
    t_up = kp_origin - R_kp * kp_origin;

    % New positions (U_U and L_U technically stay exactly the same)
    U_U = R_up * U0 + t_up;
    L_U = R_up * L0 + t_up;
    TR_U = R_up * TR0 + t_up;

    upright_state.R_upright = R_up;
    upright_state.t_upright = t_up;
    upright_state.U_U = U_U;
    upright_state.L_U = L_U;
    upright_state.TR_U = TR_U;
    upright_state.kingpin_dir = kp_dir; % Kingpin vector is fixed
    upright_state.position = L_U;
    upright_state.rotation = R_up;
    upright_state.upper_arm_upright_chassis = U_U;
    upright_state.lower_arm_upright_chassis = L_U;
    upright_state.steer_angle = delta;
end


% =========================================================================
% LOCAL FUNCTIONS
% =========================================================================

function res = eval_res_3(x, geom, U0, L0, TR0, L_kingpin, L_tie_rod, L_TR_U)
% 3-variable residual:
%   x = [theta_u, theta_l, phi]
%   f1: |U_U - L_U|^2 - L_kingpin^2
%   f2: |TR_C - TR_U|^2 - L_tie_rod^2
%   f3: |TR_U - U_U|^2 - L_TR_U^2

    theta_u = x(1);
    theta_l = x(2);
    phi = x(3);

    U_U = rotate_point_around_axis(U0, geom.upper_arm_axis, ...
        geom.upper_arm_axis_origin, theta_u);
    L_U = rotate_point_around_axis(L0, geom.lower_arm_axis, ...
        geom.lower_arm_axis_origin, theta_l);

    % Upright transform
    kp_dir_ref = geom.kingpin_axis.direction;
    kp_dir_new = (U_U - L_U) / norm(U_U - L_U);
    R_align = rotation_between_vectors(kp_dir_ref, kp_dir_new);
    R_spin = axis_angle_rotation(kp_dir_new, phi);
    R_up = R_spin * R_align;

    mid_ref = (U0 + L0) / 2;
    mid_cur = (U_U + L_U) / 2;
    t_up = mid_cur - R_up * mid_ref;

    TR_U = R_up * TR0 + t_up;

    % f1: kingpin rigidity
    d_UL = U_U - L_U;
    f1 = dot(d_UL, d_UL) - L_kingpin^2;

    % f2: tie-rod length
    d_TR = geom.tie_rod_chassis - TR_U;
    f2 = dot(d_TR, d_TR) - L_tie_rod^2;

    % f3: upright rigidity (TR to upper BJ distance)
    d_TR_U = TR_U - U_U;
    f3 = dot(d_TR_U, d_TR_U) - L_TR_U^2;

    res = [f1; f2; f3];
end


function p_rot = rotate_point_around_axis(p, axis, axis_origin, angle)
    v = p - axis_origin;
    R = axis_angle_rotation(axis, angle);
    p_rot = axis_origin + R * v;
end


function R = rotation_between_vectors(a, b)
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


function theta0 = estimate_arm_angle(p_target, p_nominal, arm_axis, arm_origin)
    v_nom = p_nominal - arm_origin;
    v_tgt = p_target - arm_origin;

    v_nom_perp = v_nom - dot(v_nom, arm_axis) * arm_axis;
    v_tgt_perp = v_tgt - dot(v_tgt, arm_axis) * arm_axis;

    n_nom = norm(v_nom_perp);
    n_tgt = norm(v_tgt_perp);

    if n_nom < 1e-10 || n_tgt < 1e-10
        theta0 = 0;
        return;
    end

    v_nom_perp = v_nom_perp / n_nom;
    v_tgt_perp = v_tgt_perp / n_tgt;

    cos_a = max(-1, min(1, dot(v_nom_perp, v_tgt_perp)));
    cross_v = cross(v_nom_perp, v_tgt_perp);
    sin_sign = sign(dot(cross_v, arm_axis));
    theta0 = sin_sign * acos(cos_a);
end


function perp = get_perpendicular(v)
    if abs(v(1)) < 0.9
        perp = cross(v, [1; 0; 0]);
    else
        perp = cross(v, [0; 1; 0]);
    end
    perp = perp / norm(perp);
end
