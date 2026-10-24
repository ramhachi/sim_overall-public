function [jack_curve, camber_curve, pushrod_lengths, damper_lengths] = ...
        forward_kinematics(design_vars, geom, delta_range)
% FORWARD_KINEMATICS POU rocker kinematics over a steering sweep.
%
% The pushrod is rigid: the rocker rotates so its length stays constant.
% Jack is the resulting damper compression (positive = shorter damper), not
% a straight-line pushrod-length proxy. Camber comes from the upright pose
% and is diagnostic only: the POU linkage does not change camber (#44).

    M_local = design_vars(1:3);
    pushrod_pin = design_vars(4:6);
    N = numel(delta_range);
    jack_curve = zeros(N, 1);
    camber_curve = zeros(N, 1);
    pushrod_lengths = zeros(N, 1);
    damper_lengths = zeros(N, 1);

    upright0 = solve_upright_position_3d(0.0, geom);
    M0 = upright0.R_upright * M_local + upright0.t_upright;
    bc = derive_bellcrank_geometry(pushrod_pin, pushrod_pin - M0, geom);
    Lp0 = norm(M0 - bc.pushrod_pin);
    Ld0 = norm(bc.damper_pin - bc.damper_chassis_mount);
    if geom.is_left
        spindle_nominal = [0; -1; 0];
    else
        spindle_nominal = [0; 1; 0];
    end
    if isfield(geom, 'Spindle_Dir') && ~isempty(geom.Spindle_Dir)
        spindle_nominal = geom.Spindle_Dir(:) / max(norm(geom.Spindle_Dir), 1e-9);
    end

    theta_guess = 0.0;
    for i = 1:N
        upright = solve_upright_position_3d(delta_range(i), geom);
        M = upright.R_upright * M_local + upright.t_upright;
        [theta, Lp, ok] = solve_pou_rocker_angle(bc, M, Lp0, theta_guess);
        if ~ok
            jack_curve(i) = NaN;
            camber_curve(i) = NaN;
            pushrod_lengths(i) = NaN;
            damper_lengths(i) = NaN;
            continue;
        end
        theta_guess = theta;
        pushrod_lengths(i) = Lp;
        damper = rotate_about_axis(bc.damper_pin, bc.frame_pivot, ...
            bc.rotation_axis, theta);
        Ld = norm(damper - bc.damper_chassis_mount);
        damper_lengths(i) = Ld;
        jack_curve(i) = Ld0 - Ld;  % damper compression
        spindle_now = upright.R_upright * spindle_nominal;
        camber_curve(i) = asin(max(-1, min(1, spindle_now(3))));
    end
end

function p = rotate_about_axis(p0, origin, axis, angle)
    p = origin + axis_angle_rotation(axis, angle) * (p0 - origin);
end
