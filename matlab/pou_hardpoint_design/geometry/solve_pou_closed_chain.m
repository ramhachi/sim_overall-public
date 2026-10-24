function lut = solve_pou_closed_chain(geom, design_vars, options)
% SOLVE_POU_CLOSED_CHAIN Rigid pushrod / bellcrank / damper kinematics (#44).
%
% Solves the upright pose (A-arms + tie rod), then the rocker angle that
% keeps the pushrod length constant. The damper length and its derivative
% give the true motion ratio d(damper)/d(wheel travel). No straight-line
% bellcrank proxy or assumed MR=1 is used.
%
% Inputs:
%   geom         - POU geometry from define_suspension_geometry
%   design_vars  - [M_pivot_upright(3); pushrod_pin(3)] in chassis frame
%   options      - .travel_m (Nt x 1, +compression), .rack_m (Nr x 1),
%                  .residual_tolerance, .bellcrank_override
%
% Output lut:
%   travel_m, rack_m, steer_rad (Nt x Nr), mr (Nt x Nr),
%   jack_m, pushrod_length_m, damper_length_m, camber_rad, arb_mr,
%   converged, bellcrank.

    if nargin < 3
        options = struct();
    end
    if ~isfield(options, 'travel_m') || isempty(options.travel_m)
        options.travel_m = linspace(-0.03, 0.03, 13)';
    end
    if ~isfield(options, 'rack_m') || isempty(options.rack_m)
        options.rack_m = linspace(-0.03, 0.03, 13)';
    end
    if ~isfield(options, 'residual_tolerance') || isempty(options.residual_tolerance)
        options.residual_tolerance = 1e-4;
    end

    travel = options.travel_m(:);
    rack = options.rack_m(:);
    nT = numel(travel);
    nR = numel(rack);
    tol = options.residual_tolerance;

    wg = wishbone_geom(geom);
    M_local = design_vars(1:3);
    pushrod_pin = design_vars(4:6);

    use_tie_rod = any(wg.tie_rod_chassis) && any(wg.tie_rod_upright);
    pose0 = solve_wishbone_pose_3d(wg, wg.W_C(3), [], [], tol, [], use_tie_rod);
    if ~pose0.converged
        error('POU:NominalPoseFailed', 'Nominal wishbone pose did not converge.');
    end
    pushrod_dir = pushrod_pin - M_local;
    if isfield(options, 'bellcrank_override') && ~isempty(options.bellcrank_override)
        bc = options.bellcrank_override;
    else
        bc = derive_bellcrank_geometry(pushrod_pin, pushrod_dir, geom);
    end
    Lp0 = norm(M_local - bc.pushrod_pin);
    Ld0 = norm(bc.damper_pin - bc.damper_chassis_mount);
    has_arb = isfield(bc, 'arb_pin') && isfield(bc, 'arb_chassis_mount');
    spindle0 = nominal_spindle(geom);

    steer = nan(nT, nR);
    mr = nan(nT, nR);
    jack = nan(nT, nR);
    lp = nan(nT, nR);
    ld = nan(nT, nR);
    camber = nan(nT, nR);
    arb_mr = nan(nT, nR);
    converged = false(nT, nR);
    theta_guess = 0.0;

    for it = 1:nT
        for ir = 1:nR
            [pose, theta, Lp, Ld, ok, Larb] = evaluate_point(wg, bc, travel(it), ...
                rack(ir), tol, theta_guess, Lp0, has_arb, M_local, use_tie_rod);
            if ~ok
                continue;
            end
            theta_guess = theta;
            steer(it, ir) = pose.steer_angle;
            jack(it, ir) = Ld0 - Ld;  % damper compression (+ = shorter damper)
            lp(it, ir) = Lp;
            ld(it, ir) = Ld;
            spindle = pose.R_upright * spindle0;
            camber(it, ir) = asin(max(-1, min(1, spindle(3))));
            converged(it, ir) = true;

            h = 1e-3;
            [~, ~, ~, Ld_p, ok_p] = evaluate_point(wg, bc, travel(it) + h, ...
                rack(ir), tol, theta, Lp0, has_arb, M_local, use_tie_rod);
            if ok_p
                mr(it, ir) = abs(-(Ld_p - Ld) / h);  % one-sided: nonzero at centre
            end
            if has_arb
                [~, ~, ~, ~, ok_p, Larb_p] = evaluate_point(wg, bc, travel(it) + h, ...
                    rack(ir), tol, theta, Lp0, has_arb, M_local, use_tie_rod);
                if ok_p
                    arb_mr(it, ir) = abs(-(Larb_p - Larb) / h);
                end
            end
        end
    end

    lut = struct();
    lut.travel_m = travel;
    lut.rack_m = rack;
    lut.steer_rad = steer;
    lut.mr = mr;
    lut.jack_m = jack;
    lut.pushrod_length_m = lp;
    lut.damper_length_m = ld;
    lut.camber_rad = camber;
    lut.arb_mr = arb_mr;
    lut.converged = converged;
    lut.bellcrank = bc;
    lut.pushrod_length_nominal_m = Lp0;
    lut.damper_length_nominal_m = Ld0;
    lut.layout = 'rows=travel_m, cols=rack_m';
end

function [pose, theta, Lp, Ld, ok, Larb] = evaluate_point(wg, bc, travel, ...
        rack, tol, theta_guess, Lp0, has_arb, M_local, use_tie_rod)
    wg_rack = wg;
    if use_tie_rod
        wg_rack.tie_rod_chassis = wg.tie_rod_chassis + [0; rack; 0];
        tie_length = norm(wg_rack.tie_rod_chassis - wg_rack.tie_rod_upright);
    else
        tie_length = [];
    end
    target_z = wg.W_C(3) - travel;
    pose = solve_wishbone_pose_3d(wg_rack, target_z, [], [], tol, tie_length, use_tie_rod);
    if ~pose.converged
        ok = false; theta = theta_guess; Lp = NaN; Ld = NaN; Larb = NaN; return;
    end
    M = pose.R_upright * M_local + pose.t_upright;
    [theta, Lp, ok] = solve_pou_rocker_angle(bc, M, Lp0, theta_guess);
    if ~ok
        Ld = NaN; Larb = NaN; return;
    end
    damper = rotate_about_axis(bc.damper_pin, bc.frame_pivot, bc.rotation_axis, theta);
    Ld = norm(damper - bc.damper_chassis_mount);
    if has_arb
        arb = rotate_about_axis(bc.arb_pin, bc.frame_pivot, bc.rotation_axis, theta);
        Larb = norm(arb - bc.arb_chassis_mount);
    else
        Larb = NaN;
    end
end

function p = rotate_about_axis(p0, origin, axis, angle)
    p = origin + axis_angle_rotation(axis, angle) * (p0 - origin);
end

function wg = wishbone_geom(geom)
    wg = struct();
    wg.U_U = geom.upper_arm_upright(:);
    wg.L_U = geom.lower_arm_upright(:);
    wg.W_C = [geom.x_wheel; geom.y_wheel; geom.z_wheel];
    wg.U_F = geom.upper_arm_front_chassis(:);
    wg.U_R = geom.upper_arm_rear_chassis(:);
    wg.L_F = geom.lower_arm_front_chassis(:);
    wg.L_R = geom.lower_arm_rear_chassis(:);
    wg.tie_rod_chassis = geom.tie_rod_chassis(:);
    wg.tie_rod_upright = geom.tie_rod_upright(:);
    wg.upper_arm_axis = geom.upper_arm_axis(:);
    wg.upper_arm_axis_origin = geom.upper_arm_axis_origin(:);
    wg.lower_arm_axis = geom.lower_arm_axis(:);
    wg.lower_arm_axis_origin = geom.lower_arm_axis_origin(:);
    wg.kingpin_axis = geom.kingpin_axis;
    wg.is_left = geom.is_left;
    wg.is_front = geom.is_front;
    wg.Spindle_Dir = nominal_spindle(geom);
end

function s = nominal_spindle(geom)
    if isfield(geom, 'Spindle_Dir') && ~isempty(geom.Spindle_Dir)
        s = geom.Spindle_Dir(:) / max(norm(geom.Spindle_Dir), 1e-9);
    elseif geom.is_left
        s = [0; -1; 0];
    else
        s = [0; 1; 0];
    end
end
