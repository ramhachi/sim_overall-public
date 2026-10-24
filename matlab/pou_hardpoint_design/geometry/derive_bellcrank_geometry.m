function bc = derive_bellcrank_geometry(pushrod_pin, pushrod_dir, geom)
% DERIVE_BELLCRANK_GEOMETRY Build the physical rocker used by the POU chain.
%
% The optimizer supplies the pushrod pin on the bellcrank and the upright
% pushrod point. This function fixes the remaining rocker hardpoints so the
% closed chain can be solved and exported (#44). Explicit hardpoints in
% geom.pou_bellcrank override the derived defaults.
%
% Inputs:
%   pushrod_pin - 3x1 pushrod pin on the rocker [m, chassis frame]
%   pushrod_dir - 3x1 nominal pushrod direction (from upright to rocker)
%   geom        - define_suspension_geometry struct
%
% Output fields:
%   pushrod_pin, frame_pivot, rotation_axis, damper_pin,
%   damper_chassis_mount, moment_arm_m, bellcrank_angle_rad, damper_length_m.

    pushrod_pin = pushrod_pin(:);
    pushrod_dir = pushrod_dir(:) / max(norm(pushrod_dir), 1e-9);

    if isfield(geom, 'pou_bellcrank') && isstruct(geom.pou_bellcrank)
        src = geom.pou_bellcrank;
        if isfield(src, 'frame_pivot') && isfield(src, 'rotation_axis') ...
                && isfield(src, 'damper_pin') && isfield(src, 'damper_chassis_mount')
            bc.pushrod_pin = pushrod_pin;
            bc.frame_pivot = src.frame_pivot(:);
            bc.rotation_axis = src.rotation_axis(:) / max(norm(src.rotation_axis), 1e-9);
            bc.damper_pin = src.damper_pin(:);
            bc.damper_chassis_mount = src.damper_chassis_mount(:);
            bc.moment_arm_m = norm(bc.pushrod_pin - bc.frame_pivot);
            bc.bellcrank_angle_rad = NaN;
            bc.damper_length_m = norm(bc.damper_pin - bc.damper_chassis_mount);
            return;
        end
    end

    if isfield(geom, 'bellcrank_moment_arm_m')
        moment_arm = geom.bellcrank_moment_arm_m;
    else
        moment_arm = 0.050;
    end
    if isfield(geom, 'bellcrank_angle_rad')
        bellcrank_angle = geom.bellcrank_angle_rad;
    else
        bellcrank_angle = deg2rad(90);
    end
    if isfield(geom, 'damper_length_m')
        damper_length = geom.damper_length_m;
    else
        damper_length = 0.200;
    end

    % Inboard direction normal to the pushrod line of action.
    if abs(pushrod_dir(2)) > 0.99
        orthogonal = cross(pushrod_dir, [0; 0; 1]);
    else
        orthogonal = cross(pushrod_dir, [0; sign(pushrod_dir(2)); 0]);
    end
    orthogonal = orthogonal / max(norm(orthogonal), 1e-9);
    if sign(orthogonal(2)) == sign(pushrod_pin(2))
        orthogonal = -orthogonal;
    end

    bc.pushrod_pin = pushrod_pin;
    bc.frame_pivot = pushrod_pin + moment_arm * orthogonal;
    bc.rotation_axis = cross(pushrod_dir, orthogonal);
    bc.rotation_axis = bc.rotation_axis / max(norm(bc.rotation_axis), 1e-9);
    bc.bellcrank_angle_rad = bellcrank_angle;
    bc.moment_arm_m = moment_arm;

    R_arm = axis_angle_rotation(bc.rotation_axis, bellcrank_angle);
    damper_arm_vec = R_arm * (orthogonal);
    bc.damper_pin = bc.frame_pivot + moment_arm * damper_arm_vec;

    damper_dir = [0; -sign(pushrod_pin(2)) * 0.5; -0.866];
    damper_dir = damper_dir / max(norm(damper_dir), 1e-9);
    bc.damper_chassis_mount = bc.damper_pin + damper_length * damper_dir;
    bc.damper_length_m = damper_length;
end
