function geom = geom_from_hardpoints(hp, is_left, is_front, static_camber, R_tire, R_loaded)
% GEOM_FROM_HARDPOINTS Build an evaluate_double_wishbone_3d corner from a
% kc.corners.<X>.hardpoints struct. Tie rod / toe link and pushrod are added
% only when present and non-zero.
if nargin < 6 || isempty(R_loaded)
    R_loaded = R_tire;
end
geom = struct();
geom.U_F = hp.upper_arm_chassis_fore(:);
geom.U_R = hp.upper_arm_chassis_aft(:);
geom.U_U = hp.upper_ball_joint(:);
geom.L_F = hp.lower_arm_chassis_fore(:);
geom.L_R = hp.lower_arm_chassis_aft(:);
geom.L_U = hp.lower_ball_joint(:);
geom.W_C = hp.wheel_center(:);
geom.R_tire = R_tire;
geom.R_loaded = R_loaded;
geom.is_left = logical(is_left);
geom.is_front = logical(is_front);
% Static camber is physical (same sign both sides). A right corner is the
% Y-mirror of a left one, which reverses the rotation about X; applying the
% left rotation to a right spindle gave right corners positive static camber.
c = cos(static_camber);
s = sin(static_camber) * (1 - 2 * ~is_left);
R_camb = [1, 0, 0; 0, c, s; 0, -s, c];
spindle_local = is_left * [0; -1; 0] + (~is_left) * [0; 1; 0];
geom.Spindle_Dir = R_camb * spindle_local;
geom.upper_arm_axis = (geom.U_R - geom.U_F) / norm(geom.U_R - geom.U_F);
geom.upper_arm_axis_origin = geom.U_F;
geom.lower_arm_axis = (geom.L_R - geom.L_F) / norm(geom.L_R - geom.L_F);
geom.lower_arm_axis_origin = geom.L_F;
geom.kingpin_axis.origin = geom.L_U;
geom.kingpin_axis.direction = (geom.U_U - geom.L_U) / norm(geom.U_U - geom.L_U);
if has_point(hp, 'tie_rod_inner') && has_point(hp, 'tie_rod_outer')
    geom.tie_rod_chassis = hp.tie_rod_inner(:);
    geom.tie_rod_upright = hp.tie_rod_outer(:);
end
if has_point(hp, 'pushrod_chassis') && has_point(hp, 'pushrod_upright')
    geom.pushrod_chassis = hp.pushrod_chassis(:);
    geom.pushrod_upright = hp.pushrod_upright(:);
end
end

function tf = has_point(hp, name)
tf = isfield(hp, name) && numel(hp.(name)) == 3 && any(hp.(name)(:) ~= 0);
end
