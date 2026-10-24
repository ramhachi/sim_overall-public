function corner = build_corner_kc(geom, metrics, staticToe, staticCamber, script, extraReasons)
% BUILD_CORNER_KC Runtime kc.corners.<X> block from one evaluated corner.
% Shared by the front/rear A-arm stage and the steering stage so every
% writer emits the same schema, quality verdict and provenance (#39/#40/#42).
%
% 14DOF roll angle phi > 0 compresses the right side. The local roll sweep
% drops the evaluated corner, so a left corner follows +phi and a right
% corner follows -phi. Jacking lifts one side and drops the other.
if nargin < 6
    extraReasons = {};
end
side = 1.0 - 2.0 * ~geom.is_left;   % +1 left, -1 right
tieRod = metrics.tie_rod_evaluated;

corner = struct();
corner.caster = deg2rad(metrics.caster_deg);
corner.kpi = deg2rad(metrics.kpi_deg);
corner.sai = deg2rad(metrics.kpi_deg);
corner.scrub_radius = metrics.scrub_radius_m;
corner.mechanical_trail = metrics.mechanical_trail_m;
corner.caster_trail = metrics.mechanical_trail_m;

hp = struct( ...
    'upper_arm_chassis_fore', geom.U_F(:)', ...
    'upper_arm_chassis_aft', geom.U_R(:)', ...
    'lower_arm_chassis_fore', geom.L_F(:)', ...
    'lower_arm_chassis_aft', geom.L_R(:)', ...
    'upper_ball_joint', geom.U_U(:)', ...
    'lower_ball_joint', geom.L_U(:)', ...
    'wheel_center', geom.W_C(:)', ...
    'contact_patch', metrics.contact_patch(:)');
if tieRod
    hp.tie_rod_inner = geom.tie_rod_chassis(:)';
    hp.tie_rod_outer = geom.tie_rod_upright(:)';
end
if isfield(geom, 'pushrod_chassis') && isfield(geom, 'pushrod_upright')
    hp.pushrod_chassis = geom.pushrod_chassis(:)';
    hp.pushrod_upright = geom.pushrod_upright(:)';
end
corner.hardpoints = hp;

kin = struct( ...
    'camber_vs_travel', metrics.camber_vs_travel, ...
    'roll_camber_gain', side * metrics.roll_camber_gain, ...
    'roll_gain_definition', metrics.roll_gain_definition, ...
    'jacking_force_per_fy', -side * metrics.jacking_force_per_fy, ...
    'anti_brake_vs_travel', metrics.anti_brake_vs_travel, ...
    'anti_drive_vs_travel', metrics.anti_drive_vs_travel);
if tieRod
    kin.toe_vs_travel = metrics.toe_vs_travel;
    kin.roll_steer_gain = side * metrics.roll_steer_gain;
    if geom.is_front
        kin.camber_vs_steer = metrics.camber_vs_steer;
        kin.toe_vs_steer = metrics.toe_vs_steer;
    end
end
corner.kinematics = kin;

s = metrics.lut_samples;
s.roll_rad = side * s.roll_rad;           % runtime roll convention
s.jacking_force_per_fy = -side * s.jacking_force_per_fy;
if side < 0
    [s.roll_rad, k] = sort(s.roll_rad);
    for f = {'roll_camber_residual_rad', 'roll_toe_residual_rad', 'roll_center_height_m'}
        s.(f{1}) = s.(f{1})(k);
    end
end
if ~tieRod
    s = rmfield(s, {'toe_rad', 'roll_toe_residual_rad'});
end
corner.sample_values = s;
corner.polyfit_quality = metrics.polyfit_quality;

if tieRod
    linkScope = 'evaluated';
else
    linkScope = 'not_evaluated';
end
if geom.is_front
    steerScope = linkScope;
else
    steerScope = 'not_applicable';
end
corner.evaluation_scope = struct( ...
    'travel', 'evaluated', ...
    'roll', 'evaluated', ...
    'steering', steerScope, ...
    'tie_rod', linkScope, ...
    'toe', linkScope, ...
    'roll_steer', linkScope, ...
    'anti', 'evaluated', ...
    'jacking', 'proxy_not_validated', ...
    'compliance', metrics.compliance.scope, ...
    'sample_values', 'evaluated');

reasons = reshape(extraReasons, 1, []);
if ~tieRod
    if geom.is_front
        reasons{end + 1} = 'tie_rod_not_evaluated';
    else
        reasons{end + 1} = 'toe_link_not_evaluated';
    end
end
if ~metrics.converged
    reasons{end + 1} = 'kinematics_not_converged';
end
if ~metrics.polyfit_quality.fit_ok
    reasons{end + 1} = 'poor_polyfit';
end
if isempty(reasons)
    verdict = 'valid';
    confidence = 'medium';  % kinematics solved; jacking/compliance stay labelled per field
else
    verdict = 'provisional';
    confidence = 'low';
end
corner.quality = struct('verdict', verdict, 'confidence', confidence, 'reasons', {reasons});
if tieRod
    model = 'a_arm_plus_tie_rod_closed_link';
else
    model = 'a_arm_only_two_dof';
end
corner.source = struct( ...
    'model', model, ...
    'script', script, ...
    'tie_rod_model', linkScope, ...
    'updated_at', datestr(now, 'yyyy-mm-dd HH:MM:SS'));

corner.compliance = metrics.compliance;
corner.proxy_metrics = {'jacking_force_per_fy'};
if metrics.compliance.provisional
    corner.proxy_metrics{end + 1} = 'compliance';
end
corner.anti_metrics = struct('dive', metrics.anti_dive_pct, ...
    'squat', metrics.anti_squat_pct, 'lift', metrics.anti_lift_pct, ...
    'rise', metrics.anti_rise_pct, 'model', metrics.anti_model, ...
    'validated', metrics.anti_validated);
corner.proxy_sources = struct( ...
    'anti', metrics.anti_model, ...
    'jacking', metrics.jacking_model, ...
    'compliance', metrics.compliance_model);
corner.solver_status = metrics.solver_status;
corner.static_alignment = struct('toe', staticToe, 'camber', staticCamber);
end
