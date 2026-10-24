function axle = resolve_axle_geometry(params, run_options, axle_key)
% RESOLVE_AXLE_GEOMETRY Resolve tire radius, wheel center, and chassis-mount X.

if nargin < 2 || isempty(run_options)
    run_options = struct();
end
if ~isfield(params, 'tire') || ~isfield(params.tire, 'radius') ...
        || ~isfinite(params.tire.radius) || params.tire.radius <= 0
    error('AxleGeometry:MissingTireRadius', ...
        'params.tire.radius is required; refusing to use a hardcoded tire radius.');
end
R_tire = params.tire.radius;
if isfield(run_options, 'tire_radius') && isfinite(run_options.tire_radius) ...
        && run_options.tire_radius > 0
    R_tire = run_options.tire_radius;
end
is_front = strcmpi(axle_key, 'front');
if is_front
    track = params.mass.track_f;
    axle_x = params.mass.a;
    wheel_name = 'FL';
else
    track = params.mass.track_r;
    axle_x = -params.mass.b;
    wheel_name = 'RL';
end
% Loaded radius from the static corner load and tire vertical stiffness (#40).
if is_front
    static_fz = 0.5 * params.mass.m_total * 9.81 * params.mass.b / params.mass.wheelbase;
else
    static_fz = 0.5 * params.mass.m_total * 9.81 * params.mass.a / params.mass.wheelbase;
end
tire_deflection = 0.0;
if isfield(params.tire, 'k_tire') && params.tire.k_tire > 0
    tire_deflection = static_fz / params.tire.k_tire;
end
R_loaded = R_tire - tire_deflection;
W_C = [axle_x; -track / 2; -R_loaded];
source = struct('wheel_center', 'mass_and_loaded_tire_radius', 'arm_x_offset', 'baseline');

hp = [];
if isfield(params, 'suspension') && isfield(params.suspension, 'kc') ...
        && isfield(params.suspension.kc, 'corners') ...
        && isfield(params.suspension.kc.corners, wheel_name) ...
        && isfield(params.suspension.kc.corners.(wheel_name), 'hardpoints')
    hp = params.suspension.kc.corners.(wheel_name).hardpoints;
    if isfield(hp, 'wheel_center') && numel(hp.wheel_center) >= 3
        W_C = reshape(hp.wheel_center(1:3), 3, 1);
        source.wheel_center = 'artifact_hardpoints';
    end
end

arm_x_offset = 0.198;
if isfield(params.suspension, 'arm_chassis_x_offset') ...
        && isfinite(params.suspension.arm_chassis_x_offset) ...
        && params.suspension.arm_chassis_x_offset > 0
    arm_x_offset = params.suspension.arm_chassis_x_offset;
    source.arm_x_offset = 'params.suspension.arm_chassis_x_offset';
end
if ~isempty(hp) && isfield(hp, 'upper_arm_chassis_fore') ...
        && isfield(hp, 'upper_arm_chassis_aft')
    offsets = abs([hp.upper_arm_chassis_fore(1) - W_C(1), ...
        hp.upper_arm_chassis_aft(1) - W_C(1)]);
    if all(isfinite(offsets)) && max(offsets) > 1e-6
        arm_x_offset = mean(offsets);
        source.arm_x_offset = 'artifact_hardpoints';
    end
end

if isfield(run_options, 'wheel_center')
    W_C = reshape(run_options.wheel_center(1:3), 3, 1);
    source.wheel_center = 'run_options';
end
if isfield(run_options, 'axle_x') && isfinite(run_options.axle_x)
    W_C(1) = run_options.axle_x;
    source.wheel_center = 'run_options_axle_x';
end
if isfield(run_options, 'arm_x_offset') && isfinite(run_options.arm_x_offset) ...
        && run_options.arm_x_offset > 0
    arm_x_offset = run_options.arm_x_offset;
    source.arm_x_offset = 'run_options';
end

axle = struct();
axle.W_C = W_C;
axle.R_tire = R_tire;
axle.R_loaded = R_loaded;
axle.static_fz_n = static_fz;
axle.tire_deflection_m = tire_deflection;
axle.arm_x_offset = arm_x_offset;
axle.source = source;
end
