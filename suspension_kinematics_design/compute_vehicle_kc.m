function params = compute_vehicle_kc(params)
% COMPUTE_VEHICLE_KC Recompute kc.corners/front/rear/steer_map for a vehicle profile.
%
% The K&C math is the same for every vehicle; only the profile changes. The
% profile's reference hardpoints (kc.corners.<X>.hardpoints) are first fitted
% to its dimensions, then every corner is re-solved with the export-grade
% dense sweep used by the A-arm and steering stages:
%   - track:     upright-side points move laterally so the wheel centre sits
%                at +-track/2; chassis pickups stay put (longer/shorter arms).
%   - wheelbase: the whole corner moves in X to the axle (front +a, rear -b).
% Before that, suspension.hardpoint_offsets.<front|rear>.* (design variables,
% metres, up/outboard positive, Y-mirrored on the right) move the arm pickups,
% ball joints and tie rod. Only the change since the last call is applied
% (kc.applied_hardpoint_offsets), so calling this twice is harmless.
% Static camber/toe come from the profile.

script = 'suspension_kinematics_design/compute_vehicle_kc.m';
kc = params.suspension.kc;
names = {'FL', 'FR', 'RL', 'RR'};
outboard = {'upper_ball_joint', 'lower_ball_joint', 'wheel_center', 'contact_patch', ...
    'tie_rod_outer', 'pushrod_upright', 'steering_arm_outer'};
groups = { ...
    'upper_inner', {'upper_arm_chassis_fore', 'upper_arm_chassis_aft'}; ...
    'lower_inner', {'lower_arm_chassis_fore', 'lower_arm_chassis_aft'}; ...
    'ubj', {'upper_ball_joint'}; ...
    'lbj', {'lower_ball_joint'}; ...
    'tie_rod', {'tie_rod_inner', 'tie_rod_outer'}};
axle_keys = {'front', 'front', 'rear', 'rear'};
new_offsets = struct('front', axle_offsets(params, 'front'), 'rear', axle_offsets(params, 'rear'));
old_offsets = struct('front', axle_offsets(kc, 'front', 'applied_hardpoint_offsets'), ...
    'rear', axle_offsets(kc, 'rear', 'applied_hardpoint_offsets'));
for i = 1:4
    is_front = i <= 2;
    is_left = mod(i, 2) == 1;
    hp = kc.corners.(names{i}).hardpoints;
    d_new = new_offsets.(axle_keys{i});
    d_old = old_offsets.(axle_keys{i});
    for g = 1:size(groups, 1)
        up = field0(d_new, [groups{g, 1}, '_up']) - field0(d_old, [groups{g, 1}, '_up']);
        out = field0(d_new, [groups{g, 1}, '_out']) - field0(d_old, [groups{g, 1}, '_out']);
        for f = groups{g, 2}
            if isfield(hp, f{1})
                hp.(f{1})(3) = hp.(f{1})(3) - up;                     % Z down
                hp.(f{1})(2) = hp.(f{1})(2) + (1 - 2 * is_left) * out; % away from Y=0
            end
        end
    end
    if is_front
        axle_x = params.mass.a;
        half_track = 0.5 * params.mass.track_f;
    else
        axle_x = -params.mass.b;
        half_track = 0.5 * params.mass.track_r;
    end
    dx = axle_x - hp.wheel_center(1);
    dy = (1 - 2 * is_left) * half_track - hp.wheel_center(2);
    for f = fieldnames(hp)'
        p = hp.(f{1});
        p(1) = p(1) + dx;
        if ismember(f{1}, outboard)
            p(2) = p(2) + dy;
        end
        hp.(f{1}) = p;
    end
    kc.corners.(names{i}).hardpoints = hp;
end
kc.applied_hardpoint_offsets = new_offsets;
params.suspension.kc = kc;

front = resolve_axle_geometry(params, struct(), 'front');
rear = resolve_axle_geometry(params, struct(), 'rear');
dense_f = kc_dense_samples(params, suspension_design_space(params, 'front'));
rack_max = 0.03;
if isfield(kc, 'steer_map') && isfield(kc.steer_map, 'rack_travel_m')
    rack_max = max(abs(kc.steer_map.rack_travel_m));
end
dense_f.RackSamples = linspace(-rack_max, rack_max, numel(dense_f.RackSamples));
dense_f.SteerMapTravelSamples = dense_f.TravelSamples(1:2:end);
dense_r = kc_dense_samples(params, suspension_design_space(params, 'rear'));
dense_r.RackSamples = [];
dense_r.SteerSamples = 0.0;

metrics = cell(1, 4);
geoms = cell(1, 4);
for i = 1:4
    is_front = i <= 2;
    if is_front
        axle = front;
        dense = dense_f;
        track = params.mass.track_f;
        cg_to_axle = params.mass.a;
        extra = {'ResidualTolerance', 5e-4};
    else
        axle = rear;
        dense = dense_r;
        track = params.mass.track_r;
        cg_to_axle = params.mass.b;
        extra = {};
    end
    geoms{i} = geom_from_hardpoints(kc.corners.(names{i}).hardpoints, mod(i, 2) == 1, ...
        is_front, params.suspension.static_camber(i), axle.R_tire, axle.R_loaded);
    args = namedargs2cell(dense);
    metrics{i} = evaluate_double_wishbone_3d(geoms{i}, ...
        'TrackWidth', track, ...
        'Wheelbase', params.mass.wheelbase, ...
        'CGHeight', params.mass.h_cg, ...
        'CGToAxle', cg_to_axle, ...
        'BrakeBiasFront', params.brake.balance, ...
        'DriveBiasRear', 1.0, ...
        'StaticToe', params.suspension.static_toe(i), ...
        'StaticCamber', 0.0, ...
        'SteeringStiffness', params.steering.stiffness, ...
        'UseTieRod', true, ...
        extra{:}, args{:});
    if ~metrics{i}.converged
        error('compute_vehicle_kc:notConverged', ...
            'K&C solve did not converge for %s (track %.3f m, wheelbase %.3f m).', ...
            names{i}, track, params.mass.wheelbase);
    end
    kc.corners.(names{i}) = build_corner_kc(geoms{i}, metrics{i}, ...
        params.suspension.static_toe(i), params.suspension.static_camber(i), script);
end
kc.front = build_axle_kc(metrics{1}, params.suspension.motion_ratio(1:2), true);
kc.rear = build_axle_kc(metrics{3}, params.suspension.motion_ratio(3:4), false);
kc.steer_map = struct( ...
    'schema_version', 'kc_steer_map/v1', ...
    'rack_travel_m', metrics{1}.steer_map.rack_travel_m, ...
    'wheel_travel_m', metrics{1}.steer_map.wheel_travel_m, ...
    'delta_L_rad', metrics{1}.steer_map.steer_rad, ...
    'delta_R_rad', metrics{2}.steer_map.steer_rad, ...
    'layout', 'delta_X_rad[i_wheel_travel][j_rack]; delta_L uses FL travel, delta_R uses FR travel', ...
    'bump_steer', 'removed (rack=0 column); add kinematics.toe_vs_travel separately', ...
    'rack_ratio_m_per_rad', params.steering.rack_ratio, ...
    'source', script);
params.suspension.kc = kc;
end

function offsets = axle_offsets(s, axle, field)
if nargin < 3
    if isfield(s, 'suspension') && isfield(s.suspension, 'hardpoint_offsets')
        s = s.suspension.hardpoint_offsets;
    else
        s = struct();
    end
elseif isfield(s, field)
    s = s.(field);
else
    s = struct();
end
offsets = struct();
if isfield(s, axle) && isstruct(s.(axle))
    offsets = s.(axle);
end
end

function v = field0(s, name)
v = 0.0;
if isfield(s, name)
    v = s.(name);
end
end
