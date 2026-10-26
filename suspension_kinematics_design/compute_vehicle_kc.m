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
% Vertical positions are unchanged. Static camber/toe come from the profile.

script = 'suspension_kinematics_design/compute_vehicle_kc.m';
kc = params.suspension.kc;
names = {'FL', 'FR', 'RL', 'RR'};
outboard = {'upper_ball_joint', 'lower_ball_joint', 'wheel_center', 'contact_patch', ...
    'tie_rod_outer', 'pushrod_upright', 'steering_arm_outer'};
for i = 1:4
    is_front = i <= 2;
    is_left = mod(i, 2) == 1;
    hp = kc.corners.(names{i}).hardpoints;
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
