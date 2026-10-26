function params = apply_kc_static_alignment(params)
% APPLY_KC_STATIC_ALIGNMENT Shift KC LUT/polynomial alignment to params.
%
% The runtime prefers kc.corners.<X>.sample_values over the legacy
% static_camber/camber_gain/static_toe fields whenever a KC block is present.
% After apply_full_design changes those alignment variables, the KC curves
% must move with them or the design variables become inert.

if ~isfield(params, 'suspension') || ~isfield(params.suspension, 'kc')
    return;
end
kc = params.suspension.kc;
if ~isfield(kc, 'corners') || ~isstruct(kc.corners)
    return;
end

% KC corner sign convention: corners store physical camber with the same sign
% on both sides (a right corner is the exact Y-mirror of the left one); the
% C++ runtime mirrors IA for the left wheels. FR used to carry a flipped
% static camber from geom_from_hardpoints; compute_vehicle_kc rebuilds it.
camber_sign = [1, 1, 1, 1];
axles = { ...
    'front', {'FL', 'FR'}, 1; ...
    'rear',  {'RL', 'RR'}, 3};

for a = 1:size(axles, 1)
    corner_names = axles{a, 2};
    base_idx = axles{a, 3};
    for c = 1:numel(corner_names)
        name = corner_names{c};
        if ~isfield(kc.corners, name)
            continue;
        end
        corner = kc.corners.(name);
        if ~isfield(corner, 'sample_values') || ~isfield(corner, 'kinematics')
            continue;
        end
        sv = corner.sample_values;
        kin = corner.kinematics;
        if ~isfield(sv, 'travel_m') || ~isfield(sv, 'camber_rad') ...
                || ~isfield(kin, 'camber_vs_travel')
            continue;
        end

        wheel_idx = base_idx + (c - 1);
        new_camber = camber_sign(wheel_idx) * params.suspension.static_camber(wheel_idx);
        old_camber = kin.camber_vs_travel(1);
        kc_gain = kin.camber_vs_travel(2);
        d_camber = new_camber - old_camber;
        sv.camber_rad = sv.camber_rad(:) + d_camber;
        kin.camber_vs_travel(1) = new_camber;
        params.suspension.camber_gain(wheel_idx) = kc_gain;

        new_toe = params.suspension.static_toe(wheel_idx);
        if isfield(sv, 'toe_rad') && isfield(kin, 'toe_vs_travel')
            old_toe = kin.toe_vs_travel(1);
            sv.toe_rad = sv.toe_rad(:) + (new_toe - old_toe);
            kin.toe_vs_travel(1) = new_toe;
        end

        if isfield(corner, 'static_alignment')
            if isfield(corner.static_alignment, 'camber_rad')
                corner.static_alignment.camber_rad = new_camber;
            end
            if isfield(corner.static_alignment, 'toe_rad')
                corner.static_alignment.toe_rad = new_toe;
            end
        end

        corner.sample_values = sv;
        corner.kinematics = kin;
        kc.corners.(name) = corner;
    end
end

params.suspension.kc = kc;
end
