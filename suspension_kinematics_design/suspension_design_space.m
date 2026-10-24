function ds = suspension_design_space(params, axleKey)
% SUSPENSION_DESIGN_SPACE Design-space constants for the A-arm stage.
% Defaults below; any field of params.suspension.design_space.<axleKey>
% (or params.suspension.design_space for shared keys) overrides them.
% Lengths [m], angles [rad] unless the name says otherwise.

isFront = strcmpi(axleKey, 'front');
ds = struct();
ds.max_upright_radius_m = 0.20;
ds.upright_x_max_m = 0.050;
ds.L_U_local_z_max_m = 0.148;
ds.max_kpi_deg = 25.0;
ds.max_caster_deg = 10.0;
ds.arm_clear_gap_m = 0.040;
ds.min_arm_length_m = 0.080;
ds.max_arm_length_m = 0.450;
ds.min_inboard_clearance_m = 0.030;
ds.wheel_envelope_margin_m = 0.020;
% Side-view pivot-axis pitch (dz/dx going forward, Z down): non-zero values
% make anti-dive/anti-squat and caster-vs-travel designable (#45).
ds.upper_axis_pitch_tan = 0.0;
ds.lower_axis_pitch_tan = 0.0;
% Optimization bounds for the two pitch variables appended to the design vector.
ds.max_pivot_pitch_tan = 0.30;
% Dense K&C export sampling (#40): travel covers bump-stop gap + margin.
ds.travel_margin_m = 0.010;
ds.travel_step_m = 0.002;
ds.roll_range_rad = deg2rad(4.0);
ds.roll_step_rad = deg2rad(0.5);
ds.steer_step_rad = deg2rad(1.0);
ds.rack_travel_max_m = 0.030;
% Rear toe link (#42): outer point offset from the lower ball joint
% (X rearward of it = trailing link, Z upward); inner point from the
% front-view zero-bump-steer construction.
ds.toe_link_outer_dx_m = -0.080;
ds.toe_link_outer_dz_m = -0.030;
% Per-corner hardpoint input (#45): kc-style hardpoint structs keyed FL/FR or
% RL/RR. With both corners present the optimizer is skipped and nothing is mirrored.
ds.hardpoints_input = struct();

if isFront
    ds.min_kingpin_z_sep_m = 0.120;
    ds.lb = [-0.450, -0.400, -0.450, -0.200, -0.050, 0.000, NaN, -0.050, 0.000, -0.050, ...
             -ds.max_pivot_pitch_tan, -ds.max_pivot_pitch_tan];
    ds.ub = [-0.300, -0.200, -0.300, -0.050, 0.050, 0.060, NaN, 0.050, 0.060, NaN, ...
             ds.max_pivot_pitch_tan, ds.max_pivot_pitch_tan];
    ds.x0 = [-0.350, -0.285, -0.320, -0.150, 0.010, 0.020, -0.120, -0.010, 0.020, 0.050, ...
             ds.upper_axis_pitch_tan, ds.lower_axis_pitch_tan];
    ds.U_U_local_z_min_m = [];   % [] = -R_tire
else
    ds.min_kingpin_z_sep_m = 0.080;
    ds.lb = [-0.450, -0.250, -0.450, -0.150, -0.050, 0.000, NaN, -0.050, 0.000, 0.060, ...
             -ds.max_pivot_pitch_tan, -ds.max_pivot_pitch_tan];
    ds.ub = [-0.300, -0.200, -0.300, -0.050, 0.050, 0.060, NaN, 0.050, 0.060, NaN, ...
             ds.max_pivot_pitch_tan, ds.max_pivot_pitch_tan];
    ds.x0 = [-0.350, -0.220, -0.320, -0.100, 0.010, 0.020, -0.120, -0.010, 0.020, 0.110, ...
             ds.upper_axis_pitch_tan, ds.lower_axis_pitch_tan];
    ds.U_U_local_z_min_m = -0.140;
end

if isfield(params, 'suspension') && isfield(params.suspension, 'design_space')
    src = params.suspension.design_space;
    ds = override(ds, src);
    if isfield(src, axleKey) && isstruct(src.(axleKey))
        ds = override(ds, src.(axleKey));
    end
end
end

function ds = override(ds, src)
names = fieldnames(src);
for i = 1:numel(names)
    if isfield(ds, names{i})
        ds.(names{i}) = src.(names{i});
    end
end
end
