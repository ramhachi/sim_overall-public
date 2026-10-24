function [x, schema, design_context] = build_design_vector_from_params(params)
% BUILD_DESIGN_VECTOR_FROM_PARAMS Convert a baseline vehicle params struct to
% the canonical full-vehicle design vector.

if nargin < 1 || isempty(params)
    params = create_vehicle_baseline_template();
end

schema = build_full_vehicle_design_schema();
names = schema.variable_names;
x = zeros(1, numel(names));

    function idx = ix(name)
        idx = find(strcmp(names, name), 1, 'first');
        if isempty(idx)
            error('build_design_vector_from_params:missingVariable', ...
                'Variable %s is not present in the full-vehicle schema.', name);
        end
    end

width_penalty = compute_track_width_mass_distribution( ...
    [12, 12, 14, 14], params.mass.track_f, params.mass.track_r);
chassis_stiffness = field_or(params.suspension, 'K_chassis', 1000.0);
chassis_penalty = max(0.0, (chassis_stiffness - 1000.0) / 1000.0 * 4.0);

x(ix('Mass')) = params.mass.m_total - width_penalty - chassis_penalty;
x(ix('FrontDist')) = params.mass.b / max(params.mass.wheelbase, eps);
x(ix('CGHeight')) = params.mass.h_cg;
x(ix('Wheelbase')) = params.mass.wheelbase;
x(ix('TrackF')) = params.mass.track_f;
x(ix('TrackR')) = params.mass.track_r;
x(ix('SpringF')) = mean_pair(params.suspension.k_spring, 1, 2);
x(ix('SpringR')) = mean_pair(params.suspension.k_spring, 3, 4);
x(ix('DamperF')) = params.suspension.c0;
x(ix('DamperR')) = params.suspension.c0;
x(ix('ARBF')) = params.suspension.k_arb_f;
x(ix('ARBR')) = params.suspension.k_arb_r;
x(ix('CamberF')) = rad2deg(mean_pair(params.suspension.static_camber, 1, 2));
x(ix('CamberR')) = rad2deg(mean_pair(params.suspension.static_camber, 3, 4));
x(ix('ToeF')) = rad2deg(mean_abs_pair(params.suspension.static_toe, 1, 2));
x(ix('ToeR')) = rad2deg(mean_abs_pair(params.suspension.static_toe, 3, 4));
x(ix('AeroCLA')) = params.aero.CL_A;
x(ix('LSDPreload')) = params.powertrain.lsd_preload;
x(ix('LSDRampAcc')) = params.powertrain.lsd_ramp_accel;
x(ix('LSDRampDec')) = params.powertrain.lsd_ramp_decel;
x(ix('FinalDrive')) = params.powertrain.final_ratio;
x(ix('POU_Jack_C1_F')) = params.suspension.pou_jack_c1(1);
x(ix('POU_Jack_C2_F')) = params.suspension.pou_jack_c2(1);
x(ix('POU_Camber_C1_F')) = params.suspension.pou_camber_c1(1);
x(ix('POU_Camber_C2_F')) = params.suspension.pou_camber_c2(1);
x(ix('SteeringStiffness')) = params.steering.stiffness;
x(ix('MotionRatioF')) = mean_pair(params.suspension.motion_ratio, 1, 2);
x(ix('MotionRatioR')) = mean_pair(params.suspension.motion_ratio, 3, 4);
x(ix('CamberGainF')) = mean_pair(params.suspension.camber_gain, 1, 2);
x(ix('CamberGainR')) = mean_pair(params.suspension.camber_gain, 3, 4);
x(ix('ChassisStiffness')) = chassis_stiffness;
x(ix('AeroBalance')) = params.aero.aero_balance;

for i = 1:numel(schema.extra_tunable_specs)
    spec = schema.extra_tunable_specs(i);
    switch spec.name
        case 'BrakeBalance'
            value = params.brake.balance;
        case 'SteeringAckermann'
            value = params.steering.ackermann;
        case 'EngineTorqueScale'
            value = 1.0;
        case 'TireFzNom'
            value = params.tire.Fz_nom;
        case 'TireRelaxationLx'
            relax = field_or(params.tire, 'relaxation', struct());
            value = field_or(relax, 'Lx_base', params.tire.Lx);
        case 'TireRelaxationLy'
            relax = field_or(params.tire, 'relaxation', struct());
            value = field_or(relax, 'Ly_base', params.tire.Ly);
        case 'TireCx'
            value = params.tire.Cx;
        case 'TireCy'
            value = params.tire.Cy;
        case 'TireVerticalK'
            value = params.tire.k_tire;
        case 'TireVerticalC'
            value = params.tire.c_tire;
        case 'BumpStopGap'
            value = params.suspension.bump_stop_gap;
        case 'BumpStopK'
            value = params.suspension.bump_stop_k;
        case 'ChassisDamping'
            value = params.suspension.C_chassis;
        case 'ChassisTwistInertia'
            value = params.suspension.I_twist;
        otherwise
            error('build_design_vector_from_params:unknownExtendedVariable', ...
                'Unsupported extended variable: %s', spec.name);
    end
    x(spec.index) = value;
end

design_context = struct('extra_tunable_specs', schema.extra_tunable_specs);
end

function value = field_or(s, name, fallback)
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end

function value = mean_pair(values, a, b)
value = mean([values(a), values(b)]);
end

function value = mean_abs_pair(values, a, b)
value = mean(abs([values(a), values(b)]));
end
