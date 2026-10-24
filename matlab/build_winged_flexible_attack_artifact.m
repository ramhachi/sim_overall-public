function artifact = build_winged_flexible_attack_artifact()
% BUILD_WINGED_FLEXIBLE_ATTACK_ARTIFACT Build a canonical attack-lap starter.
%
% This preset is a physically consistent migration starter, not an
% optimized result. The legacy winged artifact used an obsolete asymmetric
% POU schema and contained values outside the current optimizer bounds.

schema = build_full_vehicle_design_schema();
base = create_vehicle_baseline_template();
design_vector = mean(schema.variable_bounds, 2)';

design_vector = set_value(schema, design_vector, 'Mass', 324.4693867209105);
design_vector = set_value(schema, design_vector, 'FrontDist', 0.5620882797711331);
design_vector = set_value(schema, design_vector, 'CGHeight', 0.3640223333790087);
design_vector = set_value(schema, design_vector, 'Wheelbase', 1.5722316327010397);
design_vector = set_value(schema, design_vector, 'TrackF', 1.45);
design_vector = set_value(schema, design_vector, 'TrackR', 1.45);
design_vector = set_value(schema, design_vector, 'SpringF', 37758.22709988651);
design_vector = set_value(schema, design_vector, 'SpringR', 17282.790331496446);
design_vector = set_value(schema, design_vector, 'DamperF', 666.2761819131739);
design_vector = set_value(schema, design_vector, 'DamperR', 2277.088299316311);
design_vector = set_value(schema, design_vector, 'ARBF', 4600.126085310965);
design_vector = set_value(schema, design_vector, 'ARBR', 4860.288256833499);
design_vector = set_value(schema, design_vector, 'CamberF', -5.0);
design_vector = set_value(schema, design_vector, 'CamberR', -1.0267508208764737);
design_vector = set_value(schema, design_vector, 'ToeF', -2.8475198387963165);
design_vector = set_value(schema, design_vector, 'ToeR', 0.4963263407363083);
design_vector = set_value(schema, design_vector, 'AeroCLA', 4.942344032773585);
design_vector = set_value(schema, design_vector, 'LSDPreload', 59.04364972176211);
design_vector = set_value(schema, design_vector, 'LSDRampAcc', 45.44013684650006);
design_vector = set_value(schema, design_vector, 'LSDRampDec', 56.98382998970319);
design_vector = set_value(schema, design_vector, 'FinalDrive', 9.91044148254714);

% The obsolete artifact encoded eight per-wheel POU values. Current
% optimization uses four front-symmetric values, so restart from neutral.
design_vector = set_value(schema, design_vector, 'POU_Jack_C1_F', 0.0);
design_vector = set_value(schema, design_vector, 'POU_Jack_C2_F', 0.0);
design_vector = set_value(schema, design_vector, 'POU_Camber_C1_F', 0.0);
design_vector = set_value(schema, design_vector, 'POU_Camber_C2_F', 0.0);

design_vector = set_value(schema, design_vector, 'SteeringStiffness', 17642.78526629621);
design_vector = set_value(schema, design_vector, 'MotionRatioF', 0.85);
design_vector = set_value(schema, design_vector, 'MotionRatioR', 0.90);
design_vector = set_value(schema, design_vector, 'CamberGainF', -0.40);
design_vector = set_value(schema, design_vector, 'CamberGainR', -0.35);
design_vector = set_value(schema, design_vector, 'ChassisStiffness', 2000.0);
design_vector = set_value(schema, design_vector, 'AeroBalance', 0.5620882797711331);

params = apply_full_design(base, design_vector, ...
    struct('extra_tunable_specs', schema.extra_tunable_specs));
params.tire.mu_y_max = 2.1826587385738057;
params.tire.mu_x_max = 1.691329369286903;
params.steering.torque_mode = true;
params.steering.I_steer = 0.02;
params.steering.R_steer = 4.5;
params.steering.T_fric = 0.3;
params.steering.C_steer_torque = 0.3;
params.steering.T_driver_max = 40.0;
params.steering.T_driver_rate_max = 0.0;
params.drs.enabled = false;

artifact = struct( ...
    'event', 'autocross', ...
    'schema_version', schema.version, ...
    'params', params, ...
    'design_vector', design_vector, ...
    'variable_names', {schema.variable_names}, ...
    'provenance', struct( ...
        'kind', 'canonical_migration_starter', ...
        'source', 'matlab/optimized_autocrossDRS.json', ...
        'note', ['Legacy out-of-bounds geometry was clamped and obsolete ', ...
            'asymmetric POU values were reset before re-optimization.']));
assert_valid_vehicle_parameter_artifact(artifact);
end

function design_vector = set_value(schema, design_vector, name, value)
index = find(strcmp(schema.variable_names, name), 1);
if isempty(index)
    error('build_winged_flexible_attack_artifact:missingSchemaEntry', ...
        'Schema entry %s was not found.', name);
end
design_vector(index) = value;
end
