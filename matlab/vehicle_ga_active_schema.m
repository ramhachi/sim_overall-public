function active = vehicle_ga_active_schema(schema, suspension_design, extra_fixed)
% VEHICLE_GA_ACTIVE_SCHEMA Select physically tunable variables for fixed-PINN GA.
%
% suspension_design: 'kc' (default) searches the representative K&C deltas
% (KC_*, 10 variables) and keeps the hardpoints at the reference; the concrete
% hardpoints are synthesised afterwards by the suspension inverse stages.
% 'hardpoints' searches the raw HP_* offsets (14 variables) instead.
% extra_fixed: optional further names to freeze, e.g.
% vehicle_ga_low_sensitivity_variables().
if nargin < 2 || isempty(suspension_design)
    suspension_design = 'kc';
end
if nargin < 3
    extra_fixed = {};
end
unknown = setdiff(cellstr(extra_fixed), schema.variable_names);
if ~isempty(unknown)
    error('vehicle_ga_active_schema:unknownVariable', 'Unknown variable(s): %s', strjoin(unknown, ', '));
end

fixed_variable_names = { ...
    'TireFzNom', 'TireRelaxationLx', 'TireRelaxationLy', ...
    'TireCx', 'TireCy', 'TireVerticalK', 'TireVerticalC', ...
    'DamperR', ...
    'CamberGainF', 'CamberGainR', ...
    'POU_Camber_C1_F', 'POU_Camber_C2_F', ...
    'Mass', 'EngineTorqueScale', ...  % lighter / more torque is always faster
    'SteeringStiffness', ...           % unused by the model: OAT lap identical at +-58 %
    'POU_Jack_C1_F', 'POU_Jack_C2_F'}; % K&C owns jacking; any offset only slows the car
switch suspension_design
    case 'kc'
        unused_prefix = 'HP_';
    case 'hardpoints'
        unused_prefix = 'KC_';
    otherwise
        error('vehicle_ga_active_schema:unknownSuspensionDesign', ...
            'suspension_design must be ''kc'' or ''hardpoints''.');
end
fixed_variable_names = [fixed_variable_names, ...
    schema.variable_names(startsWith(schema.variable_names, unused_prefix)), ...
    reshape(cellstr(extra_fixed), 1, [])];
active_mask = ~ismember(schema.variable_names, fixed_variable_names);

active = struct( ...
    'indices', find(active_mask), ...
    'variable_names', {schema.variable_names(active_mask)}, ...
    'variable_bounds', schema.variable_bounds(active_mask, :), ...
    'fixed_variable_names', {fixed_variable_names}, ...
    'full_center', mean(schema.variable_bounds, 2)');
end
