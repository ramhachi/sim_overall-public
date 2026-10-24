function params = apply_vehicle_extra_tunables(params, x, specs)
% APPLY_VEHICLE_EXTRA_TUNABLES Apply shared extended vehicle-package vars.

if nargin < 3 || isempty(specs)
    return;
end

for i = 1:numel(specs)
    idx = specs(i).index;
    if isempty(idx) || idx > numel(x)
        continue;
    end

    value = x(idx);
    switch specs(i).apply_mode
        case 'field'
            params = set_nested_struct_field(params, specs(i).path, value);

        case 'engine_torque_scale'
            if ~isfield(params, 'powertrain') || ~isstruct(params.powertrain)
                params.powertrain = struct();
            end
            if isfield(params.powertrain, 'engine_torque_curve') && ~isempty(params.powertrain.engine_torque_curve)
                params.powertrain.engine_torque_curve = params.powertrain.engine_torque_curve * value;
            end
            if isfield(params.powertrain, 'max_torque') && ~isempty(params.powertrain.max_torque)
                params.powertrain.max_torque = params.powertrain.max_torque * value;
            end

        otherwise
            error('Unknown extra tunable apply mode: %s', specs(i).apply_mode);
    end
end

if isfield(params, 'tire') && isfield(params.tire, 'relaxation')
    if isfield(params.tire.relaxation, 'Lx_base')
        params.tire.Lx = params.tire.relaxation.Lx_base;
    end
    if isfield(params.tire.relaxation, 'Ly_base')
        params.tire.Ly = params.tire.relaxation.Ly_base;
    end
end
end
