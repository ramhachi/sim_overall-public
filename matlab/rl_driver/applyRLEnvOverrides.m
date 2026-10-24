function applyRLEnvOverrides(env, overrides, label)
% APPLYRLENVOVERRIDES Apply validated environment property overrides.

if nargin < 3 || isempty(label)
    label = 'Env override';
end

if isempty(overrides)
    return;
end

if ~isstruct(overrides)
    error('EnvOverrides must be a struct.');
end

fields = fieldnames(overrides);
for i = 1:numel(fields)
    fieldName = fields{i};
    if ~isprop(env, fieldName)
        error('Unknown environment override: %s', fieldName);
    end
    env.(fieldName) = overrides.(fieldName);
    fprintf('  %s: %s = %s\n', label, fieldName, formatOverrideValue(overrides.(fieldName)));
end
end

function valueStr = formatOverrideValue(value)
if isnumeric(value) || islogical(value)
    if isscalar(value)
        valueStr = num2str(value);
    else
        valueStr = mat2str(value);
    end
elseif ischar(value)
    valueStr = value;
elseif isstring(value) && isscalar(value)
    valueStr = char(value);
else
    valueStr = jsonencode(value);
end
end
