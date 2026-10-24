function out = merge_nested_structs(base, override)
% MERGE_NESTED_STRUCTS Recursively overlay override onto base.

out = base;

if nargin < 2 || isempty(override)
    return;
end

if ~isstruct(override)
    out = override;
    return;
end

% Struct arrays (e.g. suspension.kc.corners with FL/RL entries) cannot be
% recursively field-merged: out.field would expand to a comma-separated list.
% Treat the override array as authoritative instead.
if numel(out) ~= 1 || numel(override) ~= 1
    out = override;
    return;
end

override_fields = fieldnames(override);
for i = 1:numel(override_fields)
    field_name = override_fields{i};
    override_value = override.(field_name);

    if isfield(out, field_name) && isscalar(out.(field_name)) && isstruct(out.(field_name)) ...
            && isscalar(override_value) && isstruct(override_value)
        out.(field_name) = merge_nested_structs(out.(field_name), override_value);
    else
        out.(field_name) = override_value;
    end
end
end
