function size_class = getRequiredSizeClass(group_name, reg_params)
% GETREQUIREDSIZECLASS Returns the required FSAE size class for a given element group.
%
% Inputs:
%   group_name - String identifier (e.g., 'main_hoop', 'front_bulkhead')
%   reg_params - Struct loaded from regulation_params.json
%
% Output:
%   size_class - String 'A', 'B', 'C', 'D' or 'None'
%
% Unknown groups are an error. Silently returning 'None' lets unlisted group
% names bypass the size check entirely.

    if ~isfield(reg_params, 'element_groups') || ...
            ~isfield(reg_params.element_groups, group_name)
        error('getRequiredSizeClass:UnknownGroup', ...
            'No regulation size class is defined for frame group "%s".', group_name);
    end

    group = reg_params.element_groups.(group_name);
    if ~isfield(group, 'required_size')
        error('getRequiredSizeClass:MissingRequiredSize', ...
            'Frame group "%s" has no required_size.', group_name);
    end
    size_class = group.required_size;
end
