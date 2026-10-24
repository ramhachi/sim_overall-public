function [is_valid, reason] = checkMaterialRestriction(group_name, pipe_data, material_data, reg_params)
% CHECKMATERIALRESTRICTION Checks if a selected material/pipe is allowed for a specific group.
%
% Inputs:
%   group_name    - String identifier (e.g., 'main_hoop')
%   pipe_data     - Struct representing the specific pipe (t, D_outer, etc.)
%   material_data - Struct for the specific material from materials.json
%   reg_params    - Struct loaded from regulation_params.json
%
% Outputs:
%   is_valid - Boolean, true if material satisfies restrictions
%   reason   - String explaining why it failed (empty if valid)

    is_valid = true;
    reason = '';

    % Fail closed for unlisted groups instead of silently skipping every rule.
    if ~isfield(reg_params, 'element_groups') || ...
            ~isfield(reg_params.element_groups, group_name)
        is_valid = false;
        reason = sprintf('No material rules are defined for frame group "%s".', group_name);
        return;
    end

    % 1. Check if group requires steel
    requires_steel = false;
    if isfield(reg_params, 'material_restrictions') && isfield(reg_params.material_restrictions, 'steel_only_groups')
        steel_groups = reg_params.material_restrictions.steel_only_groups;
        if any(strcmp(steel_groups, group_name)) || (iscell(steel_groups) && any(cellfun(@(x) strcmp(x, group_name), steel_groups)))
            requires_steel = true;
        end
    end
    
    if requires_steel
        % Check if the material is steel
        if ~isfield(material_data, 'is_steel') || ~material_data.is_steel
            is_valid = false;
            reason = sprintf('Group "%s" requires steel, but "%s" is not steel.', group_name, material_data.name);
            return;
        end
    end
    
    % 2. Check if composites are used at regulation-defined locations
    is_composite = isfield(material_data, 'is_composite') && material_data.is_composite;
    if is_composite
        composite_forbidden = {};
        if isfield(reg_params, 'material_restrictions') && isfield(reg_params.material_restrictions, 'composite_forbidden_groups')
            composite_forbidden = reg_params.material_restrictions.composite_forbidden_groups;
        end
        if any(strcmp(composite_forbidden, group_name))
            is_valid = false;
            reason = sprintf('Group "%s" is regulation-defined; composite "%s" is not permitted here.', group_name, material_data.name);
            return;
        end
    end

    % 3. Check aluminum minimum thickness (F.3.5.3).
    % Apply this only to actual aluminum; other non-steel materials (CFRP,
    % titanium, ...) have different rules.
    is_aluminum = isfield(material_data, 'is_aluminum') && material_data.is_aluminum;
    if is_aluminum
        t_min_al = 2.0; % default unwelded
        if isfield(reg_params, 'material_restrictions') && isfield(reg_params.material_restrictions, 'aluminum_min_thickness_unwelded_mm')
            t_min_al = reg_params.material_restrictions.aluminum_min_thickness_unwelded_mm;
        end
        if isfield(material_data, 'min_thickness_mm')
            t_min_al = max(t_min_al, material_data.min_thickness_mm);
        end
        
        t_min_al_welded = 3.0; % default welded
        if isfield(reg_params, 'material_restrictions') && isfield(reg_params.material_restrictions, 'aluminum_min_thickness_welded_mm')
            t_min_al_welded = reg_params.material_restrictions.aluminum_min_thickness_welded_mm;
        end
        if isfield(material_data, 'min_thickness_welded_mm')
            t_min_al_welded = max(t_min_al_welded, material_data.min_thickness_welded_mm);
        end
        
        % For simplicity, we check against the stricter welded minimum thickness
        % since frame tubes are typically welded.
        t = pipe_data.t_mm;
        if t < t_min_al_welded
            is_valid = false;
            reason = sprintf('Aluminum pipe thickness (%.2f mm) is below required minimum for welded joints (%.2f mm) for "%s".', t, t_min_al_welded, group_name);
            return;
        end
    end
end
