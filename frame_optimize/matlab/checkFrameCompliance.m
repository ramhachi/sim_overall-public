function [pass, penalties, report] = checkFrameCompliance(frame_struct, catalog, materials, reg_params)
% CHECKFRAMECOMPLIANCE Check FSAE 2026 regulation compliance
%
% Syntax:
%   [pass, penalties, report] = checkFrameCompliance(frame_struct, catalog, materials, reg_params)
%
% Inputs:
%   frame_struct - Struct with fields:
%                  nodes: Nx3 [x, y, z]
%                  elements: Mx9 [node_i, node_j, D_outer, t, E, G, rho, shape, joint_factor]
%                  element_groups: Struct mapping group name -> element indices
%                  element_materials: Optional cell array of material IDs per element
%   catalog      - Pipe catalog from loadPipeCatalog()
%   materials    - Materials from loadMaterials()
%   reg_params   - Regulation params (from JSON or loadRegulationParams())
%
% Outputs:
%   pass      - true if all regulations passed, false otherwise
%   penalties - Total penalty value (0 if all pass)
%   report    - Detailed struct with violation information

    narginchk(4, 4);

    penalties = 0;
    report = struct();
    report.size_violations = {};
    report.material_violations = {};
    report.missing_structures = {};
    report.aluminum_thickness_violations = {};
    report.section_violations = {};
    report.driver_template_violations = {};

    if ischar(reg_params) || isstring(reg_params)
        reg_params = loadRegulationParams(reg_params);
    end

    [~, steel_mat] = steelReferenceMaterial(materials);

    group_names = {};
    if isfield(frame_struct, 'element_groups')
        group_names = fieldnames(frame_struct.element_groups);
    end

    %% 1. Check pipe size compliance (F.3.4.1, F.3.5.2, F.4.1.3)

    fprintf('  Checking size compliance (F.3.4.1)...\n');

    for g = 1:length(group_names)
        group_name = group_names{g};
        elem_indices = frame_struct.element_groups.(group_name);
        required_size = getRequiredSizeClass(group_name, reg_params);

        for idx = elem_indices(:)'
            if idx > size(frame_struct.elements, 1)
                continue;
            end

            elem = frame_struct.elements(idx, :);
            D_outer = elem(3);
            t = elem(4);
            shape_code = elem(8);
            if shape_code == 2
                shape = 'square';
            else
                shape = 'round';
            end

            [~, material_data] = resolveElementMaterial(frame_struct, idx, materials);
            props = computeTubeSectionProperties(D_outer, t, shape);
            actual_size = determineSizeClass(props.Iy_mm4, props.A_mm2, D_outer, t);

            [is_compliant, deficiency] = isTubeSizeCompliant( ...
                props, D_outer, t, required_size, reg_params, material_data, steel_mat);

            if ~is_compliant
                penalty = 10000 * max(deficiency, 0.1)^2;
                penalties = penalties + penalty;

                violation = struct();
                violation.element_id = idx;
                violation.group = group_name;
                violation.required_size = required_size;
                violation.actual_size = actual_size;
                violation.material = material_data.id;
                violation.D_outer_mm = D_outer;
                violation.t_mm = t;
                violation.I_mm4 = props.Iy_mm4;
                violation.A_mm2 = props.A_mm2;
                violation.penalty = penalty;

                report.size_violations{end+1} = violation;
            end
        end
    end

    fprintf('    Size violations: %d\n', length(report.size_violations));

    %% 2. Check material restrictions and aluminum thickness (F.3.2.1, F.3.5.3)

    fprintf('  Checking material restrictions (F.3.2.1)...\n');

    for g = 1:length(group_names)
        group_name = group_names{g};
        elem_indices = frame_struct.element_groups.(group_name);

        for idx = elem_indices(:)'
            if idx > size(frame_struct.elements, 1)
                continue;
            end

            elem = frame_struct.elements(idx, :);
            [material_id, material_data] = resolveElementMaterial(frame_struct, idx, materials);
            pipe_data = struct();
            pipe_data.D_outer_mm = elem(3);
            pipe_data.t_mm = elem(4);

            [is_valid, reason] = checkMaterialRestriction( ...
                group_name, pipe_data, material_data, reg_params);

            if ~is_valid
                penalties = penalties + 100000;

                violation = struct();
                violation.element_id = idx;
                violation.group = group_name;
                violation.material = material_id;
                violation.reason = reason;
                violation.penalty = 100000;

                report.material_violations{end+1} = violation;
            end
        end
    end

    fprintf('    Material violations: %d\n', length(report.material_violations));

    %% 3. Check required structures exist

    fprintf('  Checking required structures...\n');

    required_groups = {'main_hoop', 'front_hoop'};

    if isfield(frame_struct, 'element_groups')
        existing_groups = fieldnames(frame_struct.element_groups);

        for r = 1:length(required_groups)
            req_group = required_groups{r};

            if ~ismember(req_group, existing_groups)
                penalties = penalties + 50000;
                report.missing_structures{end+1} = req_group;
            elseif isempty(frame_struct.element_groups.(req_group))
                penalties = penalties + 50000;
                report.missing_structures{end+1} = req_group;
            end
        end
    else
        penalties = penalties + 50000 * length(required_groups);
        report.missing_structures = required_groups;
    end

    fprintf('    Missing structures: %d\n', length(report.missing_structures));

    %% 4. F.6 section-level requirements (V2 groups)
    if ismember('front_bulkhead', group_names) && isfield(reg_params, 'section_requirements')
        fprintf('  Checking F.6 section requirements...\n');
        requirements = reg_params.section_requirements;
        section_groups = {'fbh_support', 'side_impact', 'mh_brace_support'};
        for sg = 1:numel(section_groups)
            group_name = section_groups{sg};
            if ~isfield(requirements, group_name)
                continue;
            end
            req = requirements.(group_name);
            elem_indices = [];
            if isfield(frame_struct.element_groups, group_name)
                elem_indices = frame_struct.element_groups.(group_name);
            end
            [left_count, right_count] = countGroupSides(frame_struct, elem_indices);
            min_per_side = req.min_per_side;
            if left_count < min_per_side || right_count < min_per_side
                penalties = penalties + 50000;
                violation = struct();
                violation.group = group_name;
                violation.rules = req.rules;
                violation.min_per_side = min_per_side;
                violation.left_count = left_count;
                violation.right_count = right_count;
                violation.element_ids = reshape(elem_indices, 1, []);
                violation.penalty = 50000;
                report.section_violations{end+1} = violation;
            end
        end

        if isfield(requirements, 'driver_template') && ...
                isfield(frame_struct.element_groups, 'front_hoop') && ...
                isfield(frame_struct.element_groups, 'main_hoop')
            req = requirements.driver_template;
            fh_idx = unique(reshape(frame_struct.element_groups.front_hoop, 1, []));
            mh_idx = unique(reshape(frame_struct.element_groups.main_hoop, 1, []));
            fh_nodes = unique(reshape(frame_struct.elements(fh_idx, 1:2), 1, []));
            mh_nodes = unique(reshape(frame_struct.elements(mh_idx, 1:2), 1, []));
            fh_half_width = max(abs(frame_struct.nodes(fh_nodes, 2)));
            mh_half_width = max(abs(frame_struct.nodes(mh_nodes, 2)));
            cockpit_width = 2 * min(fh_half_width, mh_half_width);
            cockpit_length = mean(frame_struct.nodes(mh_nodes, 1)) - ...
                mean(frame_struct.nodes(fh_nodes, 1));
            if cockpit_width < req.min_width_m || cockpit_length < req.min_length_m
                penalties = penalties + 50000;
                violation = struct();
                violation.rule = 'F.5.6.4';
                violation.width_m = cockpit_width;
                violation.length_m = cockpit_length;
                violation.min_width_m = req.min_width_m;
                violation.min_length_m = req.min_length_m;
                violation.penalty = 50000;
                report.driver_template_violations{end+1} = violation;
            end
        end
        fprintf('    Section violations: %d, driver-template violations: %d\n', ...
            length(report.section_violations), length(report.driver_template_violations));
    end

    %% Summary

    pass = (penalties == 0);

    fprintf('  Total penalty: %.0f\n', penalties);
    if pass
        fprintf('  ✓ All regulations PASSED\n');
    else
        fprintf('  ✗ Regulations FAILED\n');
    end
end


function [left_count, right_count] = countGroupSides(frame_struct, elem_indices)
    left_count = 0;
    right_count = 0;
    for idx = reshape(elem_indices, 1, [])
        if idx < 1 || idx > size(frame_struct.elements, 1)
            continue;
        end
        n1 = frame_struct.elements(idx, 1);
        n2 = frame_struct.elements(idx, 2);
        y_mid = mean(frame_struct.nodes([n1, n2], 2));
        if y_mid > 1e-6
            right_count = right_count + 1;
        elseif y_mid < -1e-6
            left_count = left_count + 1;
        end
    end
end

function [material_id, material_data] = resolveElementMaterial(frame_struct, idx, materials)
    if isfield(frame_struct, 'element_materials') && ...
            numel(frame_struct.element_materials) >= idx
        candidate = frame_struct.element_materials{idx};
        if (ischar(candidate) || isstring(candidate)) && strlength(string(candidate)) > 0
            candidate = char(candidate);
            if isfield(materials, candidate)
                material_id = candidate;
                material_data = materials.(candidate);
                return;
            end
        end
    end

    E_GPa = frame_struct.elements(idx, 5);
    [material_id, material_data] = inferMaterialFromElasticModulus(E_GPa, materials);
end

function [material_id, material_data] = inferMaterialFromElasticModulus(E_GPa, materials)
    names = fieldnames(materials);
    if isempty(names)
        error('checkFrameCompliance:MissingMaterials', 'materials must contain at least one material.');
    end

    best_err = inf;
    best_names = {};
    for i = 1:numel(names)
        mat = materials.(names{i});
        if isfield(mat, 'E_GPa')
            err = abs(mat.E_GPa - E_GPa);
            if err < best_err - 1e-9
                best_err = err;
                best_names = names(i);
            elseif abs(err - best_err) <= 1e-9
                best_names{end+1} = names{i};
            end
        end
    end

    if isempty(best_names)
        error('checkFrameCompliance:UnknownMaterial', ...
            'No material matches E = %.3f GPa.', E_GPa);
    end
    if numel(best_names) > 1
        warning('checkFrameCompliance:AmbiguousMaterial', ...
            'E = %.3f GPa matches multiple materials (%s); using %s. Pass element_materials to disambiguate.', ...
            E_GPa, strjoin(best_names, ', '), best_names{1});
    end

    material_id = best_names{1};
    material_data = materials.(material_id);
end

function [steel_id, steel_mat] = steelReferenceMaterial(materials)
    names = fieldnames(materials);
    steel_id = '';
    for i = 1:numel(names)
        mat = materials.(names{i});
        if isfield(mat, 'is_steel') && mat.is_steel
            steel_id = names{i};
            steel_mat = mat;
            if strcmp(names{i}, 'AISI4130')
                return;
            end
        end
    end
    if isempty(steel_id)
        error('checkFrameCompliance:MissingSteelReference', ...
            'A steel reference material is required for size equivalence checks.');
    end
end

function [is_compliant, deficiency] = isTubeSizeCompliant( ...
        props, D_outer, t, required_size, reg_params, material_data, steel_mat)
    if strcmp(required_size, 'None')
        is_compliant = true;
        deficiency = 0;
        return;
    end
    if ~isfield(reg_params, 'size_classes') || ~isfield(reg_params.size_classes, required_size)
        is_compliant = false;
        deficiency = 1;
        return;
    end

    req = reg_params.size_classes.(required_size);
    ratios = sizeRatios(props, D_outer, t, req, material_data, steel_mat);
    is_compliant = all(ratios >= 1);
    deficiency = norm(max(0, 1 - ratios), 2);
end

function ratios = sizeRatios(props, D_outer, t, req, material_data, steel_mat)
    is_steel = isfield(material_data, 'is_steel') && material_data.is_steel;

    if is_steel
        ratios = [props.Iy_mm4 / req.I_min_mm4, ...
                  props.A_mm2 / req.A_min_mm2, ...
                  D_outer / req.D_min_mm, ...
                  t / req.t_min_mm];
        return;
    end

    % F.3.5.2 / F.4.1.3 equivalence: compare bending strength, axial
    % strength, buckling stiffness, and energy capacity against the steel
    % size-class minimum, then keep the geometry minima as a lower bound.
    E_mat = material_data.E_GPa;
    E_steel = steel_mat.E_GPa;
    Sy_mat = weldedStrength(material_data, 'Sy');
    Sy_steel = weldedStrength(steel_mat, 'Sy');
    Su_mat = weldedStrength(material_data, 'Su');
    Su_steel = weldedStrength(steel_mat, 'Su');

    c = max(D_outer / 2, eps);
    c_req = max(req.D_min_mm / 2, eps);

    ratios = [ ...
        (props.Iy_mm4 / c * Sy_mat) / (req.I_min_mm4 / c_req * Sy_steel), ...
        (props.Iy_mm4 / c * Su_mat) / (req.I_min_mm4 / c_req * Su_steel), ...
        (props.A_mm2 * Sy_mat) / (req.A_min_mm2 * Sy_steel), ...
        (E_mat * props.Iy_mm4) / (E_steel * req.I_min_mm4), ...
        (props.A_mm2 * Su_mat) / (req.A_min_mm2 * Su_steel)];

    % F.4.1.3 replaces the steel D/t thresholds with equivalence; only the
    % material-specific minimum thickness remains as a geometry constraint.
    if isfield(material_data, 'min_thickness_mm') && material_data.min_thickness_mm > 0
        ratios(end+1) = t / material_data.min_thickness_mm;
    end
end

function value = weldedStrength(material_data, prefix)
    welded_field = [prefix '_welded_MPa'];
    unwelded_field = [prefix '_unwelded_MPa'];
    if isfield(material_data, welded_field)
        value = material_data.(welded_field);
    elseif isfield(material_data, unwelded_field)
        value = material_data.(unwelded_field);
    else
        error('checkFrameCompliance:MissingStrength', ...
            'Material "%s" has no %s strength.', material_data.id, prefix);
    end
end
