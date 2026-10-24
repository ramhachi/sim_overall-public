function [obj_val, results] = frameObjectiveFunction(x, ~, ~, ~, config, catalog, materials, reg_params)
% FRAMEOBJECTIVEFUNCTION Evaluates the full objective function for the frame.
%
% f(x) = W_mass * f_mass + W_cg * f_cg + P_stiff + Penalties
%
% Stiffness is a PENALTY (not reward):
%   K >= K_target: P_stiff = 0 (sufficient stiffness, no further reward)
%   K_min <= K < K_target: P_stiff = gamma_below * ((K_target-K)/(K_target-K_min))^2
%   K < K_min: P_stiff = gamma_hard * ((K_min-K)/K_min)^2 + gamma_below
%
% Inputs:
%   x             - Vector of design variables [5 pipes, 4 shape, 3 distance, 2 toggles]
%   nodes_base    - Unused (we regenerate) but kept for compatibility
%   bcs           - Boundary conditions
%   loads         - Loads applied
%   config        - Optimization configuration struct
%   catalog       - Array of pipe catalog structs
%   materials     - Materials struct mapping IDs to properties
%   reg_params    - Regulation parameters struct

    schema = struct('schema_version', 'frame_design/v1');
    if isfield(config, 'design_schema') && ~isempty(config.design_schema)
        schema = config.design_schema;
    elseif isfield(config, 'design_schema_version') && strcmp(config.design_schema_version, 'frame_design/v2')
        schema = frame_design_schema_v2(numel(catalog));
    end
    design = decode_frame_design_vector(x, schema);
    is_v2 = strcmp(design.schema_version, 'frame_design/v2');
    
    shape_params = [];
    if isfield(design, 'shape_params')
        shape_params = design.shape_params;
    elseif isfield(design, 'geometry')
        shape_params = [ ...
            design.geometry.fh_top_z_m, ...
            design.geometry.si_half_width_m, ...
            design.geometry.si_upper_delta_m, ...
            design.geometry.si_x_ratio];
    end
    distance_thresholds = design.distance_thresholds;
    node_toggles = design.node_toggles;
    node_ratios = design.node_ratios;
    
    geom = config.geom;
    suspension_file = config.suspension_file;
    
    % 2. Generate Nodes geometrically (now handles dynamic node counts and flexible placement)
    try
        if is_v2
            [nodes, ~, ~, ~, ~, ~, ~, susp_nodes_idx, rear_susp_idx, v2_meta] = ...
                generate_parametric_nodes_v2(geom, suspension_file, design, config);
        else
            [nodes, ~, ~, ~, fh_x, mh_x, fbh_nodes_idx, susp_nodes_idx, rear_susp_idx] = ...
                generate_parametric_nodes(geom, suspension_file, shape_params, node_toggles, node_ratios);
            v2_meta = struct();
        end
    catch ME
        warning('Geometric generation failed: %s', ME.message);
        obj_val = rejectedDesignObjective(config, 1e8);
        results = struct('mass', NaN, 'K_torsion', NaN, 'success', false, ...
            'error', ME.message, 'design_schema_version', design.schema_version);
        return;
    end
    
    provisional = false;
    provisional_reasons = {};

    % 3. Generate Topology Adaptively
    if is_v2
        elements_geom = generate_adaptive_topology_v2(nodes, v2_meta, config, distance_thresholds);
        [cheap_pass, cheap_penalty, cheap_report] = ...
            check_frame_geometry_constraints_v2(nodes, v2_meta, design, config, elements_geom);
        if ~cheap_pass
            obj_val = rejectedDesignObjective(config, cheap_penalty);
            results = struct('mass', NaN, 'K_torsion', NaN, 'success', false, ...
                'geometry_precheck_penalty', cheap_penalty, ...
                'geometry_precheck_report', cheap_report, ...
                'design_schema_version', design.schema_version);
            return;
        end
    else
        elements_geom = generate_adaptive_topology(nodes, fbh_nodes_idx, susp_nodes_idx, fh_x, mh_x, config, distance_thresholds, rear_susp_idx);
        cheap_penalty = 0;
        cheap_report = struct();
    end
    num_elements = size(elements_geom.connectivity, 1);
    
    elements_fem = elements_geom.base_elements;
    element_materials = cell(num_elements, 1);
    
    material_penalty = 0;
    geometry_penalty = 0;
    cg_penalty = 0;
    
    total_mass_calc = 0;
    cg_moment_z = 0;
    L_total = 0; % Track total pipe length for sparsity penalty
    
    for i = 1:num_elements
        group_idx = elements_geom.group_indices(i);
        
        if group_idx == 0
            continue; % Skip fixed elements (FBH, suspension brackets, etc.)
        end
        
        pipe_indices = design.pipe_group_indices_vector;
        if group_idx > numel(pipe_indices)
            pipe_idx = pipe_indices(end);
        else
            pipe_idx = pipe_indices(group_idx);
        end
        pipe_idx = round(pipe_idx);
        
        % Ensure index is within bounds
        pipe_idx = max(1, min(length(catalog), pipe_idx));
        pipe = catalog(pipe_idx);
        mat = materials.(pipe.material_id);
        element_materials{i} = pipe.material_id;
        
        elements_fem(i, 1) = elements_geom.connectivity(i, 1);
        elements_fem(i, 2) = elements_geom.connectivity(i, 2);
        elements_fem(i, 3) = pipe.D_outer_mm;
        elements_fem(i, 4) = pipe.t_mm;
        elements_fem(i, 5) = mat.E_GPa;
        elements_fem(i, 6) = mat.G_GPa;
        elements_fem(i, 7) = mat.rho_kg_m3;
        
        if strcmp(pipe.shape, 'round')
            elements_fem(i, 8) = 1;
        else
            elements_fem(i, 8) = 2;
        end
        
        % Material restrictions check (F.3.2.1)
        group_name = elements_geom.group_names{group_idx};
        [is_mat_valid, ~] = checkMaterialRestriction(group_name, pipe, mat, reg_params);
        if ~is_mat_valid
            material_penalty = material_penalty + 1e7;
        end
        
        % Minimum length / crossed pipe checking
        p1 = nodes(elements_fem(i, 1), :);
        p2 = nodes(elements_fem(i, 2), :);
        L_elem = norm(p2 - p1);

        L_total = L_total + L_elem; % Accumulate total length

        % NOTE: geometry_penalty disabled (2026-03-02) - allowing short pipes for bracing
        % if L_elem < 0.100 % Soft minimum 100mm pipe (shorter tubes are acceptable for bracing)
        %     geometry_penalty = geometry_penalty + 1e5;  % Reduced penalty (1e6 → 1e5)
        % end
    end
    
    % Timoshenko shear areas (cols 10-11, #46); the FEM uses Phi = 12EI/(G kA L^2).
    elements_fem = appendShearAreas(elements_fem);

    % Joint model (#48): per-element CIDECT jf + rigid end offsets (cols 12-13).
    % config.joint_factor (scalar) overrides jf for welded members; bonded CFRP gets none.
    bonded = cellfun(@(id) ~isempty(id) && isfield(materials.(id), 'is_composite') ...
        && materials.(id).is_composite, element_materials);
    [joint_jf, joint_offsets] = compute_joint_corrections(nodes, elements_fem, bonded);
    if ~isempty(getConfigField(config, 'joint_factor', []))
        joint_jf(~bonded) = config.joint_factor;
    end
    elements_fem(:, 9) = joint_jf;
    elements_fem(:, 12:13) = joint_offsets;

    % Calculate CG Height for newly assigned frame
    % Also calculate mass components here fully, as elements_fem is completely populated now.
    for i = 1:num_elements
        p1 = nodes(elements_fem(i, 1), :);
        p2 = nodes(elements_fem(i, 2), :);
        L_elem = norm(p2 - p1);
        mid_pt_z = (p1(3) + p2(3)) / 2;
        
        D_out_m = elements_fem(i, 3) / 1000; % Convert mm to meters
        t_wall_m = elements_fem(i, 4) / 1000;
        rho = elements_fem(i, 7);
        
        area = tubeAreaM2(D_out_m, t_wall_m, elements_fem(i, 8));
        mass_elem = L_elem * area * rho;
        
        total_mass_calc = total_mass_calc + mass_elem;
        cg_moment_z = cg_moment_z + (mass_elem * mid_pt_z);
    end
    
    current_cg_z = cg_moment_z / total_mass_calc;
    if isfield(config, 'max_cg_z') && current_cg_z > config.max_cg_z
        % Add penalty scaling with how far above the CG limit it is
        excess_cg = current_cg_z - config.max_cg_z;
        % Add 5 million baseline penalty + scale with excess height
        cg_penalty = 5e6 + (excess_cg * 1e8);
    end
    
    % --- 2. Dynamic FEM Load mapping ---
    node_config = struct();
    
    % Use rear suspension mounts as fixed points when available
    if ~isempty(rear_susp_idx)
        rear_left = rear_susp_idx([1, 3, 5, 7]);  % RL mounts
        rear_right = rear_susp_idx([2, 4, 6, 8]); % RR mounts
        node_config.rear_susp_left_nodes = rear_left;
        node_config.rear_susp_right_nodes = rear_right;
        node_config.rear_lower_nodes = [rear_susp_idx(1), rear_susp_idx(2)];
        node_config.rear_upper_nodes = [rear_susp_idx(3), rear_susp_idx(4)];
    elseif isfield(config, 'require_rear_suspension_hardpoints') && config.require_rear_suspension_hardpoints
        error('frameObjectiveFunction:MissingRearSuspensionHardpoints', ...
            'Rear suspension hardpoints are required by the canonical frame optimization flow.');
    elseif isfield(config, 'allow_legacy_defaults') && config.allow_legacy_defaults
        warning('frameObjectiveFunction:LegacyRearNodeFallback', ...
            'Using legacy rear-node defaults [9,10]/[13,14]; result is provisional.');
        node_config.rear_lower_nodes  = [9, 10];
        node_config.rear_upper_nodes  = [13, 14];
        provisional = true;
        provisional_reasons{end+1} = 'legacy rear-node fallback';
    else
        error('frameObjectiveFunction:MissingRearSuspensionHardpoints', ...
            'Rear suspension hardpoints are missing. Pass real nodes or set allow_legacy_defaults=true.');
    end
    
    % Use front suspension mounts as load points
    node_config.front_susp_left_nodes = susp_nodes_idx([1, 3, 5, 7]);
    node_config.front_susp_right_nodes = susp_nodes_idx([2, 4, 6, 8]);
    if is_v2 && isfield(v2_meta, 'diff_mount_idx') && ~isempty(v2_meta.diff_mount_idx)
        node_config.diff_mount_nodes = v2_meta.diff_mount_idx;
        node_config.diff_reaction_force_N = getConfigField(config, 'diff_reaction_force_N', 1500);
        node_config.diff_reaction_torque_Nm = getConfigField(config, 'diff_reaction_torque_Nm', 500);
    end
    
    if isfield(geom, 'dimensions') && isfield(geom.dimensions, 'total_mass_kg')
        node_config.total_mass_kg = geom.dimensions.total_mass_kg;
    elseif isfield(config, 'require_total_mass_kg') && config.require_total_mass_kg
        error('frameObjectiveFunction:MissingVehicleMass', ...
            'geom.dimensions.total_mass_kg is required by the canonical frame optimization flow.');
    elseif isfield(config, 'allow_legacy_defaults') && config.allow_legacy_defaults
        warning('frameObjectiveFunction:LegacyMassFallback', ...
            'Using legacy total mass 230 kg; result is provisional.');
        node_config.total_mass_kg = 230;
        provisional = true;
        provisional_reasons{end+1} = 'legacy total mass fallback';
    else
        error('frameObjectiveFunction:MissingVehicleMass', ...
            'geom.dimensions.total_mass_kg is missing. Pass the real mass or set allow_legacy_defaults=true.');
    end
    
    dynamic_cases = generate_load_cases(nodes, node_config.total_mass_kg, 9.81, node_config);
    evaluate_all_cases = is_v2;
    if isfield(config, 'load_case_eval_mode') && strcmpi(config.load_case_eval_mode, 'torsion_only')
        evaluate_all_cases = false;
    end
    [fem_result, case_results, case_displacement_penalty, fem_success] = ...
        solveObjectiveLoadCases(nodes, elements_fem, dynamic_cases, evaluate_all_cases, config);
    if ~fem_success
        obj_val = rejectedDesignObjective(config, 1e8);
        results = fem_result;
        results.success = false;
        results.cg_z = NaN;
        results.design_schema_version = design.schema_version;
        return;
    end
    
    % 3. Check Exclusion Zones (with group-based exemptions for mandatory infrastructure)
    [excl_penalties, collision_names] = checkExclusionZones(nodes, elements_fem(:, 1:2), config, elements_geom.group_indices, elements_geom.group_names);
    total_excl_penalty = sum(excl_penalties);

    compliance_penalty = 0;
    compliance_pass = true;
    compliance_report = struct();
    enable_compliance_check = true;
    if isfield(config, 'enable_frame_compliance_check')
        enable_compliance_check = config.enable_frame_compliance_check;
    end
    if enable_compliance_check
        try
            frame_struct = struct();
            frame_struct.nodes = nodes;
            frame_struct.elements = elements_fem;
            frame_struct.element_groups = build_element_groups(elements_geom.group_indices, elements_geom.group_names);
            frame_struct.element_materials = element_materials;
            [compliance_pass, compliance_penalty, compliance_report] = ...
                checkFrameCompliance(frame_struct, catalog, materials, reg_params);
            raw_compliance_penalty = compliance_penalty;
            compliance_scale = getConfigField(config, 'compliance_penalty_scale', 1000);
            compliance_penalty = compliance_penalty * compliance_scale;
            compliance_report.raw_penalty = raw_compliance_penalty;
            compliance_report.penalty_scale = compliance_scale;
        catch ME
            warning('Frame compliance check failed: %s', ME.message);
            compliance_pass = false;
            compliance_penalty = 1e8;
            compliance_report = struct('error', ME.message);
        end
    end
    
    % 4. Calculate Objective
    W_mass = config.W_mass;
    mass_ref = config.mass_ref;
    K_min = config.K_min;
    K_target = config.K_target;

    mass = fem_result.mass;
    K = fem_result.K_torsion;

    % Mass term (quadratic: penalizes heavy designs progressively harder)
    f_mass = (mass / mass_ref)^2;

    % CG Height term
    if isfield(config, 'cg_ref')
        cg_ref = config.cg_ref;
    else
        cg_ref = 0.300;
        provisional = true;
        provisional_reasons{end+1} = 'default cg_ref=0.300';
    end
    f_cg = current_cg_z / cg_ref;

    % Mass target penalty (exceeding target mass)
    P_mass_target = 0;
    if isfield(config, 'mass_target') && isfield(config, 'gamma_mass')
        mass_target = config.mass_target;
        if mass > mass_target
            excess_ratio = (mass - mass_target) / mass_target;
            P_mass_target = config.gamma_mass * excess_ratio^2;
        end
    end

    % Stiffness penalty (saturates at K_target — no reward for exceeding)
    gamma_below = 8.0;
    gamma_hard = 50.0;
    if isfield(config, 'gamma_below'), gamma_below = config.gamma_below; end
    if isfield(config, 'gamma_hard'), gamma_hard = config.gamma_hard; end

    if K >= K_target
        P_stiff = 0;
    elseif K >= K_min
        P_stiff = gamma_below * ((K_target - K) / (K_target - K_min))^2;
    else
        P_stiff = gamma_hard * ((K_min - K) / K_min)^2 + gamma_below;
    end
    
    % Sparsity Penalty Check (Minimize unnecessary piping)
    sparsity_penalty = 0;
    
    % Penalize if total length exceeds 45m (more realistic for FSAE frames)
    max_total_length = 45.0;
    if isfield(config, 'sparsity_max_total_length_m')
        max_total_length = config.sparsity_max_total_length_m;
    end
    sparsity_scale = getConfigField(config, 'sparsity_penalty_scale', 1e5);
    if L_total > max_total_length
        excess_length = L_total - max_total_length;
        sparsity_penalty = sparsity_penalty + (excess_length * sparsity_scale);
    end
    
    % Penalize if element count exceeds 120
    max_elements = 120;
    if isfield(config, 'sparsity_max_elements')
        max_elements = config.sparsity_max_elements;
    end
    if num_elements > max_elements
        excess_elements = num_elements - max_elements;
        sparsity_penalty = sparsity_penalty + (excess_elements^2 * sparsity_scale);
    end
    
    % Final Check: Discard individuals violating exclusion zones
    if total_excl_penalty > 0
        % Identify exactly which elements/zones are colliding
        bad_elem_idxs = find(excl_penalties > 0);
        for idx = 1:min(3, length(bad_elem_idxs)) % Show up to 3 collisions
            e_idx = bad_elem_idxs(idx);
            z_name = collision_names{e_idx};
            fprintf('   [DEBUG] Collision! Elem %d (Nodes %d-%d) intersects zone "%s".\n', ...
                e_idx, elements_fem(e_idx,1), elements_fem(e_idx,2), z_name);
        end
        fprintf('   [DEBUG] Total Exclusion Penalty: %.2f (Shape: [%.2f, %.2f, %.2f, %.2f])\n', ...
            total_excl_penalty, shape_params(1), shape_params(2), shape_params(3), shape_params(4));
        obj_val = rejectedDesignObjective(config, total_excl_penalty);
    else
        W_cg = 1.5;
        if isfield(config, 'W_cg')
            W_cg = config.W_cg;
        end
        obj_val = W_mass * f_mass + W_cg * f_cg + P_stiff + P_mass_target ...
            + material_penalty + geometry_penalty + cg_penalty + total_excl_penalty ...
            + sparsity_penalty + compliance_penalty + cheap_penalty + case_displacement_penalty;
    end
    
    % 5. Collect detailed results
    results = fem_result;
    results.f_mass = f_mass;
    results.f_cg = f_cg;
    results.P_stiff = P_stiff;
    results.P_mass_target = P_mass_target;
    results.material_penalty = material_penalty;
    results.geometry_penalty = geometry_penalty;
    results.cg_penalty = cg_penalty;
    results.exclusion_penalty = total_excl_penalty;
    results.compliance_penalty = compliance_penalty;
    results.compliance_pass = compliance_pass;
    results.compliance_report = compliance_report;
    results.geometry_precheck_penalty = cheap_penalty;
    results.geometry_precheck_report = cheap_report;
    results.case_displacement_penalty = case_displacement_penalty;
    results.load_case_results = case_results;
    results.design_schema_version = design.schema_version;
    results.solver_backend = resolveObjectiveSolverBackend(config);
    if is_v2
        results.design_variables = design;
        results.frame_meta = v2_meta;
    end
    results.sparsity_penalty = sparsity_penalty;
    results.L_total = L_total;
    results.num_elements = num_elements;
    results.obj_val = obj_val;
    results.success = true;
    results.provisional = provisional;
    results.provisional_reasons = provisional_reasons;
    results.nodes = nodes; % Pass back generated nodes
    results.elements = elements_fem; % FEM element rows actually solved
    results.cg_z = current_cg_z; % Pass back CG metadata
end

function obj_val = rejectedDesignObjective(config, penalty)
    if getConfigField(config, 'finite_rejection_penalty', false)
        obj_val = getConfigField(config, 'invalid_design_penalty', 1e9) + penalty;
    else
        obj_val = Inf;
    end
end

function element_groups = build_element_groups(group_indices, group_names)
    element_groups = struct();
    for g = 1:numel(group_names)
        element_groups.(group_names{g}) = find(group_indices == g);
    end
end

function [primary_result, case_results, displacement_penalty, success] = solveObjectiveLoadCases(nodes, elements_fem, load_cases, evaluate_all_cases, config)
    if nargin < 4
        evaluate_all_cases = false;
    end
    if nargin < 5
        config = struct();
    end

    num_cases = 1;
    if evaluate_all_cases
        num_cases = numel(load_cases);
    end

    case_results = struct('name', {}, 'success', {}, 'K_torsion', {}, 'mass', {}, ...
        'max_translation_m', {}, 'loads', {}, 'bcs', {}, 'error', {});
    displacement_penalty = 0;
    max_allowed = getConfigField(config, 'case_max_displacement_m', 0.025);
    primary_result = struct('mass', NaN, 'K_torsion', NaN, 'success', false);
    success = false;

    for ci = 1:num_cases
        try
            result = solveFemCase(nodes, elements_fem, load_cases(ci).bcs, load_cases(ci).loads, config);
        catch ME
            fprintf('   [DEBUG] FEM Solver Exception (%s): %s\n', load_cases(ci).name, ME.message);
            warning('FEM Solver failed for %s: %s', load_cases(ci).name, ME.message);
            primary_result = struct('mass', NaN, 'K_torsion', NaN, 'success', false, 'error', ME.message);
            return;
        end

        if ~result.success
            fprintf('   [DEBUG] FEM Solver returned success=false for %s\n', load_cases(ci).name);
            primary_result = result;
            return;
        end

        max_translation = extractMaxTranslation(result);
        case_results(ci).name = load_cases(ci).name;
        case_results(ci).success = true;
        case_results(ci).K_torsion = result.K_torsion;
        case_results(ci).mass = result.mass;
        case_results(ci).max_translation_m = max_translation;
        case_results(ci).loads = load_cases(ci).loads;
        case_results(ci).bcs = load_cases(ci).bcs;
        case_results(ci).error = '';

        if ci == 1
            primary_result = result;
        end
        if isfinite(max_translation) && max_translation > max_allowed
            excess = (max_translation - max_allowed) / max_allowed;
            case_results(ci).displacement_excess_m = max_translation - max_allowed;
            displacement_penalty = displacement_penalty + 25 * excess^2;
        else
            case_results(ci).displacement_excess_m = 0;
        end
    end

    success = true;
end

function result = solveFemCase(nodes, elements_fem, bcs, loads, config)
    backend = resolveObjectiveSolverBackend(config);
    switch backend
        case 'mex'
            result = beam_fem_solver(nodes, elements_fem, bcs, loads);
        case 'tcp_java'
            result = beam_fem_solver_tcp_java(nodes, elements_fem, bcs, loads);
        otherwise
            error('frameObjectiveFunction:UnknownSolverBackend', ...
                'Unknown solver backend "%s". Use auto/mex/tcp_java.', backend);
    end
end

function backend = resolveObjectiveSolverBackend(config)
    requested = getConfigField(config, 'solver_backend', 'tcp_java');
    requested = lower(char(string(requested)));
    if strcmp(requested, 'auto')
        if ensureNativeMexOnPath()
            backend = 'mex';
        else
            backend = 'tcp_java';
        end
        return;
    end
    if strcmp(requested, 'mex')
        if ~ensureNativeMexOnPath()
            error('frameObjectiveFunction:MissingMexSolver', ...
                'solver_backend=mex requested, but native beam_fem_solver MEX is not available.');
        end
        backend = 'mex';
        return;
    end
    backend = requested;
end

function available = ensureNativeMexOnPath()
    matlab_dir = fileparts(mfilename('fullpath'));
    frame_root = fileparts(matlab_dir);
    mex_dir = fullfile(frame_root, 'mex');
    if exist(mex_dir, 'dir')
        addpath(mex_dir, '-begin');
    end
    solver_path = which('beam_fem_solver');
    available = ~isempty(solver_path) && contains(solver_path, ['.' mexext]);
end

function elements_fem = appendShearAreas(elements_fem)
    shapes = {'round', 'square'};
    for i = 1:size(elements_fem, 1)
        props = computeTubeSectionProperties(elements_fem(i, 3), elements_fem(i, 4), shapes{elements_fem(i, 8)});
        elements_fem(i, 10:11) = [props.Asy_mm2, props.Asz_mm2];
    end
end

function area = tubeAreaM2(D_outer_m, t_m, shape_code)
    D_inner_m = D_outer_m - 2 * t_m;
    if shape_code == 2
        area = D_outer_m^2 - D_inner_m^2;
    else
        area = pi * ((D_outer_m / 2)^2 - (D_inner_m / 2)^2);
    end
end

function max_translation = extractMaxTranslation(result)
    max_translation = NaN;
    if ~isfield(result, 'U') || isempty(result.U)
        return;
    end
    U = result.U;
    if size(U, 2) >= 3
        max_translation = max(max(abs(U(:, 1:3))));
    else
        max_translation = max(abs(U(:)));
    end
end

function value = getConfigField(config, field_name, default_value)
    value = default_value;
    if isstruct(config) && isfield(config, field_name) && ~isempty(config.(field_name))
        value = config.(field_name);
    end
end
