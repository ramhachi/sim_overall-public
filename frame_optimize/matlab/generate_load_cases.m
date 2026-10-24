function load_cases = generate_load_cases(nodes, mass_kg, g, node_config)
% GENERATE_LOAD_CASES Generate load cases for FSAE vehicle frame
%
% Generates torsion, lateral bending, braking, rollover, and optional
% differential reaction cases
% using physically correct FSAE boundary conditions.
%
% Inputs:
%   nodes       - Nx3 node coordinates [m]
%   mass_kg     - vehicle total mass [kg]
%   g           - gravitational acceleration [m/s²] (default: 9.81)
%   node_config - (optional) struct with node index mapping:
%                 .rear_lower_nodes  - [left, right] (default: [5, 6])
%                 .front_lower_nodes - [left, right] (default: [1, 2])
%                 .rear_upper_nodes  - [left, right] (default: [7, 8])
%                 .front_upper_nodes - [left, right] (default: [3, 4])
%                 .main_hoop_top_nodes - main-hoop top node(s) for rollover
%                 .file_cases   - (optional) output of import_frame_load_cases; when
%                                 given, its cases replace the fixed-g lateral /
%                                 braking / diff-reaction proxies (#49). Needs
%                                 .front_susp_nodes, .rear_susp_nodes (8 each,
%                                 generator order) and .diff_mount_nodes.
%
% Output:
%   load_cases - struct array with fields:
%     .name        - case name
%     .bcs         - Bx3 boundary conditions [node_id, dof, value]
%     .loads       - Lx3 load array [node_id, dof, force_value]
%     .description - description string

if nargin < 3 || isempty(g)
    g = 9.81;
end

% Default node mapping
if nargin < 4 || isempty(node_config)
    node_config = struct();
end
if ~isfield(node_config, 'rear_lower_nodes'),  node_config.rear_lower_nodes  = [5, 6]; end
if ~isfield(node_config, 'front_lower_nodes'), node_config.front_lower_nodes = [1, 2]; end
if ~isfield(node_config, 'rear_upper_nodes'),  node_config.rear_upper_nodes  = [7, 8]; end
if ~isfield(node_config, 'front_upper_nodes'), node_config.front_upper_nodes = [3, 4]; end

% Pass through to generate_fsae_boundary_conditions
bc_config = node_config;
bc_config.torque_Nm = 1000;
bc_config.total_mass_kg = mass_kg;

load_cases = struct('name', {}, 'bcs', {}, 'loads', {}, 'description', {});

%% Case 1: Torsional rigidity test
[bcs_t, loads_t] = generate_fsae_boundary_conditions(nodes, 'torsion', bc_config);
load_cases(1).name = 'torsion';
load_cases(1).bcs = bcs_t;
load_cases(1).loads = loads_t;
load_cases(1).description = sprintf('Torsional rigidity test (1000Nm couple at front, rear fixed)');

%% Case 2: Lateral bending (cornering)
[bcs_l, loads_l] = generate_fsae_boundary_conditions(nodes, 'lateral', bc_config);
load_cases(2).name = 'lateral_bending';
load_cases(2).bcs = bcs_l;
load_cases(2).loads = loads_l;
load_cases(2).description = 'Cornering (1.5G total lateral force at front suspension mounts)';

%% Case 3: Longitudinal bending (braking)
[bcs_b, loads_b] = generate_fsae_boundary_conditions(nodes, 'longitudinal', bc_config);
load_cases(3).name = 'longitudinal_bending';
load_cases(3).bcs = bcs_b;
load_cases(3).loads = loads_b;
load_cases(3).description = 'Braking (1.0G total longitudinal force at front nodes)';

%% Case 4: Rollover
[bcs_r, loads_r] = generate_fsae_boundary_conditions(nodes, 'rollover', bc_config);
load_cases(4).name = 'rollover';
load_cases(4).bcs = bcs_r;
load_cases(4).loads = loads_r;
load_cases(4).description = 'Rollover: rule main-hoop load case at the main-hoop top';

%% Measured / simulated mount loads replace the fixed-g proxies (#49)
if isfield(node_config, 'file_cases') && ~isempty(node_config.file_cases)
    load_cases = load_cases([1, 4]);
    fc = node_config.file_cases;
    A = node_config.rear_susp_nodes;
    % 3-2-1 statically determinate support on the rear mounts: the reactions carry
    % the unbalanced (inertial) resultant. ponytail: inertia relief would spread it.
    bcs_321 = [A(1) 1 0; A(1) 2 0; A(1) 3 0; A(2) 1 0; A(2) 3 0; A(5) 3 0];
    node_sets = {node_config.front_susp_nodes, node_config.rear_susp_nodes, ...
        getFieldOr(node_config, 'diff_mount_nodes', [])};
    for k = 1:numel(fc.cases)
        ml = fc.cases(k).mount_loads;
        loads = zeros(0, 3);
        for r = 1:size(ml, 1)
            n = node_sets{ml(r, 1)}(ml(r, 2));
            for d = 1:6
                if ml(r, 2 + d) ~= 0
                    loads(end+1, :) = [n, d, ml(r, 2 + d)]; %#ok<AGROW>
                end
            end
        end
        load_cases(end+1).name = fc.cases(k).name; %#ok<AGROW>
        load_cases(end).bcs = bcs_321;
        load_cases(end).loads = loads;
        load_cases(end).description = sprintf('Mount loads from %s', fc.source);
        load_cases(end).source = fc.source;
    end
    return;
end

%% Case 5: Differential reaction (only when diff mount nodes are declared)
if isfield(bc_config, 'diff_mount_nodes') && ~isempty(bc_config.diff_mount_nodes)
    [bcs_d, loads_d] = generate_fsae_boundary_conditions(nodes, 'diff_reaction', bc_config);
    load_cases(5).name = 'diff_reaction';
    load_cases(5).bcs = bcs_d;
    load_cases(5).loads = loads_d;
    load_cases(5).description = 'Differential mount reaction load case';
end

end

function v = getFieldOr(s, f, default)
v = default;
if isfield(s, f)
    v = s.(f);
end
end
