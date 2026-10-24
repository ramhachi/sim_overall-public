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
load_cases(4).description = 'Rollover proxy (1.5G lateral + 1.5G vertical at main hoop / upper rear nodes)';

%% Case 5: Differential reaction (only when diff mount nodes are declared)
if isfield(bc_config, 'diff_mount_nodes') && ~isempty(bc_config.diff_mount_nodes)
    [bcs_d, loads_d] = generate_fsae_boundary_conditions(nodes, 'diff_reaction', bc_config);
    load_cases(5).name = 'diff_reaction';
    load_cases(5).bcs = bcs_d;
    load_cases(5).loads = loads_d;
    load_cases(5).description = 'Differential mount reaction load case';
end

end
