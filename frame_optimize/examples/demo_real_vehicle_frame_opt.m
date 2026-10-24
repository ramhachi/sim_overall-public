%% DEMO_REAL_VEHICLE_FRAME - Demonstrate FEM solver with Formula TMU dimensions
%
% This script creates a 28-node FSAE-compliant frame using real Formula TMU
% vehicle dimensions and evaluates it under 3 load cases:
%   1. Torsional rigidity (±1000 Nm)
%   2. Lateral bending (1.5G cornering)
%   3. Longitudinal bending (1.0G braking)
%
% Expected output:
%   - Torsional rigidity: 500-1500 Nm/deg
%   - Mass: 15-25 kg
%   - Computation time: 2-3 ms/evaluation (TCP solver)

clear; close all; clc;

fprintf('=== Formula TMU Real Vehicle Frame Demo ===\n\n');

%% 1. Load vehicle geometry and materials

script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
matlab_dir = fullfile(script_dir, '..', 'matlab');

% Add MATLAB functions to path
addpath(matlab_dir);

fprintf('1. Loading vehicle geometry and materials...\n');

% Load real vehicle geometry
geom_file = fullfile(data_dir, 'real_vehicle_geometry.json');
if ~isfile(geom_file)
    error('Vehicle geometry file not found: %s', geom_file);
end
geom_text = fileread(geom_file);
geom = jsondecode(geom_text);

% Define ground clearance
Z_floor = 0.050; % Global minimum ground clearance (50mm)
FH_z_drop_from_fbh = 0.100; % FH is 100mm lower than FBH (offset relative to FBH bottom)

% Shift exclusion zones to respect ground clearance
for i = 1:length(geom.exclusion_zones)
    geom.exclusion_zones(i).min(3) = geom.exclusion_zones(i).min(3) + Z_floor;
    geom.exclusion_zones(i).max(3) = geom.exclusion_zones(i).max(3) + Z_floor;
end

fprintf('   Vehicle dimensions:\n');
fprintf('     Wheelbase:   %.2f m\n', geom.dimensions.wheelbase_m);
fprintf('     Track (F/R): %.2f / %.2f m\n', ...
        geom.dimensions.track_front_m, geom.dimensions.track_rear_m);
fprintf('     Total mass:  %.0f kg\n', geom.dimensions.total_mass_kg);

% Load materials and pipe catalog
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);

fprintf('   Loaded %d materials, %d pipes\n', length(fieldnames(materials)), length(catalog));

%% 2. Create real vehicle frame geometry (28 nodes)

fprintf('\n2. Creating real vehicle frame (28 nodes, FSAE-compliant)...\n');

% Extract dimensions
L = geom.dimensions.wheelbase_m;  % 1.55 m (FH to MH distance)
H_cg = geom.dimensions.cg_height_m;  % 0.28 m

%% ============================================================
%%  Node layout — Real FSAE tapered frame
%%
%%  Top view (X-Y plane) — tapers from narrow FBH to wider MH:
%%
%%     FBH        FH           SI           MH        MH_Brace
%%    ┌──┐     ┌──────┐    ┌──────┐    ┌────────┐   ┌──────┐
%%    │  │    /│      │\  /│      │\  /│        │\  │      │
%%    └──┘   / └──────┘ \/ └──────┘ \/ └────────┘ \ └──────┘
%%   X=-0.3  X=0        X=0.55      X=L=1.55     X=L+0.3
%%
%%  Side view (X-Z plane):
%%                 FH(1.0m)     SI(0.32m)      MH(1.35m)
%%                  ┌──┐                        ┌─┐
%%                  │  │                        │ │ ← SH bar
%%                  │  │        ┬ 0.32m(SI_h)   │ │
%%                  │  │        ┤ 0.05m(SI_l)   │ │
%%   FBH(0.2m)     │  │                        │ │
%%    ┌─┐          │  │                        │ │
%%    └─┘──────────└──┘────────────────────────└─┘──────
%%   ground (Z=0)
%% ============================================================

% Frame widths (half-widths from centerline)
% FBH widths are loaded dynamically below
W_fh  = 0.250;   % FH: 500mm wide (clears T.1.1.1 front width 400mm)
W_si  = 0.300;   % SI: 600mm wide (clears T.1.1.1 max width 550mm)
W_mh  = 0.300;   % MH: 600mm wide at base
W_mh_top = 0.150; % MH top: 300mm wide
W_rear = 0.250;  % Rear bracing: 500mm wide

% Frame heights (All relative to ground Z=0)
H_si_lower = 0.05 + Z_floor; % SI lower height
H_si_upper = 0.45 + Z_floor; % SI upper height
H_mh  = 1.35 + Z_floor; % MH height
H_sh  = 1.15 + Z_floor; % Shoulder harness bar height
H_rear_brace_upper_limit = 0.5; % Configurable limit for rear braces

% Primary X positions (FBH and FH are dynamic, see below)
X_mh  = L;       % Main Hoop = wheelbase
X_rear = L + 0.30; % Rear bracing/engine mount

% FBH and Validation Configuration
FBH_width = 0.400;  % 400mm width
FBH_height = 0.400; % 400mm height
FBH_z_offset_from_cv = -0.050; % User-configurable FBH Z offset from wheel center (Lowered by 50mm)
FH_z_drop_from_fbh = 0.100; % FH is 100mm lower than FBH (offset relative to FBH bottom)
FH_z_rise_from_fbh_top = 0.050; % FH top is 50mm higher than FBH top (configurable limit)

% X-axis offsets relative to suspension geometry (+X is rearward)
FBH_x_shift = 0.030; % Shift FBH 30mm rearward (closer to front suspension mounts)
FH_min_dist_from_susp = 0.150; % Minimum distance from rear-most front suspension mount to FH

% Load real front suspension hardpoints and calculate FBH geometry
base_offset_x = 0.500; % Default forward placement distance 
offset_x = base_offset_x - FBH_x_shift; % Align FBH based on shift (+X direction)
offset_z = FBH_z_offset_from_cv; % Align FBH bottom with absolute suspension geometry + user offset

suspension_file = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));
[fbh_nodes_susp, suspension_mount_nodes_susp, upright_nodes, has_suspension] = load_suspension_hardpoints(suspension_file, offset_x, offset_z, FBH_width, FBH_height);

% Calculate Front Hoop Z constraints based on absolute FBH height
fbh_min_z = min(fbh_nodes_susp(:, 3));
fbh_max_z = max(fbh_nodes_susp(:, 3));
Z_FH_lower = max(Z_floor, fbh_min_z - FH_z_drop_from_fbh); % FH is lower than FBH, bounded by 50mm global clearance
H_fh = fbh_max_z + FH_z_rise_from_fbh_top; % FH total height from ground (constrained relative to FBH top)

% The Front Hoop X-position is determined later based on suspension mount positions
inner_bracket_nodes = zeros(8, 3);
L_bracket = 0.050; % Default for fallback case without suspension

if has_suspension
    fprintf('   Added 8 front suspension mount points (Nodes 29-36).\n');
    fbh_nodes = fbh_nodes_susp;
    suspension_mount_nodes = suspension_mount_nodes_susp;
    X_fbh = fbh_nodes(1, 1);
    
    % Upright Mapping: LowerF, LowerR, UpperF, UpperR -> Upright 1,2,3,4 respectively per side
    upright_map = [1, 2, 3, 4, 1, 2, 3, 4];
    % The front hoop MUST sit behind the suspension to avoid collision.
    % Find the farthest rearward X point of the suspension mounts:
    max_susp_x = max(suspension_mount_nodes(:, 1));
    % Place the Front Hoop safely behind it based on user constraint
    X_fh = max_susp_x + FH_min_dist_from_susp;
    fprintf('   Dynamically placed Front Hoop at X = %.3f m to clear suspension.\n', X_fh);
    
    % Generate Inner Bracket Nodes (Nodes 37-44) connecting from mounts to FH plane
    for i = 1:8
        c_node = suspension_mount_nodes(i, :);
        % Vector from suspension mount directly back to the FH plane
        dx = X_fh - c_node(1);
        
        % For visual structure, angle them slightly inward/outward to meet FH width
        target_y = sign(c_node(2)) * W_fh;
        dy = target_y - c_node(2);
        
        % And height proportionally
        target_z = H_fh/2; % roughly mid height
        dz = target_z - c_node(3);
        
        inner_bracket_nodes(i, :) = c_node + [dx, dy, dz];
    end
else
    fprintf('   Suspension mounts not found, creating default points.\n');
    X_fbh = -0.30;
    W_fbh = 0.150; H_fbh = 0.25;
    fbh_nodes = [
        X_fbh, -W_fbh, 0;         
        X_fbh,  W_fbh, 0;         
        X_fbh, -W_fbh, H_fbh;     
        X_fbh,  W_fbh, H_fbh;     
    ];
    suspension_mount_nodes = zeros(8, 3);
    suspension_mount_nodes(1:2, :) = fbh_nodes(1:2, :) + [-0.05, 0, 0]; 
    suspension_mount_nodes(3:4, :) = fbh_nodes(3:4, :) + [-0.05, 0, 0]; 
    suspension_mount_nodes(5:6, :) = fbh_nodes(1:2, :) + [0.35, 0, 0]; 
    suspension_mount_nodes(7:8, :) = fbh_nodes(3:4, :) + [0.35, 0, 0]; 
    
    for i = 1:8
        c_node = suspension_mount_nodes(i, :);
        inner_bracket_nodes(i, :) = c_node - [0, sign(c_node(2)) * L_bracket, 0];
    end
    X_fh = 0.0; % Default
end

% Place Main Hoop exactly at the rear of the cockpit template (T.1.1.1)
X_mh = X_fh + 0.60;

% Place Side Impact structure proportionally halfway between FH and MH
X_si = X_fh + (X_mh - X_fh) * 0.35; % ~35% of the driver compartment

% Nodes: [x, y, z] in meters
nodes = [
    %% Front Hoop (nodes 1-4): X = X_fh
    X_fh, -W_fh, Z_FH_lower; % 1: FH lower left
    X_fh,  W_fh, Z_FH_lower; % 2: FH lower right
    X_fh, -W_fh, H_fh;       % 3: FH upper left
    X_fh,  W_fh, H_fh;       % 4: FH upper right

    %% Side Impact Structure (nodes 5-8): X = 0.55m
    X_si, -W_si, H_si_lower;  % 5: SI lower left
    X_si,  W_si, H_si_lower;  % 6: SI lower right
    X_si, -W_si, H_si_upper;  % 7: SI upper left
    X_si,  W_si, H_si_upper;  % 8: SI upper right

    %% Main Hoop Pentagon (nodes 9-13): X = X_mh (Rear of cockpit template)
    X_mh, -W_mh, Z_floor;    % 9: MH lower left
    X_mh,  W_mh, Z_floor;    % 10: MH lower right
    X_mh, -W_mh_top, H_sh;   % 11: MH shoulder left (150mm below top based on rules)
    X_mh,  W_mh_top, H_sh;   % 12: MH shoulder right
    X_mh,  0.0,      H_mh;   % 13: MH top center (Peak)

    %% MH Bracing / Rear structure (nodes 14-17): X = L + 0.3m
    X_rear, -W_rear, H_rear_brace_upper_limit;  % 14: Rear brace upper left
    X_rear,  W_rear, H_rear_brace_upper_limit;  % 15: Rear brace upper right
    X_rear, -W_rear, Z_floor;                   % 16: Rear brace lower left
    X_rear,  W_rear, Z_floor;                   % 17: Rear brace lower right

    %% 18-21: Front Bulkhead (X = -0.3m)
    fbh_nodes;
    %% 22-29: Suspension Mounts (Functional mounts)
    suspension_mount_nodes; 
];

% We already generated inner brackets above, just append them here
nodes = [nodes; inner_bracket_nodes]; % Nodes 30-37

% Elements: [node_i, node_j, D_outer, t, E, G, rho, shape, joint_factor]
% ALL FRAME ELEMENTS ARE STEEL (AISI 4130) PER USER REQUEST (No Aluminum)
D_steel_A = 25.0;   % Size A: 25mm OD (FSAE 2026 F.3.4.1)
t_steel_A = 2.0;    % Size A: 2.0mm wall
D_steel_B = 25.0;   % Size B: 25mm OD
t_steel_B = 1.6;    % Size B: 1.6mm wall (exceeds 1.2mm min)
E_steel = materials.AISI4130.E_GPa;
G_steel = materials.AISI4130.G_GPa;
rho_steel = materials.AISI4130.rho_kg_m3;

shape = 1;           % 1=round tube
jf = 0.85;           % Joint efficiency

%% ============================================================
%%  Adaptive Topology Preparation
%% ============================================================
% Elements and connectivity are no longer hardcoded here.
% They are dynamically generated inside the objective function
% using generate_adaptive_topology.m based on distance thresholds.
% We only pass the node generation logic and the suspension/fbh indices.

fbh_nodes_idx = 20:23;
susp_nodes_idx = 24:31;


% Initial plotting will just show nodes since topology is adaptive
fprintf('   Created %d nodes. Topology will be dynamically generated during optimization.\n', size(nodes, 1));
fprintf('   Material: Steel (main/front hoops)\n');

%% 3. Check exclusion zones

fprintf('\n3. Checking exclusion zones...\n');

% Check if any elements intersect exclusion zones
% (Simplified check: only check node positions)
n_violations = 0;

% Add dynamic T.1.2.1 Internal Cross Section Template zone (FBH+300mm to FH)
t121_zone = struct();
t121_zone.name = 'T.1.2.1 Internal Cross Section (Path)';
t121_zone.type = 'AABB';
t121_zone.min = [X_fbh + 0.3, -0.175, H_si_lower + 0.05]; % Start after 300mm
t121_zone.max = [X_fh,         0.175, H_si_lower + 0.05 + 0.35]; % Ends at FH
t121_zone.comment = 'Clarified path from FBH+300mm to FH';
% Add to exclusion zones for checking
geom.exclusion_zones(end+1) = t121_zone;

for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);
    min_pt = zone.min(:)';  % Ensure row vector
    max_pt = zone.max(:)';  % Ensure row vector

    % Check each node
    n_nodes = size(nodes, 1);
    for j = 1:n_nodes
        p = nodes(j, :);
        if all(p >= min_pt & p <= max_pt)
            warning('Node %d is inside exclusion zone "%s"', j, zone.name);
            n_violations = n_violations + 1;
        end
    end
end

if n_violations == 0
    fprintf('   ✓ No exclusion zone violations\n');
else
    fprintf('   ⚠ %d exclusion zone violations detected\n', n_violations);
end

%% 4. Generate load cases with FSAE-correct boundary conditions

fprintf('\n4. Generating load cases with FSAE-correct BCs...\n');

% Configure node mapping for this 43-node frame
% --- REPLACED WITH DYNAMIC LOADS INSIDE GA ---
% The static load cases are now generated dynamically per frame inside frameObjectiveFunction.m
% to handle shifting node indices correctly when nodes are added/removed.
bcs_opt = [];
loads_opt = [];

%% 5. Optimization Configuration
fprintf('\n5. Configuring Engine...\n');

config = struct();
config.W_mass = 1.0;
config.W_stiff = 0.5;
config.mass_ref = 22.0;
config.K_min = 500;
config.K_target = 1200;
config.gamma_stiff = 10;
config.alpha_stiff = 2;
config.joint_factor = 0.85;

config.pop_size = 40;
config.max_generations = 10;
config.use_parallel = false;
config.use_custom_ga = true;

reg_params = struct();
reg_params.minimum_aluminum_thickness_welded_mm = 3.0;
reg_params.minimum_aluminum_thickness_unwelded_mm = 2.0;

config.exclusion_zones = {};
for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);
    exclusion = struct();
    if isfield(zone, 'type')
        exclusion.type = zone.type;
    else
        exclusion.type = 'AABB';
    end
    
    if isfield(zone, 'min')
        exclusion.min_bounds = zone.min;
        exclusion.max_bounds = zone.max;
    else
        exclusion.min_bounds = zone.min_bounds;
        exclusion.max_bounds = zone.max_bounds;
    end
    
    if isfield(zone, 'name')
        exclusion.name = zone.name;
    else
        exclusion.name = sprintf('Zone_%d', i);
    end
    
    config.exclusion_zones{end+1} = exclusion;
end

% Pass geometric raw data into config so objective function can regenerate nodes on the fly
config.geom = geom;
config.suspension_file = suspension_file;

% Added property for maximum acceptable Center of Gravity height limit (Z axis)
config.max_cg_z = 0.380; % Max 380mm height from Z=0 origin

%% 6. Run Mixed-Integer Optimization
% loads_opt = load_cases(1).loads; % Handled dynamically
% bcs_opt = load_cases(1).bcs;     % Handled dynamically

% We have 5 pipe catalog groups, 4 shape parameters:
% [H_fh, W_si, H_si_upper, X_si_ratio]
% PLUS 3 distance thresholds: [R_front, R_side, R_rear]
% PLUS 2 dynamic node toggles: [Has_Front_Mid_Nodes, Has_Rear_Mid_Nodes]
% PLUS 4 dynamic node spatial ratios: [Front_X_ratio, Front_Z_ratio, Rear_X_ratio, Rear_Z_ratio]
% Total 18 variables
num_vars = 18;
lb = ones(1, num_vars);
ub = length(catalog) * ones(1, num_vars);

% Custom bounds for Geometry
% H_fh: 0.85m to 1.10m
lb(6) = 0.85; ub(6) = 1.10;
% W_si: 0.25m to 0.45m
lb(7) = 0.25; ub(7) = 0.45;
% H_si_upper: 0.35m to 0.55m
lb(8) = 0.35; ub(8) = 0.55;
% X_si_ratio: 0.25 to 0.75 (Ensures minimum 150mm gap to FH and MH)
lb(9) = 0.25; ub(9) = 0.75;

% Custom bounds for Distance Thresholds (R_front, R_side, R_rear)
lb(10) = 0.40; ub(10) = 0.80;
lb(11) = 0.40; ub(11) = 0.80;
lb(12) = 0.40; ub(12) = 0.80;

% Custom bounds for Node Generation Toggles (0=Off, 1=On)
lb(13) = 0; ub(13) = 1;
lb(14) = 0; ub(14) = 1;

% Custom bounds for Node Spatial Ratios (0.2 to 0.8 to avoid boundary overlap)
lb(15) = 0.2; ub(15) = 0.8; % Front_X_ratio
lb(16) = 0.2; ub(16) = 0.8; % Front_Z_ratio
lb(17) = 0.2; ub(17) = 0.8; % Rear_X_ratio
lb(18) = 0.2; ub(18) = 0.8; % Rear_Z_ratio

% Tell the GA that variables 1:5 (pipes) and 13:14 (toggles) are integers
intcon = [1, 2, 3, 4, 5, 13, 14];

% Force MATLAB built-in GA to handle mixed integer programming
config.use_custom_ga = false;
config.use_parallel = true; % Recommended if parallel computing toolbox is installed

% --- Explicitly Start FEM Server (Centralized Control for Parallelism) ---
fprintf('\n5b. Initializing High-Performance FEM Server...\n');
beam_fem_solver_tcp_java('start');

fprintf('\n6. Running Specialized Custom Mixed-Integer GA...\n');

% Pass Initial Guess (Baseline values) to GA
% [H_fh, W_si, H_si_upper, X_si_ratio]
% X_si_ratio is tricky because it depends on X_fh and X_mh.
% In standard setup: X_si = X_fh + (X_mh - X_fh)*ratio. Let's solve for ratio.
default_X_si_ratio = (0.55 - X_fh) / (X_mh - X_fh); % Baseline target was 0.55m

% Custom GA Options Struct
custom_options = struct();
custom_options.PopulationSize = 60; 
custom_options.MaxGenerations = 40; 
custom_options.UseParallel = config.use_parallel;
custom_options.InitialGuess = [
    5, 5, 5, 5, 5, ...     % Typical pipe indices (baseline)
    H_fh, 0.35, 0.45+Z_floor, default_X_si_ratio, ... % Shape
    0.6, 0.65, 0.6, ... % R_front, R_side, R_rear baseline
    1, 1, ... % [Front Mid Nodes On, Rear Mid Nodes On]
    0.5, 0.5, 0.5, 0.5 ... % [Front X, Front Z, Rear X, Rear Z] ratios
];

% Pass objective function without static indices (handled dynamically inside)
objective_func = @(x) frameObjectiveFunction(x, nodes, bcs_opt, loads_opt, config, catalog, materials, reg_params);
[best_x, best_fval, exitflag, output, population, scores] = custom_mixed_ga(objective_func, num_vars, lb, ub, intcon, custom_options);

% Re-generate final optimized nodes and topology
shape_params = best_x(6:9);
distance_thresholds = best_x(10:12);
node_toggles = best_x(13:14);
node_ratios = best_x(15:18);
[nodes, fbh_nodes, susp_nodes, inner_nodes, fh_x, mh_x, fbh_nodes_idx, susp_nodes_idx] = generate_parametric_nodes(geom, suspension_file, shape_params, node_toggles, node_ratios);

% Regenerate final topology for viewer
elements_geom = generate_adaptive_topology(nodes, fbh_nodes_idx, susp_nodes_idx, fh_x, mh_x, config, distance_thresholds);

%% 7. Visualize Results
fprintf('\n7. Extracted Result & Viewing...\n');

% Force the figure to be brought to the front
fsae_viewer_fig = findall(0, 'Type', 'figure', 'Name', 'FSAE Frame Optimization Dashboard');
if ~isempty(fsae_viewer_fig)
    close(fsae_viewer_fig);
end    

pareto_results = struct();
% Store the generation history
pareto_results.gen_history = 1:custom_options.MaxGenerations;
pareto_results.history_K = output.history_K;
pareto_results.history_Obj = output.history;

% Regenerate final boundary conditions for accurate viewing based on final toggles
node_config = struct();
node_config.rear_lower_nodes  = [9, 10];
node_config.rear_upper_nodes  = [13, 14];
% 1:FL_LF, 2:FR_LF, 3:FL_UF, 4:FR_UF, 5:FL_LR, 6:FR_LR, 7:FL_UR, 8:FR_UR
node_config.front_susp_left_nodes = susp_nodes_idx([1, 3, 5, 7]);
node_config.front_susp_right_nodes = susp_nodes_idx([2, 4, 6, 8]);
final_cases = generate_load_cases(nodes, geom.dimensions.total_mass_kg, 9.81, node_config);
bcs_opt = final_cases(1).bcs;
loads_opt = final_cases(1).loads;

% Pass boundary conditions and loads for rendering
fsae_viewer(nodes, elements_geom, best_x(1:5), catalog, pareto_results, bcs_opt, loads_opt);

fprintf('\n✓ Optimization completed successfully!\n');

% 自動的に画像を保存する
saveas(gcf, 'fsae_optimized_dashboard.png');
fprintf('   Saved optimization dashboard to: fsae_optimized_dashboard.png\n');

% --- Explicitly Stop FEM Server ---
fprintf('\n8. Shutting down FEM Server...\n');
beam_fem_solver_tcp_java('stop');

% 画面が自動で閉じないように、ユーザーがウィンドウを閉じるまで待機する
uiwait(gcf);
