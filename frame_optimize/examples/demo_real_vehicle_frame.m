%% DEMO_REAL_VEHICLE_FRAME - Demonstrate FEM solver with Formula TMU dimensions
%
% This script creates an FSAE-compliant frame using real Formula TMU
% vehicle dimensions with front & rear suspension brackets, evaluated under 3 load cases:
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

fprintf('\n2. Creating real vehicle frame (FSAE-compliant with rear section)...\n');

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
W_mh_mid = W_mh + 0.100; % MH mid-wide: base + 100mm margin (±400mm)
W_mh_top = 0.100; % MH shoulder: ±100mm (narrow taper)
% Frame heights (All relative to ground Z=0)
H_si_lower = 0.05 + Z_floor; % SI lower height
H_si_upper = 0.45 + Z_floor; % SI upper height
H_mh  = 1.35 + Z_floor; % MH height
H_sh  = 1.15 + Z_floor; % Shoulder harness bar height

% Primary X positions (FBH and FH are dynamic, see below)
X_mh  = L;       % Main Hoop (overridden later based on FH + cockpit template)

% FBH and Validation Configuration
FBH_width = 0.400;  % 400mm width
FBH_height = 0.400; % 400mm height
FBH_z_offset_from_cv = -0.050; % User-configurable FBH Z offset from wheel center (Lowered by 50mm)
FH_z_drop_from_fbh = 0.100; % FH is 100mm lower than FBH (offset relative to FBH bottom)
FH_z_rise_from_fbh_top = 0.050; % FH top is 50mm higher than FBH top (configurable limit)

% X-axis offsets relative to suspension geometry (+X is rearward)
FBH_x_shift = 0.030; % Shift FBH 30mm rearward (closer to front suspension mounts)
FH_min_dist_from_susp = 0.100; % Minimum distance from rear-most front suspension mount to FH

% Load real front suspension hardpoints and calculate FBH geometry
base_offset_x = 0.500; % Default forward placement distance 
offset_x = base_offset_x - FBH_x_shift; % Align FBH based on shift (+X direction)
offset_z = FBH_z_offset_from_cv; % Align FBH bottom with absolute suspension geometry + user offset

suspension_file = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));
[fbh_nodes_susp, suspension_mount_nodes_susp, upright_nodes, has_suspension] = load_suspension_hardpoints(suspension_file, offset_x, offset_z, FBH_width, FBH_height);

% Load rear suspension hardpoints
[rear_mount_nodes_raw, rear_upright_nodes, has_rear_suspension] = load_rear_suspension_hardpoints(suspension_file, L);

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

% --- Rear Section Box ---
% Determined by rear suspension mount positions
% Rear susp F mounts at X_rear_f, R mounts at X_rear_r
if has_rear_suspension
    % Find the X ranges of rear suspension mounts
    rear_f_mount_x = min(rear_mount_nodes_raw(1:4, 1));  % F mounts (rows 1-4)
    rear_r_mount_x = max(rear_mount_nodes_raw(5:8, 1));  % R mounts (rows 5-8)
    rear_mount_y_max = max(abs(rear_mount_nodes_raw(:, 2)));
    rear_mount_z_max = max(rear_mount_nodes_raw(:, 3));
    rear_mount_z_min = min(rear_mount_nodes_raw(:, 3));
    
    X_rear_f = rear_f_mount_x - 0.05;  % Front ring 50mm ahead of F mounts
    X_rear_r = rear_r_mount_x + 0.05;  % Rear ring 50mm behind R mounts
    W_rear = W_mh;  % Same width as MH base — mounts protrude outward
    H_rear_upper = rear_mount_z_max + 0.10; % 100mm above highest mount
    H_rear_lower = Z_floor;
    
    fprintf('   Rear section: X = %.3f to %.3f m\n', X_rear_f, X_rear_r);
else
    % Fallback if no rear suspension data
    X_rear_f = X_mh + 0.30;
    X_rear_r = X_mh + 0.60;
    W_rear = 0.250;
    H_rear_upper = 0.35;
    H_rear_lower = Z_floor;
end

% MH mid-wide height: FH upper height - 200mm
H_mh_mid = H_fh - 0.200;

% Nodes: [x, y, z] in meters
nodes = [
    %% Front Hoop (nodes 1-4): X = X_fh
    X_fh, -W_fh, Z_FH_lower; % 1: FH lower left
    X_fh,  W_fh, Z_FH_lower; % 2: FH lower right
    X_fh, -W_fh, H_fh;       % 3: FH upper left
    X_fh,  W_fh, H_fh;       % 4: FH upper right

    %% Side Impact Structure (nodes 5-8)
    X_si, -W_si, H_si_lower;  % 5: SI lower left
    X_si,  W_si, H_si_lower;  % 6: SI lower right
    X_si, -W_si, H_si_upper;  % 7: SI upper left
    X_si,  W_si, H_si_upper;  % 8: SI upper right

    %% Main Hoop Heptagon (nodes 9-15): X = X_mh
    X_mh, -W_mh,     Z_floor;    % 9:  MH base left
    X_mh,  W_mh,     Z_floor;    % 10: MH base right
    X_mh, -W_mh_mid, H_mh_mid;   % 11: MH mid-wide left
    X_mh,  W_mh_mid, H_mh_mid;   % 12: MH mid-wide right
    X_mh, -W_mh_top, H_sh;       % 13: MH shoulder left
    X_mh,  W_mh_top, H_sh;       % 14: MH shoulder right
    X_mh,  0.0,      H_mh;       % 15: MH top center

    %% Rear Section Box — Front Ring (nodes 16-19)
    X_rear_f, -W_rear, H_rear_upper;  % 16: RS front upper left
    X_rear_f,  W_rear, H_rear_upper;  % 17: RS front upper right
    X_rear_f, -W_rear, H_rear_lower;  % 18: RS front lower left
    X_rear_f,  W_rear, H_rear_lower;  % 19: RS front lower right

    %% Rear Section Box — Rear Ring (nodes 20-23)
    X_rear_r, -W_rear, H_rear_upper;  % 20: RS rear upper left
    X_rear_r,  W_rear, H_rear_upper;  % 21: RS rear upper right
    X_rear_r, -W_rear, H_rear_lower;  % 22: RS rear lower left
    X_rear_r,  W_rear, H_rear_lower;  % 23: RS rear lower right

    %% Front Bulkhead (nodes 24-27)
    fbh_nodes;
    suspension_mount_nodes; % Nodes 28-35 (Front suspension mounts)
];

% Append front inner brackets (Nodes 36-43)
nodes = [nodes; inner_bracket_nodes];

% --- Rear Suspension Brackets ---
rear_inner_bracket_nodes = zeros(8, 3);
if has_rear_suspension
    % Append rear suspension mount nodes (Nodes 44-51)
    nodes = [nodes; rear_mount_nodes_raw];
    rear_susp_start_idx = 44;
    fprintf('   Added 8 rear suspension mount points (Nodes 44-51).\n');
    
    % Generate rear inner bracket nodes (Nodes 52-59)
    % Same pattern as front: extend 50mm along A-arm axis into chassis
    L_bracket_rear = 0.050;
    rear_upright_map = [1, 2, 3, 4, 1, 2, 3, 4]; % Map to upright L/R Lower/Upper
    
    for i = 1:8
        mount_pt = rear_mount_nodes_raw(i, :);
        upright_pt = rear_upright_nodes(rear_upright_map(i), :);
        
        % Vector from upright to mount (pointing into chassis)
        arm_vec = mount_pt - upright_pt;
        unit_vec = arm_vec / norm(arm_vec);
        
        % Place inner bracket 50mm along this axis from mount
        rear_inner_bracket_nodes(i, :) = mount_pt + unit_vec * L_bracket_rear;
    end
    
    nodes = [nodes; rear_inner_bracket_nodes]; % Nodes 52-59
    fprintf('   Added 8 rear inner bracket nodes (Nodes 52-59).\n');
else
    % Pad with zeros if no rear suspension
    nodes = [nodes; zeros(8, 3)]; % 44-51 placeholder
    nodes = [nodes; zeros(8, 3)]; % 52-59 placeholder
end

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
%%  Element definitions — real FSAE frame topology
%%  Follows reference CAD: FBH→FBH Support→FH→SI→MH→MH Brace
%% ============================================================

elements = [
    %% --- Front Bulkhead (FBH) --- nodes 24-27, closed rectangle, Size B
    24, 25, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH bottom
    25, 27, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH right
    27, 26, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH top
    26, 24, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH left

    %% --- FBH Support --- FBH→FH triangulation, Size B
    % Lower supports (FBH lower → FH lower)
    24,  1, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LL → FH LL
    25,  2, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LR → FH LR
    % Upper supports (FBH upper → FH upper)
    26,  3, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH UL → FH UL
    27,  4, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH UR → FH UR
    % Diagonal braces (triangulation)
    24,  3, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LL → FH UL
    25,  4, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LR → FH UR

    %% --- Front Hoop (FH) --- nodes 1-4, continuous steel tube, Size A
    1,  2, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % FH bottom
    2,  4, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % FH right
    4,  3, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % FH top
    3,  1, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % FH left

    %% --- Side Impact Structure (SI) --- nodes 5-8, Size B minimum
    %% FH→SI lower rails (left/right)
     1,  5, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LL → SI LL
     2,  6, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LR → SI LR
    %% FH→SI upper rails
     3,  7, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH UL → SI UL
     4,  8, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH UR → SI UR
    %% SI verticals
     5,  7, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI left vertical
     6,  8, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI right vertical
    %% SI diagonals (triangulation for stiffness)
     1,  7, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LL → SI UL (X-brace L)
     2,  8, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LR → SI UR (X-brace R)

    %% --- SI → MH lower rails ---
     5,  9, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI LL → MH LL
     6, 10, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI LR → MH LR
    %% SI → MH upper rails
     7, 11, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI UL → MH UL
     8, 12, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI UR → MH UR
    %% SI→MH diagonals (rear side panel triangulation)
     5, 11, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI LL → MH mid L
     6, 12, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % SI LR → MH mid R

    %% --- Main Hoop Heptagon (MH) --- nodes 9-15, continuous steel tube, Size A
     9, 10, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH bottom
    10, 12, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH right: base→mid
    12, 14, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH right: mid→shoulder
    14, 15, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH right: shoulder→top
    15, 13, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH left: top→shoulder
    13, 11, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH left: shoulder→mid
    11,  9, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % MH left: mid→base

    %% --- MH → Rear Section Front Ring --- nodes 9-15 → 16-19
    % MH shoulders to RS front ring upper
    13, 16, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH SL → RS FUL
    14, 17, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH SR → RS FUR
    % MH base to RS front ring lower
     9, 18, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH BL → RS FLL
    10, 19, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH BR → RS FLR
    % MH diagonal braces to RS front ring (triangulation)
     9, 16, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH BL → RS FUL
    10, 17, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % MH BR → RS FUR

    %% --- Rear Section Front Ring (16-19) ---
    16, 17, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS front cross upper
    18, 19, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS front cross lower
    16, 18, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS front left vertical
    17, 19, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS front right vertical

    %% --- Rear Section Rear Ring (20-23) ---
    20, 21, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS rear cross upper
    22, 23, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS rear cross lower
    20, 22, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS rear left vertical
    21, 23, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS rear right vertical

    %% --- Rear Section Rails (Front Ring → Rear Ring) ---
    16, 20, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS upper left rail
    17, 21, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS upper right rail
    18, 22, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS lower left rail
    19, 23, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS lower right rail
    % Diagonals for torsional stiffness (X-brace on sides)
    16, 22, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS left X-brace
    17, 23, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS right X-brace
    18, 20, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS floor-to-top left diag
    19, 21, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % RS floor-to-top right diag

    %% --- Shoulder Harness Bar --- nodes 13-14, steel Size A
    13, 14, D_steel_A, t_steel_A, E_steel, G_steel, rho_steel, shape, jf;  % SH bar


    %% --- Floor cross-braces (triangulate floor plane) ---
     1,  6, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % Floor diagonal FH-SI
     5, 10, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % Floor diagonal SI-MH

    %% --- Front Suspension Mount Brackets (Size B pipes) ---
    % Mounts (28-35) → Inner brackets (36-43)
    28, 36, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FL Lower F Mount -> Inner
    29, 37, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FR Lower F Mount -> Inner
    30, 38, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FL Upper F Mount -> Inner
    31, 39, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FR Upper F Mount -> Inner
    32, 40, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FL Lower R Mount -> Inner
    33, 41, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FR Lower R Mount -> Inner
    34, 42, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FL Upper R Mount -> Inner
    35, 43, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FR Upper R Mount -> Inner
    
    % Inner brackets (36-39) → FBH (24-27)
    24, 36, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LL -> Inner FL Lower F
    25, 37, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH LR -> Inner FR Lower F
    26, 38, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH UL -> Inner FL Upper F
    27, 39, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FBH UR -> Inner FR Upper F
    
    % Inner brackets (40-43) → FH (1-4)
     1, 40, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LL -> Inner FL Lower R
     2, 41, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH LR -> Inner FR Lower R
     3, 42, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH UL -> Inner FL Upper R
     4, 43, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf;  % FH UR -> Inner FR Upper R
];

%% --- Rear Suspension Mount Brackets (added dynamically) ---
if has_rear_suspension
    rear_susp_elems = [];
    % Rear mounts (44-51) → Rear inner brackets (52-59)
    for i = 1:8
        rear_susp_elems = [rear_susp_elems;
            43+i, 51+i, D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf];
    end
    
    % Rear inner brackets → nearest rear section ring nodes
    % F-brackets (52-55) connect to front ring (16-19)
    % R-brackets (56-59) connect to rear ring (20-23)
    % Mapping: bracket idx 1,2=lower L/R → ring lower L/R; 3,4=upper L/R → ring upper L/R
    f_ring_targets = [18, 19, 16, 17]; % lower L/R, upper L/R
    r_ring_targets = [22, 23, 20, 21]; % lower L/R, upper L/R
    
    for i = 1:4
        rear_susp_elems = [rear_susp_elems;
            51+i, f_ring_targets(i), D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf];
    end
    for i = 1:4
        rear_susp_elems = [rear_susp_elems;
            55+i, r_ring_targets(i), D_steel_B, t_steel_B, E_steel, G_steel, rho_steel, shape, jf];
    end
    
    elements = [elements; rear_susp_elems];
end

n_nodes = size(nodes, 1);
n_elems = size(elements, 1);

% --- Calculate Frame Mass and Center of Gravity (CG) ---
total_frame_mass = 0;
cg_moment = [0, 0, 0];

for i = 1:n_elems
    n1 = nodes(elements(i, 1), :);
    n2 = nodes(elements(i, 2), :);
    
    L_elem = norm(n2 - n1);
    mid_pt = (n1 + n2) / 2;
    
    D_out = elements(i, 3);
    t_wall = elements(i, 4);
    rho = elements(i, 7);
    
    area = pi * ((D_out/2)^2 - (D_out/2 - t_wall)^2);
    mass_elem = L_elem * area * rho;
    
    total_frame_mass = total_frame_mass + mass_elem;
    cg_moment = cg_moment + (mass_elem * mid_pt);
end

CG_frame = cg_moment / total_frame_mass;

fprintf('   Created %d nodes, %d elements\n', n_nodes, n_elems);
fprintf('   Material: Steel (main/front hoops), Aluminum (secondary)\n');
fprintf('   Frame Mass: %.2f kg\n', total_frame_mass);
fprintf('   Frame CG:   X=%.3f, Y=%.3f, Z=%.3f m\n', CG_frame(1), CG_frame(2), CG_frame(3));

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

% Configure node mapping for this frame
node_config = struct();

% Rear suspension: use actual rear mount nodes if available, otherwise MH lower
if has_rear_suspension
    % Rear suspension mounts: 44=RL_LF, 45=RR_LF, 46=RL_UF, 47=RR_UF
    %                         48=RL_LR, 49=RR_LR, 50=RL_UR, 51=RR_UR
    node_config.rear_susp_left_nodes  = [44, 46, 48, 50]; % RL mounts
    node_config.rear_susp_right_nodes = [45, 47, 49, 51]; % RR mounts
    node_config.rear_lower_nodes  = [44, 45];   % Use rear susp lower F mounts
    node_config.rear_upper_nodes  = [46, 47];   % Use rear susp upper F mounts
else
    node_config.rear_lower_nodes  = [9, 10];    % Fallback: MH base
    node_config.rear_upper_nodes  = [13, 14];   % Fallback: MH shoulder
end

% Front suspension mounts (left/right)
node_config.front_susp_left_nodes = [28, 30, 32, 34];
node_config.front_susp_right_nodes = [29, 31, 33, 35];
node_config.total_mass_kg = geom.dimensions.total_mass_kg;

load_cases = generate_load_cases(nodes, geom.dimensions.total_mass_kg, 9.81, node_config);

fprintf('   Generated %d load cases:\n', length(load_cases));
for i = 1:length(load_cases)
    fprintf('     %d. %s: %s\n', i, load_cases(i).name, load_cases(i).description);
    fprintf('        BCs: %d constraints, Loads: %d forces\n', ...
            size(load_cases(i).bcs, 1), size(load_cases(i).loads, 1));
end

%% 5. Run FEM solver for all load cases (TCP version)

fprintf('\n5. Running FEM solver (TCP version) for all load cases...\n');

results = cell(length(load_cases), 1);
times = zeros(length(load_cases), 1);

for i = 1:length(load_cases)
    fprintf('   Case %d/%d: %s...\n', i, length(load_cases), load_cases(i).name);

    % Each load case has its own boundary conditions
    case_bcs = load_cases(i).bcs;
    case_loads = load_cases(i).loads;

    tic;
    results{i} = beam_fem_solver_tcp_java(nodes, elements, case_bcs, case_loads);
    times(i) = toc;

    fprintf('     ✓ Completed in %.2f ms\n', times(i) * 1000);
end

% Shutdown TCP server
beam_fem_solver_tcp_java('shutdown');

fprintf('   Average solve time: %.2f ms\n', mean(times) * 1000);

%% 6. Display results

fprintf('\n=== RESULTS ===\n\n');

% Display frame properties (from first load case)
fprintf('Frame Properties:\n');
fprintf('  Mass:               %.2f kg\n', results{1}.mass);
fprintf('  Torsional rigidity: %.1f Nm/deg\n', results{1}.K_torsion);

% Check if mass is reasonable
if results{1}.mass < 15 || results{1}.mass > 35
    warning('Mass %.2f kg is outside expected range (15-35 kg)', results{1}.mass);
else
    fprintf('  ✓ Mass is within expected range\n');
end

% Check if torsional rigidity is reasonable
% Note: Torsional rigidity depends heavily on frame design and boundary conditions
if results{1}.K_torsion < 200 || results{1}.K_torsion > 2000
    warning('Torsional rigidity %.1f Nm/deg is outside expected range (200-2000)', ...
            results{1}.K_torsion);
else
    fprintf('  ✓ Torsional rigidity is within expected range\n');
end

fprintf('\nLoad Case Results:\n');
for i = 1:length(load_cases)
    fprintf('  %d. %s:\n', i, load_cases(i).name);
    fprintf('     Max displacement: %.3f mm\n', max(abs(results{i}.U(:))) * 1000);
    if isfield(results{i}, 'stress')
        fprintf('     Max stress:       %.1f MPa\n', max(abs(results{i}.stress(:))));
    end
end

%% 7. Visualize frame

fprintf('\n7. Visualizing frame...\n');

figure('Name', 'Formula TMU Real Vehicle Frame', 'Position', [100, 100, 1400, 800]);

% Plot 1: (REMOVED - Moving geometry to isolated detail window)
% The dashboard now starts with load case results in subplot 1-3.

% Plot 1-3: Deformed shapes for each load case
for i = 1:min(3, length(load_cases))
    subplot(2, 3, i); % Use top row for load cases
    hold on; axis equal; grid on;
    xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
    title(sprintf('Case %d: %s (1000×)', i, load_cases(i).name));

    scale = 1000;  % Magnification factor

    % Plot original (faint)
    for j = 1:n_elems
        node_i = elements(j, 1);
        node_j = elements(j, 2);
        p1 = nodes(node_i, :);
        p2 = nodes(node_j, :);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'Color', [0.8 0.8 0.8], 'LineWidth', 0.5);
    end

    % Plot deformed
    for j = 1:n_elems
        node_i = elements(j, 1);
        node_j = elements(j, 2);
        p1 = nodes(node_i, :) + scale * results{i}.U(node_i, 1:3);
        p2 = nodes(node_j, :) + scale * results{i}.U(node_j, 1:3);
        plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], ...
              'r-', 'LineWidth', 2);
    end
    
    % Vizualize applied loads as distinct arrows pointing to the nodes
    if isfield(load_cases(i), 'loads') && ~isempty(load_cases(i).loads)
        case_loads = load_cases(i).loads;  % [node_idx, dof, force]
        max_f = max(abs(case_loads(:, 3)));
        if max_f > 0
            arrow_scale = 0.3 / max_f; % Scale arrow length visually (~0.3m max)
            for k = 1:size(case_loads, 1)
                n_idx = case_loads(k, 1);
                dof = case_loads(k, 2);
                f_val = case_loads(k, 3);
                
                start_pt = nodes(n_idx, :) + scale * results{i}.U(n_idx, 1:3);
                vec = [0, 0, 0];
                if dof >= 1 && dof <= 3
                    vec(dof) = f_val * arrow_scale;
                end
                
                % Draw arrow pointing to the node
                quiver3(start_pt(1)-vec(1), start_pt(2)-vec(2), start_pt(3)-vec(3), ...
                        vec(1), vec(2), vec(3), 0, 'Color', 'm', 'LineWidth', 2, 'MaxHeadSize', 2.0);
                
                % Highlight loaded node specifically
                scatter3(start_pt(1), start_pt(2), start_pt(3), 60, 'm', 'filled', 'o', 'MarkerEdgeColor', 'k');
            end
        end
    end
    
    % Visualize A-Arms if suspension is available
    if exist('has_suspension', 'var') && has_suspension && exist('upright_nodes', 'var')
        % We map the deformed chassis mount coordinates for the lines
        get_def_node = @(idx) nodes(idx, :) + scale * results{i}.U(idx, 1:3);
        
        mlf_fl = get_def_node(29); muf_fl = get_def_node(31); mlr_fl = get_def_node(33); mur_fl = get_def_node(35);
        mlf_fr = get_def_node(30); muf_fr = get_def_node(32); mlr_fr = get_def_node(34); mur_fr = get_def_node(36);
        
        % Uprights average deflection
        avg_def_fl = mean([results{i}.U(29, 1:3); results{i}.U(31, 1:3); results{i}.U(33, 1:3); results{i}.U(35, 1:3)], 1) * scale;
        avg_def_fr = mean([results{i}.U(30, 1:3); results{i}.U(32, 1:3); results{i}.U(34, 1:3); results{i}.U(36, 1:3)], 1) * scale;
        
        upl_fl = upright_nodes(1, :) + avg_def_fl; upu_fl = upright_nodes(3, :) + avg_def_fl;
        upl_fr = upright_nodes(2, :) + avg_def_fr; upu_fr = upright_nodes(4, :) + avg_def_fr;
        
        % Draw A-Arms FL
        plot3([mlf_fl(1), upl_fl(1)], [mlf_fl(2), upl_fl(2)], [mlf_fl(3), upl_fl(3)], 'color', [1 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([mlr_fl(1), upl_fl(1)], [mlr_fl(2), upl_fl(2)], [mlr_fl(3), upl_fl(3)], 'color', [1 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([muf_fl(1), upu_fl(1)], [muf_fl(2), upu_fl(2)], [muf_fl(3), upu_fl(3)], 'color', [0.8 0.8 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([mur_fl(1), upu_fl(1)], [mur_fl(2), upu_fl(2)], [mur_fl(3), upu_fl(3)], 'color', [0.8 0.8 0], 'LineWidth', 1.5, 'LineStyle', '--');
        
        % Draw A-Arms FR
        plot3([mlf_fr(1), upl_fr(1)], [mlf_fr(2), upl_fr(2)], [mlf_fr(3), upl_fr(3)], 'color', [1 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([mlr_fr(1), upl_fr(1)], [mlr_fr(2), upl_fr(2)], [mlr_fr(3), upl_fr(3)], 'color', [1 0.5 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([muf_fr(1), upu_fr(1)], [muf_fr(2), upu_fr(2)], [muf_fr(3), upu_fr(3)], 'color', [0.8 0.8 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([mur_fr(1), upu_fr(1)], [mur_fr(2), upu_fr(2)], [mur_fr(3), upu_fr(3)], 'color', [0.8 0.8 0], 'LineWidth', 1.5, 'LineStyle', '--');
        
        % Upright to Wheel Center Line
        plot3([upl_fl(1), upu_fl(1)], [upl_fl(2), upu_fl(2)], [upl_fl(3), upu_fl(3)], 'k-', 'LineWidth', 2);
        plot3([upl_fr(1), upu_fr(1)], [upl_fr(2), upu_fr(2)], [upl_fr(3), upu_fr(3)], 'k-', 'LineWidth', 2);
    end
    
    % Visualize Rear A-Arms if rear suspension is available
    if exist('has_rear_suspension', 'var') && has_rear_suspension && exist('rear_upright_nodes', 'var')
        get_def_node = @(idx) nodes(idx, :) + scale * results{i}.U(idx, 1:3);
        
        % Rear mount nodes: 44=RL_LF, 45=RR_LF, 46=RL_UF, 47=RR_UF
        %                    48=RL_LR, 49=RR_LR, 50=RL_UR, 51=RR_UR
        % Rear upright nodes: 1=RL_Lower, 2=RR_Lower, 3=RL_Upper, 4=RR_Upper
        
        % Average deflection for rear uprights
        avg_def_rl = mean([results{i}.U(44, 1:3); results{i}.U(46, 1:3); results{i}.U(48, 1:3); results{i}.U(50, 1:3)], 1) * scale;
        avg_def_rr = mean([results{i}.U(45, 1:3); results{i}.U(47, 1:3); results{i}.U(49, 1:3); results{i}.U(51, 1:3)], 1) * scale;
        
        rup_l_rl = rear_upright_nodes(1, :) + avg_def_rl;
        rup_u_rl = rear_upright_nodes(3, :) + avg_def_rl;
        rup_l_rr = rear_upright_nodes(2, :) + avg_def_rr;
        rup_u_rr = rear_upright_nodes(4, :) + avg_def_rr;
        
        % Draw Rear A-Arms (Lower in green, Upper in dark green)
        % Point-by-point drawing
        rl_lf = get_def_node(44); rl_uf = get_def_node(46);
        rl_lr = get_def_node(48); rl_ur = get_def_node(50);
        rr_lf = get_def_node(45); rr_uf = get_def_node(47);
        rr_lr = get_def_node(49); rr_ur = get_def_node(51);
        
        % RL lower arms
        plot3([rl_lf(1), rup_l_rl(1)], [rl_lf(2), rup_l_rl(2)], [rl_lf(3), rup_l_rl(3)], 'color', [0 0.8 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([rl_lr(1), rup_l_rl(1)], [rl_lr(2), rup_l_rl(2)], [rl_lr(3), rup_l_rl(3)], 'color', [0 0.8 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
        % RL upper arms
        plot3([rl_uf(1), rup_u_rl(1)], [rl_uf(2), rup_u_rl(2)], [rl_uf(3), rup_u_rl(3)], 'color', [0.2 0.6 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([rl_ur(1), rup_u_rl(1)], [rl_ur(2), rup_u_rl(2)], [rl_ur(3), rup_u_rl(3)], 'color', [0.2 0.6 0], 'LineWidth', 1.5, 'LineStyle', '--');
        % RR lower arms
        plot3([rr_lf(1), rup_l_rr(1)], [rr_lf(2), rup_l_rr(2)], [rr_lf(3), rup_l_rr(3)], 'color', [0 0.8 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([rr_lr(1), rup_l_rr(1)], [rr_lr(2), rup_l_rr(2)], [rr_lr(3), rup_l_rr(3)], 'color', [0 0.8 0.2], 'LineWidth', 1.5, 'LineStyle', '--');
        % RR upper arms
        plot3([rr_uf(1), rup_u_rr(1)], [rr_uf(2), rup_u_rr(2)], [rr_uf(3), rup_u_rr(3)], 'color', [0.2 0.6 0], 'LineWidth', 1.5, 'LineStyle', '--');
        plot3([rr_ur(1), rup_u_rr(1)], [rr_ur(2), rup_u_rr(2)], [rr_ur(3), rup_u_rr(3)], 'color', [0.2 0.6 0], 'LineWidth', 1.5, 'LineStyle', '--');
        
        % Rear upright kingpin lines
        plot3([rup_l_rl(1), rup_u_rl(1)], [rup_l_rl(2), rup_u_rl(2)], [rup_l_rl(3), rup_u_rl(3)], 'k-', 'LineWidth', 2);
        plot3([rup_l_rr(1), rup_u_rr(1)], [rup_l_rr(2), rup_u_rr(2)], [rup_l_rr(3), rup_u_rr(3)], 'k-', 'LineWidth', 2);
    end

    view(3); rotate3d on;
end

% Plot 4: Mass breakdown
subplot(2, 3, 4);
steel_mass = 0;
alum_mass = 0;
for i = 1:n_elems
    node_i = elements(i, 1);
    node_j = elements(i, 2);
    L_elem = norm(nodes(node_j, :) - nodes(node_i, :));
    D = elements(i, 3) / 1000;  % mm to m
    t = elements(i, 4) / 1000;
    rho = elements(i, 7);
    A = pi * (D^2 - (D-2*t)^2) / 4;
    m = A * L_elem * rho;

    if elements(i, 5) == E_steel
        steel_mass = steel_mass + m;
    else
        alum_mass = alum_mass + m;
    end
end

bar([steel_mass, alum_mass]);
set(gca, 'XTickLabel', {'Steel', 'Aluminum'});
ylabel('Mass [kg]');
title('Material Breakdown');
grid on;

% Plot 6: Performance summary
subplot(2, 3, 6);
axis off;
text(0.1, 0.9, 'Performance Summary', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.75, sprintf('Total mass: %.2f kg', results{1}.mass), 'FontSize', 11);
text(0.1, 0.65, sprintf('Steel: %.2f kg (%.0f%%)', steel_mass, ...
                        100*steel_mass/results{1}.mass), 'FontSize', 10);
text(0.1, 0.55, sprintf('Aluminum: %.2f kg (%.0f%%)', alum_mass, ...
                        100*alum_mass/results{1}.mass), 'FontSize', 10);
text(0.1, 0.40, sprintf('Torsional rigidity: %.1f Nm/deg', results{1}.K_torsion), ...
     'FontSize', 11);
text(0.1, 0.25, sprintf('Avg solve time: %.2f ms', mean(times)*1000), 'FontSize', 11);
text(0.1, 0.10, sprintf('Nodes: %d, Elements: %d', n_nodes, n_elems), 'FontSize', 10);

%% 8. Detail Window (Isolated)
figure('Name', 'Frame Geometry - Detail View', 'Position', [150, 150, 1000, 800]);
hold on; axis equal; grid on;
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
title('Frame Geometry (Detail View)');

% Plot elements (color by material)
for i = 1:n_elems
    node_i = elements(i, 1);
    node_j = elements(i, 2);
    p1 = nodes(node_i, :);
    p2 = nodes(node_j, :);
    plot3([p1(1), p2(1)], [p1(2), p2(2)], [p1(3), p2(3)], 'Color', 'b', 'LineWidth', 2.5);
end

% Plot nodes with labels
scatter3(nodes(:,1), nodes(:,2), nodes(:,3), 40, 'k', 'filled');
for i = 1:n_nodes
    text(nodes(i,1), nodes(i,2), nodes(i,3)+0.05, sprintf('%d', i), 'FontSize', 9);
end

% Mark exclusion zones
for i = 1:length(geom.exclusion_zones)
    zone = geom.exclusion_zones(i);
    min_pt = zone.min; max_pt = zone.max;
    corners = [min_pt(1), min_pt(2), min_pt(3); max_pt(1), min_pt(2), min_pt(3); max_pt(1), max_pt(2), min_pt(3); min_pt(1), max_pt(2), min_pt(3);
               min_pt(1), min_pt(2), max_pt(3); max_pt(1), min_pt(2), max_pt(3); max_pt(1), max_pt(2), max_pt(3); min_pt(1), max_pt(2), max_pt(3)];
    plot3(corners([1,2,3,4,1],1), corners([1,2,3,4,1],2), corners([1,2,3,4,1],3), 'Color', [1 0.5 0], 'LineWidth', 1, 'LineStyle', '--');
    plot3(corners([5,6,7,8,5],1), corners([5,6,7,8,5],2), corners([5,6,7,8,5],3), 'Color', [1 0.5 0], 'LineWidth', 1, 'LineStyle', '--');
end

% Templates in detail view
tpl1_z = H_fh;
tpl1_x_front = X_fh;
tpl1_x_rear = X_fh + 0.60;
tpl1_w_front = 0.40 / 2;
tpl1_w_max = 0.55 / 2;
tpl1_x_max = X_fh + 0.50; 

tpl1_pts = [
    tpl1_x_front, -tpl1_w_front, tpl1_z;
    tpl1_x_max,   -tpl1_w_max,   tpl1_z;
    tpl1_x_rear,  -tpl1_w_max,   tpl1_z;
    tpl1_x_rear,   tpl1_w_max,   tpl1_z;
    tpl1_x_max,    tpl1_w_max,   tpl1_z;
    tpl1_x_front,  tpl1_w_front, tpl1_z;
    tpl1_x_front, -tpl1_w_front, tpl1_z;
];
plot3(tpl1_pts(:,1), tpl1_pts(:,2), tpl1_pts(:,3), 'm-', 'LineWidth', 2.5);

tpl2_w = 0.35 / 2;
tpl2_h_min = H_si_lower + 0.05; 
tpl2_h_max = tpl2_h_min + 0.35;

% Draw swept volume from FBH + 300mm to FH
x_span = [X_fbh + 0.3, X_fh];
for x = x_span
    tpl2_pts = [x, -tpl2_w, tpl2_h_min; x, tpl2_w, tpl2_h_min; x, tpl2_w, tpl2_h_max; x, -tpl2_w, tpl2_h_max; x, -tpl2_w, tpl2_h_min];
    plot3(tpl2_pts(:,1), tpl2_pts(:,2), tpl2_pts(:,3), 'g-', 'LineWidth', 2.0);
end
% Longitudinal lines connecting corners
for y = [-tpl2_w, tpl2_w]
    for z = [tpl2_h_min, tpl2_h_max]
        plot3([X_fbh + 0.3, X_fh], [y, y], [z, z], 'g--', 'LineWidth', 1.0);
    end
end
text((X_fbh+0.3+X_fh)/2, 0, tpl2_h_max+0.05, 'T.1.2.1 FBH+300mm to FH Path', 'Color', [0 0.5 0], 'FontSize', 9, 'FontWeight', 'bold');

% Plot Center of Gravity (CG)
plot3(CG_frame(1), CG_frame(2), CG_frame(3), 'p', 'MarkerSize', 15, 'MarkerFaceColor', 'm', 'MarkerEdgeColor', 'k');
text(CG_frame(1), CG_frame(2), CG_frame(3) + 0.10, sprintf('CG (%.2fkg)', total_frame_mass), 'Color', 'm', 'FontSize', 10, 'FontWeight', 'bold');

view(3); rotate3d on;
legend('Steel Tube', 'Nodes', 'Templates', 'Center of Gravity', 'Location', 'best');

fprintf('\n✓ Demo completed successfully!\n');
fprintf('\nNext steps:\n');
fprintf('  1. Run optimization: demo_full_pipeline.m\n');
fprintf('  2. Adjust exclusion zones in real_vehicle_geometry.json\n');
fprintf('  3. Modify node positions for specific design\n');
