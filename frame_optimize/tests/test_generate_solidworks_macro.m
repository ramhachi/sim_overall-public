function test_generate_solidworks_macro()
% TEST_GENERATE_SOLIDWORKS_MACRO - Verify generate_solidworks_macro.m
%
% Tests:
%   1. MATLAB syntax — function runs without error on synthetic data
%   2. Output .bas file exists and is non-empty
%   3. VBA syntax sanity — balanced Sub/End Sub, no Dim inside loops
%   4. Node data — correct count and coordinate transform
%   5. Element classification — all groups populated correctly
%   6. Generated VBA contains required sections
%
% Usage:
%   cd <project_root>/frame_optimize
%   addpath('matlab'); addpath('tests');
%   test_generate_solidworks_macro

fprintf('=======================================================\n');
fprintf(' Test: generate_solidworks_macro\n');
fprintf('=======================================================\n');

% --- Setup paths ---
this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir);  % frame_optimize/
matlab_dir = fullfile(root_dir, 'matlab');
data_dir = fullfile(root_dir, 'data');
addpath(matlab_dir);

% Temp output directory
test_output_dir = fullfile(tempdir, 'test_sw_macro');
if isfolder(test_output_dir)
    rmdir(test_output_dir, 's');
end
mkdir(test_output_dir);

% --- Load real data ---
fprintf('\n[Setup] Loading materials and pipe catalog...\n');
materials = loadMaterials(fullfile(data_dir, 'materials.json'));
catalog = loadPipeCatalog(fullfile(data_dir, 'pipe_catalog.json'), materials);
fprintf('  Loaded %d pipes\n', length(catalog));

% --- Create synthetic frame data (simplified demo) ---
fprintf('[Setup] Creating synthetic frame data...\n');

% Simplified 59-node frame (matches generate_parametric_nodes output)
nodes = zeros(59, 3);

% Front Hoop (1-4)
X_fh = 0.0;
W_fh = 0.250;
H_fh = 0.95;
Z_floor = 0.050;
nodes(1,:) = [X_fh, -W_fh, Z_floor];
nodes(2,:) = [X_fh,  W_fh, Z_floor];
nodes(3,:) = [X_fh, -W_fh, H_fh];
nodes(4,:) = [X_fh,  W_fh, H_fh];

% Side Impact (5-8)
X_si = 0.35;
W_si = 0.300;
nodes(5,:) = [X_si, -W_si, 0.05+Z_floor];
nodes(6,:) = [X_si,  W_si, 0.05+Z_floor];
nodes(7,:) = [X_si, -W_si, 0.45+Z_floor];
nodes(8,:) = [X_si,  W_si, 0.45+Z_floor];

% Main Hoop (9-15) — heptagon
X_mh = 1.55;
W_mh = 0.300;
nodes(9,:)  = [X_mh, -W_mh, Z_floor];
nodes(10,:) = [X_mh,  W_mh, Z_floor];
nodes(11,:) = [X_mh, -0.400, 0.80];
nodes(12,:) = [X_mh,  0.400, 0.80];
nodes(13,:) = [X_mh, -0.100, 1.15];
nodes(14,:) = [X_mh,  0.100, 1.15];
nodes(15,:) = [X_mh,  0.000, 1.40];

% Rear Section (16-23)
X_rf = 1.85; X_rr = 2.15;
W_rear = 0.250;
nodes(16,:) = [X_rf, -W_rear, 0.35];
nodes(17,:) = [X_rf,  W_rear, 0.35];
nodes(18,:) = [X_rf, -W_rear, Z_floor];
nodes(19,:) = [X_rf,  W_rear, Z_floor];
nodes(20,:) = [X_rr, -W_rear, 0.35];
nodes(21,:) = [X_rr,  W_rear, 0.35];
nodes(22,:) = [X_rr, -W_rear, Z_floor];
nodes(23,:) = [X_rr,  W_rear, Z_floor];

% FBH (24-27)
X_fbh = -0.30;
W_fbh = 0.150;
H_fbh = 0.25;
nodes(24,:) = [X_fbh, -W_fbh, Z_floor];
nodes(25,:) = [X_fbh,  W_fbh, Z_floor];
nodes(26,:) = [X_fbh, -W_fbh, Z_floor+H_fbh];
nodes(27,:) = [X_fbh,  W_fbh, Z_floor+H_fbh];

% Front suspension mounts (28-35) — dummy positions
for i = 28:35
    nodes(i,:) = [X_fbh-0.05, (-1)^i * 0.20, 0.10 + 0.05*mod(i,4)];
end
% Front inner brackets (36-43)
for i = 36:43
    nodes(i,:) = nodes(i-8,:) + [0.05, 0, 0];
end
% Rear suspension mounts (44-51)
for i = 44:51
    nodes(i,:) = [X_rr+0.05, (-1)^i * 0.25, 0.10 + 0.05*mod(i,4)];
end
% Rear inner brackets (52-59)
for i = 52:59
    nodes(i,:) = nodes(i-8,:) + [-0.05, 0, 0];
end

% --- Build elements_geom struct ---
% group assignments: 1=FH, 2=MH, 3=SI, 4=structural, 5=shoulder
connectivity = [
    % Group 1: Front Hoop
    1  2;  % FH bottom
    2  4;  % FH right
    4  3;  % FH top
    3  1;  % FH left
    % Group 2: Main Hoop
    9 10;  % MH bottom
    10 12; % MH right base→mid
    12 14; % MH right mid→shoulder
    14 15; % MH right shoulder→top
    15 13; % MH left top→shoulder
    13 11; % MH left shoulder→mid
    11  9; % MH left mid→base
    % Group 3: Side Impact
    1  5; 2  6; 3  7; 4  8;
    5  7; 6  8;
    1  7; 2  8;
    % Group 4: Structural (FBH, FBH support, Rear, Brackets)
    24 25; 25 27; 27 26; 26 24;           % FBH rectangle
    24  1; 25  2; 26  3; 27  4;           % FBH support
    24  3; 25  4;                         % FBH support diagonal
    13 16; 14 17; 9 18; 10 19;            % MH→rear
    16 17; 18 19; 16 18; 17 19;           % Rear front ring
    20 21; 22 23; 20 22; 21 23;           % Rear rear ring
    16 20; 17 21; 18 22; 19 23;           % Rear rails
    28 36; 29 37; 30 38; 31 39;           % Front susp brackets
    44 52; 45 53; 46 54; 47 55;           % Rear susp brackets
    ];
group_indices = [
    1*ones(4,1);    % FH (4 elements)
    2*ones(7,1);    % MH (7 elements)
    3*ones(8,1);    % SI (8 elements)
    4*ones(4,1);    % FBH
    4*ones(6,1);    % FBH support
    4*ones(4,1);    % MH→rear
    4*ones(4,1);    % Rear front ring
    4*ones(4,1);    % Rear rear ring
    4*ones(4,1);    % Rear rails
    4*ones(4,1);    % Front susp brackets
    4*ones(4,1);    % Rear susp brackets
    ];

% Shoulder harness bar
connectivity = [connectivity; 13 14];
group_indices = [group_indices; 5];

% Build base_elements (dummy section properties — only connectivity/groups matter for macro gen)
n_elem = size(connectivity, 1);
base_elements = zeros(n_elem, 9);
base_elements(:, 1:2) = connectivity;
base_elements(:, 3) = 25.0;  % D_outer
base_elements(:, 4) = 1.6;   % t
base_elements(:, 5) = 200;   % E
base_elements(:, 6) = 79;    % G
base_elements(:, 7) = 7850;  % rho
base_elements(:, 8) = 1;     % shape
base_elements(:, 9) = 0.85;  % jf

elements_geom = struct();
elements_geom.connectivity = connectivity;
elements_geom.group_indices = group_indices;
elements_geom.base_elements = base_elements;
elements_geom.group_names = {'front_hoop', 'main_hoop', 'side_impact', 'structural', 'shoulder'};

% best_x: 18-variable GA result
% x(1:5) = pipe catalog indices (use middle of catalog range)
n_cat = length(catalog);
mid_idx = max(1, round(n_cat / 2));
best_x = zeros(1, 18);
best_x(1:5) = [mid_idx, mid_idx, mid_idx, mid_idx, mid_idx];
best_x(6:18) = [0.70, 0.35, 0.45, 0.50, 0.6, 0.65, 0.6, 1, 1, 0.5, 0.5, 0.5, 0.5];

% --- Options ---
opts = struct();
opts.output_dir = test_output_dir;
opts.filename = 'test_macro.swb';
opts.export_individual_bodies = true;
opts.body_export_dir = fullfile(test_output_dir, 'pipe_bodies');

fprintf('  Nodes: %d, Elements: %d, Catalog idx: %d\n', ...
    size(nodes, 1), n_elem, mid_idx);

%% Test 1: Function runs without error
fprintf('\n[Test 1] generate_solidworks_macro() runs without error...\n');
try
    output_path = generate_solidworks_macro(nodes, elements_geom, best_x, catalog, materials, opts);
    fprintf('  -> PASS (output: %s)\n', output_path);
catch ME
    fprintf('  -> FAIL: %s\n', ME.message);
    fprintf('  Stack trace:\n');
    for k = 1:length(ME.stack)
        fprintf('    %s line %d\n', ME.stack(k).name, ME.stack(k).line);
    end
    error('Test 1 failed');
end

%% Test 2: Output file exists and is non-empty
fprintf('\n[Test 2] Output .swb file exists and is non-empty...\n');
assert(isfile(output_path), '.swb file not created');
info = dir(output_path);
assert(info.bytes > 1000, sprintf('.swb file too small: %d bytes', info.bytes));
fprintf('  -> PASS (file size: %d bytes)\n', info.bytes);

%% Test 3: Read generated VBA and check structure
fprintf('\n[Test 3] VBA structure validation...\n');
vba_text = fileread(output_path);
vba_lines = strsplit(vba_text, '\n');

% 3a: .swb format — must NOT have Attribute VB_Name (that's .bas only)
assert(~contains(vba_text, 'Attribute VB_Name'), ...
    'Attribute VB_Name found — .swb format should not have it');
assert(contains(vba_text, 'Option Explicit'), 'Missing Option Explicit');
fprintf('  3a: .swb format correct (no Attribute, has Option Explicit) -> PASS\n');

% 3b: Balanced Sub/End Sub
n_sub = sum(~cellfun('isempty', regexp(vba_lines, '^\s*(Public\s+|Private\s+)?Sub\s+')));
n_endsub = sum(~cellfun('isempty', regexp(vba_lines, '^\s*End\s+Sub\s*$')));
assert(n_sub == n_endsub, sprintf('Unbalanced Sub/End Sub: %d vs %d', n_sub, n_endsub));
fprintf('  3b: Balanced Sub/End Sub (%d each) -> PASS\n', n_sub);

% 3c: Balanced Function/End Function
n_func = sum(~cellfun('isempty', regexp(vba_lines, '^\s*(Public\s+|Private\s+)?Function\s+')));
n_endfunc = sum(~cellfun('isempty', regexp(vba_lines, '^\s*End\s+Function\s*$')));
assert(n_func == n_endfunc, sprintf('Unbalanced Function/End Function: %d vs %d', n_func, n_endfunc));
fprintf('  3c: Balanced Function/End Function (%d each) -> PASS\n', n_func);

% 3d: No Dim inside For/Do loops (common VBA error)
in_loop = false;
dim_in_loop = 0;
loop_depth = 0;
for li = 1:length(vba_lines)
    line = strtrim(vba_lines{li});
    if startsWith(line, 'For ') || startsWith(line, 'Do ')
        loop_depth = loop_depth + 1;
    end
    if startsWith(line, 'Next ') || strcmp(line, 'Loop')
        loop_depth = max(0, loop_depth - 1);
    end
    if loop_depth > 0 && startsWith(line, 'Dim ')
        dim_in_loop = dim_in_loop + 1;
        fprintf('  WARNING: Dim inside loop at line %d: %s\n', li, line);
    end
end
assert(dim_in_loop == 0, sprintf('%d Dim statements found inside loops', dim_in_loop));
fprintf('  3d: No Dim inside loops -> PASS\n');

%% Test 4: Node data correctness
fprintf('\n[Test 4] Node coordinate data...\n');

% 4a: Correct node count
node_pattern = sprintf('NUM_NODES As Long = %d', size(nodes, 1));
assert(contains(vba_text, node_pattern), 'Wrong node count in VBA');
fprintf('  4a: Node count (%d) correct -> PASS\n', size(nodes, 1));

% 4b: Coordinate transform verification (spot check node 1)
% Frame: [X_fh, -W_fh, Z_floor] = [0.0, -0.250, 0.050]
% SW:    [-X,    Z,     Y]       = [0.0,  0.050, -0.250]
sw_x = -nodes(1,1);
sw_y = nodes(1,3);
sw_z = nodes(1,2);
pattern_n1 = sprintf('nd(1,1)=%.6f: nd(1,2)=%.6f: nd(1,3)=%.6f', sw_x, sw_y, sw_z);
assert(contains(vba_text, pattern_n1), ...
    sprintf('Node 1 transform mismatch.\n  Expected: %s\n  Looking in VBA...', pattern_n1));
fprintf('  4b: Node 1 coordinate transform correct -> PASS\n');

%% Test 5: Required VBA sections present
fprintf('\n[Test 5] Required VBA sections...\n');
required_subs = {
    'Sub InitNodes()'
    'Sub CreateCustomProfiles('
    'Sub Build_FrontHoop_Bent('
    'Sub Build_MainHoop_Bent('
    'Sub Build_FrontHoop_Bottom('
    'Sub Build_MainHoop_Bottom('
    'Sub Build_FBH('
    'Sub Build_Shoulder('
    'Sub Build_FBHSupport('
    'Sub Build_SideImpact('
    'Sub Build_MH_to_Rear('
    'Sub Build_RearSection('
    'Sub Build_Brackets('
    'Sub Build_Structural('
    'Sub TrimMembers('
    'Sub SetCutListProps('
    'Sub ExportBodies('
    'Sub Main()'
    };
for i = 1:length(required_subs)
    assert(contains(vba_text, required_subs{i}), ...
        sprintf('Missing required sub: %s', required_subs{i}));
end
fprintf('  All %d required subs present -> PASS\n', length(required_subs));

%% Test 6: Profile constants
fprintf('\n[Test 6] Pipe profile constants...\n');
pipe = catalog(mid_idx);
expected_od = sprintf('PIPE_OD_1 As Double = %.2f', pipe.D_outer_mm);
assert(contains(vba_text, expected_od), 'Missing PIPE_OD_1 constant');
fprintf('  Profile constants for catalog(%d) present -> PASS\n', mid_idx);

%% Test 7: Fillet in hoop sections
fprintf('\n[Test 7] Fillet commands in hoop bent sections...\n');
assert(contains(vba_text, 'CreateFillet'), 'No CreateFillet calls found');
% Count fillets: FH has 2, MH has 5 = 7 total
n_fillets = length(strfind(vba_text, 'CreateFillet'));
assert(n_fillets >= 7, sprintf('Expected >=7 fillets, found %d', n_fillets));
fprintf('  Found %d fillet commands (expected >=7) -> PASS\n', n_fillets);

%% Test 8: InsertStructuralWeldment5 calls (hoops + straight members)
fprintf('\n[Test 8] InsertStructuralWeldment5 calls...\n');
n_weldment = length(strfind(vba_text, 'InsertStructuralWeldment5'));
% 2 hoop bent + 2 hoop bottom + 10 straight member groups = 14
% Some may be empty (e.g. structural_other might have 0 elements)
assert(n_weldment >= 10, sprintf('Expected >=10 weldment calls, found %d', n_weldment));
fprintf('  Found %d weldment insertion calls -> PASS\n', n_weldment);

%% Test 9: No double-backslash in paths (VBA doesn't need escaping)
fprintf('\n[Test 9] No double-backslash in VBA string literals...\n');
% The body export dir should have single backslashes
n_double_bs = length(strfind(vba_text, '\\'));
assert(n_double_bs == 0, sprintf('Found %d double-backslash occurrences in VBA', n_double_bs));
fprintf('  No double-backslash found -> PASS\n');

%% Test 10: Main sub execution order
fprintf('\n[Test 10] Main sub execution order...\n');
% Verify priority order: InitNodes -> Profiles -> FH -> MH -> FBH -> ...
main_start = strfind(vba_text, 'Sub Main()');
assert(~isempty(main_start), 'Sub Main() not found');
main_text = vba_text(main_start:end);

% Check order of calls in Main
order_calls = {
    'InitNodes'
    'CreateCustomProfiles'
    'Build_FrontHoop_Bent'
    'Build_MainHoop_Bent'
    'Build_FrontHoop_Bottom'
    'Build_MainHoop_Bottom'
    'Build_FBH'
    'Build_Shoulder'
    'Build_FBHSupport'
    'Build_SideImpact'
    'Build_MH_to_Rear'
    'Build_RearSection'
    'Build_Brackets'
    'Build_Structural'
    'TrimMembers'
    'SetCutListProps'
    'ExportBodies'
    };
prev_pos = 0;
for i = 1:length(order_calls)
    pos = strfind(main_text, order_calls{i});
    assert(~isempty(pos), sprintf('%s not called in Main', order_calls{i}));
    assert(pos(1) > prev_pos, ...
        sprintf('%s appears before %s in Main (wrong order)', ...
        order_calls{i}, order_calls{max(1,i-1)}));
    prev_pos = pos(1);
end
fprintf('  Execution order correct (%d steps) -> PASS\n', length(order_calls));

%% Test 11: Late-binding — no SldWorks.xxx types in output
fprintf('\n[Test 11] Late-binding (no early-binding types in .swb)...\n');
% SldWorks.xxx types cause "User-defined type not defined" compile error
% in .swb macros that lack the type library reference
vba_code_lines = strsplit(vba_text, '\n');
early_binding_count = 0;
for li = 1:length(vba_code_lines)
    line = strtrim(vba_code_lines{li});
    if ~startsWith(line, '''') && contains(line, 'SldWorks.')
        early_binding_count = early_binding_count + 1;
        fprintf('  FAIL at line %d: %s\n', li, line);
    end
end
assert(early_binding_count == 0, ...
    sprintf('Found %d early-binding type references', early_binding_count));
fprintf('  No SldWorks.xxx types found -> PASS\n');

% Also check no remaining enum prefixes (swXxx_e.)
enum_prefix_count = 0;
for li = 1:length(vba_code_lines)
    line = strtrim(vba_code_lines{li});
    if ~startsWith(line, '''') && ~isempty(regexp(line, 'sw\w+_e\.', 'once'))
        enum_prefix_count = enum_prefix_count + 1;
        fprintf('  FAIL at line %d: %s\n', li, line);
    end
end
assert(enum_prefix_count == 0, ...
    sprintf('Found %d enum-prefixed references', enum_prefix_count));
fprintf('  No enum prefixes found -> PASS\n');

% Check SW API constants are defined
assert(contains(vba_text, 'SW_DEFAULT_TEMPLATE_PART'), 'Missing SW_DEFAULT_TEMPLATE_PART');
assert(contains(vba_text, 'SW_SOLID_BODY'), 'Missing SW_SOLID_BODY');
fprintf('  SW API constants defined -> PASS\n');

%% Cleanup
fprintf('\n[Cleanup] Removing temp files...\n');
if isfolder(test_output_dir)
    rmdir(test_output_dir, 's');
end

fprintf('\n=======================================================\n');
fprintf(' All Tests PASSED! (%d tests)\n', 11);
fprintf('=======================================================\n');
end
