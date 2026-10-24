%% VISUALIZE_FRAME_WITH_BRACKETS - Frame + Front/Rear suspension brackets
%
% Builds on the verified suspension geometry and adds:
%   1. Frame skeleton (FBH → FH → SI → MH → Rear Section)
%   2. Front suspension brackets (existing pattern)
%   3. Rear suspension brackets (new, same pattern as front)
%
% This is a visual verification tool — no FEM solver needed.

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

%% 1. Load suspension geometry
json_path = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));
artifact = load_vehicle_artifact(json_path);
if ~isfield(artifact.params, 'suspension') || ~isfield(artifact.params.suspension, 'kc') ...
        || ~isfield(artifact.params.suspension.kc, 'corners')
    error('Artifact does not contain suspension hardpoints: %s', json_path);
end
raw = artifact.params.suspension.kc.corners;
WB = artifact.params.mass.wheelbase;

fprintf('=== Frame + Bracket Visualization ===\n');
fprintf('Wheelbase: %.4f m\n', WB);

%% 2. Coordinate conversion
% Suspension JSON: X fwd, Y left, Z down
% Frame CS:        X rear, Y right, Z up
map_front = @(pt) [-pt(1), pt(2), -pt(3)];
map_rear  = @(pt) [-pt(1) + WB, pt(2), -pt(3)];

%% 3. Extract front suspension (FL, mirrored to FR)
fl = raw.FL.hardpoints;
front_chassis_mounts = zeros(8, 3);  % [FL_LF, FR_LF, FL_UF, FR_UF, FL_LR, FR_LR, FL_UR, FR_UR]
front_uprights = zeros(4, 3);       % [FL_Lower, FR_Lower, FL_Upper, FR_Upper]
front_wc = zeros(2, 3);             % [FL_WC, FR_WC]

% Lower F
pt = map_front(fl.lower_arm_chassis_fore);
front_chassis_mounts(1, :) = [pt(1), -abs(pt(2)), pt(3)]; % FL
front_chassis_mounts(2, :) = [pt(1),  abs(pt(2)), pt(3)]; % FR
% Upper F  
pt = map_front(fl.upper_arm_chassis_fore);
front_chassis_mounts(3, :) = [pt(1), -abs(pt(2)), pt(3)];
front_chassis_mounts(4, :) = [pt(1),  abs(pt(2)), pt(3)];
% Lower R
pt = map_front(fl.lower_arm_chassis_aft);
front_chassis_mounts(5, :) = [pt(1), -abs(pt(2)), pt(3)];
front_chassis_mounts(6, :) = [pt(1),  abs(pt(2)), pt(3)];
% Upper R
pt = map_front(fl.upper_arm_chassis_aft);
front_chassis_mounts(7, :) = [pt(1), -abs(pt(2)), pt(3)];
front_chassis_mounts(8, :) = [pt(1),  abs(pt(2)), pt(3)];

% Uprights
pt = map_front(fl.lower_ball_joint);
front_uprights(1, :) = [pt(1), -abs(pt(2)), pt(3)];
front_uprights(2, :) = [pt(1),  abs(pt(2)), pt(3)];
pt = map_front(fl.upper_ball_joint);
front_uprights(3, :) = [pt(1), -abs(pt(2)), pt(3)];
front_uprights(4, :) = [pt(1),  abs(pt(2)), pt(3)];

% Wheel centers
pt = map_front(fl.wheel_center);
front_wc(1, :) = [pt(1), -abs(pt(2)), pt(3)];
front_wc(2, :) = [pt(1),  abs(pt(2)), pt(3)];

%% 4. Extract rear suspension (RL, mirrored to RR)
rl = raw.RL.hardpoints;
rear_chassis_mounts = zeros(8, 3);  % Same order as front
rear_uprights = zeros(4, 3);
rear_wc = zeros(2, 3);

% Lower F
pt = map_rear(rl.lower_arm_chassis_fore);
rear_chassis_mounts(1, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_chassis_mounts(2, :) = [pt(1),  abs(pt(2)), pt(3)];
% Upper F
pt = map_rear(rl.upper_arm_chassis_fore);
rear_chassis_mounts(3, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_chassis_mounts(4, :) = [pt(1),  abs(pt(2)), pt(3)];
% Lower R
pt = map_rear(rl.lower_arm_chassis_aft);
rear_chassis_mounts(5, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_chassis_mounts(6, :) = [pt(1),  abs(pt(2)), pt(3)];
% Upper R
pt = map_rear(rl.upper_arm_chassis_aft);
rear_chassis_mounts(7, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_chassis_mounts(8, :) = [pt(1),  abs(pt(2)), pt(3)];

% Uprights
pt = map_rear(rl.lower_ball_joint);
rear_uprights(1, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_uprights(2, :) = [pt(1),  abs(pt(2)), pt(3)];
pt = map_rear(rl.upper_ball_joint);
rear_uprights(3, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_uprights(4, :) = [pt(1),  abs(pt(2)), pt(3)];

% Wheel centers
pt = map_rear(rl.wheel_center);
rear_wc(1, :) = [pt(1), -abs(pt(2)), pt(3)];
rear_wc(2, :) = [pt(1),  abs(pt(2)), pt(3)];

%% 5. Frame geometry parameters
Z_floor = 0.050;
W_fh  = 0.250;
W_si  = 0.300;
W_mh  = 0.300;
W_mh_mid = W_mh + 0.100; % Mid-wide: base + 100mm margin (400mm from center)
W_mh_top = 0.100;     % Shoulder: 100mm from center (narrower taper)

H_si_lower = 0.05 + Z_floor;
H_si_upper = 0.45 + Z_floor;
H_mh  = 1.35 + Z_floor;
H_sh  = 1.15 + Z_floor;

% FBH parameters
FBH_width = 0.400;
FBH_height = 0.400;
FBH_x_shift = 0.030;
FBH_z_offset = -0.050;
FH_min_dist = 0.100;

% FBH position (from load_suspension_hardpoints logic)
base_offset_x = 0.500;
offset_x = base_offset_x - FBH_x_shift;
z_cv = -fl.wheel_center(3);  % Wheel center Z in frame CS
fbh_x = -offset_x;
fbh_z = z_cv + FBH_z_offset;
W_fbh = FBH_width / 2;

fbh_nodes = [
    fbh_x, -W_fbh, fbh_z;
    fbh_x,  W_fbh, fbh_z;
    fbh_x, -W_fbh, fbh_z + FBH_height;
    fbh_x,  W_fbh, fbh_z + FBH_height;
];

% FH position (behind rearmost front susp mount)
max_front_susp_x = max(front_chassis_mounts(:, 1));
X_fh = max_front_susp_x + FH_min_dist;
Z_FH_lower = max(Z_floor, min(fbh_nodes(:,3)) - 0.100);
H_fh = max(fbh_nodes(:,3)) + 0.050;

% MH position
X_mh = X_fh + 0.60;

% SI position
X_si = X_fh + (X_mh - X_fh) * 0.35;

fprintf('FBH X: %.3f m\n', fbh_x);
fprintf('FH X:  %.3f m\n', X_fh);
fprintf('SI X:  %.3f m\n', X_si);
fprintf('MH X:  %.3f m\n', X_mh);

%% 6. Rear section box (from rear susp mount positions)
rear_f_x = min(rear_chassis_mounts(1:4, 1));    % Front mounts
rear_r_x = max(rear_chassis_mounts(5:8, 1));    % Rear mounts
rear_y_max = max(abs(rear_chassis_mounts(:, 2)));
rear_z_max = max(rear_chassis_mounts(:, 3));

X_rs_f = rear_f_x - 0.05;
X_rs_r = rear_r_x + 0.05;
W_rs = W_mh;  % Same width as MH base — mounts protrude outward
H_rs = rear_z_max + 0.10;

fprintf('RS front ring X: %.3f m\n', X_rs_f);
fprintf('RS rear ring X:  %.3f m\n', X_rs_r);
fprintf('RS width:  ±%.3f m\n', W_rs);
fprintf('RS height: %.3f m\n', H_rs);

%% 7. Generate inner bracket nodes (50mm axial extension)
% Front: mount → upright vector, extend 50mm from mount into chassis
L_brk = 0.050;
front_inner = zeros(8, 3);
% Mapping: rows 1,2→upright lower(1,2); 3,4→upright upper(3,4); 5,6→lower(1,2); 7,8→upper(3,4)
front_up_map = [1, 2, 3, 4, 1, 2, 3, 4];
for i = 1:8
    m = front_chassis_mounts(i, :);
    u = front_uprights(front_up_map(i), :);
    v = m - u;
    front_inner(i, :) = m + (v / norm(v)) * L_brk;
end

% Rear: same pattern
rear_inner = zeros(8, 3);
rear_up_map = [1, 2, 3, 4, 1, 2, 3, 4];
for i = 1:8
    m = rear_chassis_mounts(i, :);
    u = rear_uprights(rear_up_map(i), :);
    v = m - u;
    rear_inner(i, :) = m + (v / norm(v)) * L_brk;
end

%% 8. Build frame nodes (for visualization only)
% MH mid-wide height: FH upper height - 200mm
H_mh_mid = H_fh - 0.200;

frame_nodes = [
    % FH (1-4)
    X_fh, -W_fh, Z_FH_lower;
    X_fh,  W_fh, Z_FH_lower;
    X_fh, -W_fh, H_fh;
    X_fh,  W_fh, H_fh;
    % SI (5-8)
    X_si, -W_si, H_si_lower;
    X_si,  W_si, H_si_lower;
    X_si, -W_si, H_si_upper;
    X_si,  W_si, H_si_upper;
    % MH Heptagon (9-15):
    %   9,10 = base L/R (wide)
    %   11,12 = mid-wide L/R (at FH height - 200mm)
    %   13,14 = shoulder L/R (narrow taper)
    %   15    = top center
    X_mh, -W_mh,     Z_floor;    % 9:  base L
    X_mh,  W_mh,     Z_floor;    % 10: base R
    X_mh, -W_mh_mid, H_mh_mid;   % 11: mid-wide L
    X_mh,  W_mh_mid, H_mh_mid;   % 12: mid-wide R
    X_mh, -W_mh_top, H_sh;       % 13: shoulder L
    X_mh,  W_mh_top, H_sh;       % 14: shoulder R
    X_mh,  0.0,      H_mh;       % 15: top center
    % RS Front Ring (16-19)
    X_rs_f, -W_rs, H_rs;
    X_rs_f,  W_rs, H_rs;
    X_rs_f, -W_rs, Z_floor;
    X_rs_f,  W_rs, Z_floor;
    % RS Rear Ring (20-23)
    X_rs_r, -W_rs, H_rs;
    X_rs_r,  W_rs, H_rs;
    X_rs_r, -W_rs, Z_floor;
    X_rs_r,  W_rs, Z_floor;
];

% Frame element connectivity (node pairs)
frame_edges = [
    % FH rectangle
    1,2; 2,4; 4,3; 3,1;
    % FH→SI rails
    1,5; 2,6; 3,7; 4,8;
    % SI verticals
    5,7; 6,8;
    % SI→MH rails
    5,9; 6,10; 7,11; 8,12;
    % MH heptagon (9→10→12→14→15→13→11→9)
    9,10; 10,12; 12,14; 14,15; 15,13; 13,11; 11,9;
    % MH→RS front ring
    13,16; 14,17; 9,18; 10,19;
    % RS front ring
    16,17; 18,19; 16,18; 17,19;
    % RS rear ring
    20,21; 22,23; 20,22; 21,23;
    % RS rails (front→rear)
    16,20; 17,21; 18,22; 19,23;
];

%% 9. Plot
figure('Name', 'Frame + Suspension Brackets', 'Position', [50, 50, 1400, 900]);
hold on; axis equal; grid on;
xlabel('X [m] (→ rearward)'); ylabel('Y [m] (→ right)'); zlabel('Z [m] (→ up)');
title('Frame Structure with Front + Rear Suspension Brackets');

% --- Frame tubes (dark blue) ---
for e = 1:size(frame_edges, 1)
    n1 = frame_nodes(frame_edges(e, 1), :);
    n2 = frame_nodes(frame_edges(e, 2), :);
    plot3([n1(1), n2(1)], [n1(2), n2(2)], [n1(3), n2(3)], '-', 'Color', [0.1 0.1 0.6], 'LineWidth', 2.5);
end

% --- FBH rectangle (dark blue) ---
fbh_order = [1,2,4,3,1];
plot3(fbh_nodes(fbh_order, 1), fbh_nodes(fbh_order, 2), fbh_nodes(fbh_order, 3), '-', 'Color', [0.1 0.1 0.6], 'LineWidth', 2.5);
% FBH → FH supports
for i = 1:4
    plot3([fbh_nodes(i,1), frame_nodes(i,1)], [fbh_nodes(i,2), frame_nodes(i,2)], [fbh_nodes(i,3), frame_nodes(i,3)], '-', 'Color', [0.3 0.3 0.8], 'LineWidth', 1.5);
end

% --- Frame node markers ---
scatter3(frame_nodes(:,1), frame_nodes(:,2), frame_nodes(:,3), 40, [0.1 0.1 0.6], 'filled');
scatter3(fbh_nodes(:,1), fbh_nodes(:,2), fbh_nodes(:,3), 40, [0.1 0.1 0.6], 'filled');

% --- Front A-arms ---
draw_arm = @(cf, cr, up, col) ...
    plot3([cf(1), up(1), cr(1), cf(1)], [cf(2), up(2), cr(2), cf(2)], [cf(3), up(3), cr(3), cf(3)], '--', 'Color', col, 'LineWidth', 1.5);

for side = 1:2  % 1=left(odd), 2=right(even)
    % Lower arm: mounts 1/2(LF) and 5/6(LR) → upright 1/2(Lower)
    draw_arm(front_chassis_mounts(side, :), front_chassis_mounts(side+4, :), front_uprights(side, :), [1 0.5 0]);
    % Upper arm: mounts 3/4(UF) and 7/8(UR) → upright 3/4(Upper)
    draw_arm(front_chassis_mounts(side+2, :), front_chassis_mounts(side+6, :), front_uprights(side+2, :), [0.8 0.3 0]);
end

% --- Rear A-arms ---
for side = 1:2
    draw_arm(rear_chassis_mounts(side, :), rear_chassis_mounts(side+4, :), rear_uprights(side, :), [0 0.7 0]);
    draw_arm(rear_chassis_mounts(side+2, :), rear_chassis_mounts(side+6, :), rear_uprights(side+2, :), [0 0.5 0]);
end

% --- Uprights ---
for side = 1:2
    plot3([front_uprights(side,1), front_uprights(side+2,1)], ...
          [front_uprights(side,2), front_uprights(side+2,2)], ...
          [front_uprights(side,3), front_uprights(side+2,3)], 'k-', 'LineWidth', 3);
    plot3([rear_uprights(side,1), rear_uprights(side+2,1)], ...
          [rear_uprights(side,2), rear_uprights(side+2,2)], ...
          [rear_uprights(side,3), rear_uprights(side+2,3)], 'k-', 'LineWidth', 3);
end

% --- Front bracket pipes: mount → inner bracket → frame ---
for i = 1:8
    % Mount → inner bracket (red)
    plot3([front_chassis_mounts(i,1), front_inner(i,1)], ...
          [front_chassis_mounts(i,2), front_inner(i,2)], ...
          [front_chassis_mounts(i,3), front_inner(i,3)], 'r-', 'LineWidth', 2);
end
scatter3(front_chassis_mounts(:,1), front_chassis_mounts(:,2), front_chassis_mounts(:,3), 80, 'r', 'filled', 'MarkerEdgeColor', 'k');
scatter3(front_inner(:,1), front_inner(:,2), front_inner(:,3), 50, [0.8 0 0], 'filled', 's');

% Front inner brackets → frame connections (adapter pipes)
% F brackets (1-4) → FBH (nearest)
for i = 1:4
    plot3([front_inner(i,1), fbh_nodes(i,1)], ...
          [front_inner(i,2), fbh_nodes(i,2)], ...
          [front_inner(i,3), fbh_nodes(i,3)], 'r-', 'LineWidth', 1.5);
end
% R brackets (5-8) → FH (nearest)
for i = 5:8
    fh_node = frame_nodes(i-4, :); % FH nodes 1-4
    plot3([front_inner(i,1), fh_node(1)], ...
          [front_inner(i,2), fh_node(2)], ...
          [front_inner(i,3), fh_node(3)], 'r-', 'LineWidth', 1.5);
end

% --- Rear bracket pipes: mount → inner bracket → rear section ---
for i = 1:8
    % Mount → inner bracket (cyan)
    plot3([rear_chassis_mounts(i,1), rear_inner(i,1)], ...
          [rear_chassis_mounts(i,2), rear_inner(i,2)], ...
          [rear_chassis_mounts(i,3), rear_inner(i,3)], 'c-', 'LineWidth', 2);
end
scatter3(rear_chassis_mounts(:,1), rear_chassis_mounts(:,2), rear_chassis_mounts(:,3), 80, [0 0.7 0], 'filled', 'MarkerEdgeColor', 'k');
scatter3(rear_inner(:,1), rear_inner(:,2), rear_inner(:,3), 50, [0 0.5 0.5], 'filled', 's');

% Rear inner brackets → rear section frame connections
% F brackets (1-4) → RS front ring (nodes 16-19 of frame)
% Mapping: 1→18(lower L), 2→19(lower R), 3→16(upper L), 4→17(upper R)
rs_f_targets = [18, 19, 16, 17];
for i = 1:4
    rs_node = frame_nodes(rs_f_targets(i), :);
    plot3([rear_inner(i,1), rs_node(1)], ...
          [rear_inner(i,2), rs_node(2)], ...
          [rear_inner(i,3), rs_node(3)], 'c-', 'LineWidth', 1.5);
end
% R brackets (5-8) → RS rear ring (nodes 20-23 of frame)
% Mapping: 5→22(lower L), 6→23(lower R), 7→20(upper L), 8→21(upper R)
rs_r_targets = [22, 23, 20, 21];
for i = 5:8
    rs_node = frame_nodes(rs_r_targets(i-4), :);
    plot3([rear_inner(i,1), rs_node(1)], ...
          [rear_inner(i,2), rs_node(2)], ...
          [rear_inner(i,3), rs_node(3)], 'c-', 'LineWidth', 1.5);
end

% --- Tire circles ---
tire_r = 0.228;
theta = linspace(0, 2*pi, 60);
all_wc = [front_wc; rear_wc];
for w = 1:4
    wc = all_wc(w, :);
    cx = wc(1) + tire_r * cos(theta);
    cz = wc(3) + tire_r * sin(theta);
    cy = ones(size(theta)) * wc(2);
    plot3(cx, cy, cz, '-', 'Color', [0.7 0.7 0.7], 'LineWidth', 1);
end

% --- Labels ---
text(front_wc(1,1), front_wc(1,2)-0.08, front_wc(1,3), 'FL', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
text(front_wc(2,1), front_wc(2,2)+0.03, front_wc(2,3), 'FR', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'r');
text(rear_wc(1,1), rear_wc(1,2)-0.08, rear_wc(1,3), 'RL', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
text(rear_wc(2,1), rear_wc(2,2)+0.03, rear_wc(2,3), 'RR', 'FontSize', 11, 'FontWeight', 'bold', 'Color', [0 0.5 0]);

% Structure labels
text(X_fh, 0, H_fh+0.05, 'FH', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(X_mh, 0, H_mh+0.05, 'MH', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(fbh_x, 0, max(fbh_nodes(:,3))+0.05, 'FBH', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text((X_rs_f+X_rs_r)/2, 0, H_rs+0.05, 'Rear Section', 'FontSize', 10, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Color', [0 0.5 0.5]);

view([-45, 25]);
rotate3d on;

fprintf('\n✓ Frame + brackets visualization complete.\n');
fprintf('  Frame nodes: %d\n', size(frame_nodes, 1));
fprintf('  Front mounts: %d, inner brackets: %d\n', size(front_chassis_mounts, 1), size(front_inner, 1));
fprintf('  Rear mounts: %d, inner brackets: %d\n', size(rear_chassis_mounts, 1), size(rear_inner, 1));
