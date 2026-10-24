%% VISUALIZE_SUSPENSION_GEOMETRY - Verify front + rear suspension loading
%
% Standalone script: reads suspension geometry from the vehicle artifact,
% converts to frame coordinates, and plots both front and rear A-arms
% with chassis mount points, upright points, and wheel centers.
%
% Frame CS: +X rearward, +Y right, +Z up
% Origin: front wheel center line

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
matlab_dir = fullfile(script_dir, '..', 'matlab');
addpath(matlab_dir);

%% 1. Load artifact directly
json_path = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));
fprintf('Loading: %s\n', json_path);
assert(isfile(json_path), 'File not found: %s', json_path);

artifact = load_vehicle_artifact(json_path);
if ~isfield(artifact.params, 'suspension') || ~isfield(artifact.params.suspension, 'kc') ...
        || ~isfield(artifact.params.suspension.kc, 'corners')
    error('Artifact does not contain suspension hardpoints: %s', json_path);
end
raw = artifact.params.suspension.kc.corners;
WB = artifact.params.mass.wheelbase;
fprintf('Wheelbase: %.4f m\n', WB);

%% 2. Coordinate conversion functions
% Suspension JSON: X fwd, Y left, Z down
% Frame CS:        X rear, Y right, Z up
% Front: origin = front wheel center line → just invert X and Z
% Rear:  origin = rear wheel center line → invert X and Z, then add wheelbase to X

map_front = @(pt) [-pt(1), pt(2), -pt(3)];
map_rear  = @(pt) [-pt(1) + WB, pt(2), -pt(3)];

%% 3. Extract all points

% --- Front Left (FL) ---
fl = raw.FL.hardpoints;
fl_lf = map_front(fl.lower_arm_chassis_fore);
fl_lr = map_front(fl.lower_arm_chassis_aft);
fl_uf = map_front(fl.upper_arm_chassis_fore);
fl_ur = map_front(fl.upper_arm_chassis_aft);
fl_up_l = map_front(fl.lower_ball_joint);
fl_up_u = map_front(fl.upper_ball_joint);
% Wheel center
fl_wc = map_front(fl.wheel_center);

% --- Front Right (FR) = mirror FL across Y=0 ---
fr_lf = [fl_lf(1),  abs(fl_lf(2)), fl_lf(3)];
fr_lr = [fl_lr(1),  abs(fl_lr(2)), fl_lr(3)];
fr_uf = [fl_uf(1),  abs(fl_uf(2)), fl_uf(3)];
fr_ur = [fl_ur(1),  abs(fl_ur(2)), fl_ur(3)];
fr_up_l = [fl_up_l(1),  abs(fl_up_l(2)), fl_up_l(3)];
fr_up_u = [fl_up_u(1),  abs(fl_up_u(2)), fl_up_u(3)];
fr_wc = [fl_wc(1),  abs(fl_wc(2)), fl_wc(3)];
% FL needs negative Y
fl_lf(2) = -abs(fl_lf(2)); fl_lr(2) = -abs(fl_lr(2));
fl_uf(2) = -abs(fl_uf(2)); fl_ur(2) = -abs(fl_ur(2));
fl_up_l(2) = -abs(fl_up_l(2)); fl_up_u(2) = -abs(fl_up_u(2));
fl_wc(2) = -abs(fl_wc(2));

% --- Rear Left (RL) ---
rl = raw.RL.hardpoints;
rl_lf = map_rear(rl.lower_arm_chassis_fore);
rl_lr = map_rear(rl.lower_arm_chassis_aft);
rl_uf = map_rear(rl.upper_arm_chassis_fore);
rl_ur = map_rear(rl.upper_arm_chassis_aft);
rl_up_l = map_rear(rl.lower_ball_joint);
rl_up_u = map_rear(rl.upper_ball_joint);
rl_wc = map_rear(rl.wheel_center);
rl_lf(2) = -abs(rl_lf(2)); rl_lr(2) = -abs(rl_lr(2));
rl_uf(2) = -abs(rl_uf(2)); rl_ur(2) = -abs(rl_ur(2));
rl_up_l(2) = -abs(rl_up_l(2)); rl_up_u(2) = -abs(rl_up_u(2));
rl_wc(2) = -abs(rl_wc(2));

% --- Rear Right (RR) = mirror RL across Y=0 ---
rr_lf = [rl_lf(1),  abs(rl_lf(2)), rl_lf(3)];
rr_lr = [rl_lr(1),  abs(rl_lr(2)), rl_lr(3)];
rr_uf = [rl_uf(1),  abs(rl_uf(2)), rl_uf(3)];
rr_ur = [rl_ur(1),  abs(rl_ur(2)), rl_ur(3)];
rr_up_l = [rl_up_l(1),  abs(rl_up_l(2)), rl_up_l(3)];
rr_up_u = [rl_up_u(1),  abs(rl_up_u(2)), rl_up_u(3)];
rr_wc = [rl_wc(1),  abs(rl_wc(2)), rl_wc(3)];

%% 4. Print coordinates for verification
fprintf('\n=== Front Chassis Mounts (Frame CS) ===\n');
fprintf('  FL Lower F: [%.4f, %.4f, %.4f]\n', fl_lf);
fprintf('  FL Lower R: [%.4f, %.4f, %.4f]\n', fl_lr);
fprintf('  FL Upper F: [%.4f, %.4f, %.4f]\n', fl_uf);
fprintf('  FL Upper R: [%.4f, %.4f, %.4f]\n', fl_ur);
fprintf('  FL Wheel C: [%.4f, %.4f, %.4f]\n', fl_wc);

fprintf('\n=== Rear Chassis Mounts (Frame CS) ===\n');
fprintf('  RL Lower F: [%.4f, %.4f, %.4f]\n', rl_lf);
fprintf('  RL Lower R: [%.4f, %.4f, %.4f]\n', rl_lr);
fprintf('  RL Upper F: [%.4f, %.4f, %.4f]\n', rl_uf);
fprintf('  RL Upper R: [%.4f, %.4f, %.4f]\n', rl_ur);
fprintf('  RL Wheel C: [%.4f, %.4f, %.4f]\n', rl_wc);

%% 5. Visualize
figure('Name', 'Suspension Geometry Verification', 'Position', [100, 100, 1200, 800]);
hold on; axis equal; grid on;
xlabel('X [m] (→ rearward)'); ylabel('Y [m] (→ right)'); zlabel('Z [m] (→ up)');
title('Front + Rear Suspension Geometry (Frame CS)');

% Helper: draw A-arm triangle (2 chassis mounts → 1 upright)
draw_arm = @(cf, cr, up, col) ...
    plot3([cf(1), up(1), cr(1), cf(1)], ...
          [cf(2), up(2), cr(2), cf(2)], ...
          [cf(3), up(3), cr(3), cf(3)], '-', 'Color', col, 'LineWidth', 2);

% --- Front A-arms ---
% Lower (orange)
draw_arm(fl_lf, fl_lr, fl_up_l, [1 0.5 0]);
draw_arm(fr_lf, fr_lr, fr_up_l, [1 0.5 0]);
% Upper (dark orange)
draw_arm(fl_uf, fl_ur, fl_up_u, [0.8 0.3 0]);
draw_arm(fr_uf, fr_ur, fr_up_u, [0.8 0.3 0]);

% --- Rear A-arms ---
% Lower (green)
draw_arm(rl_lf, rl_lr, rl_up_l, [0 0.7 0]);
draw_arm(rr_lf, rr_lr, rr_up_l, [0 0.7 0]);
% Upper (dark green)
draw_arm(rl_uf, rl_ur, rl_up_u, [0 0.5 0]);
draw_arm(rr_uf, rr_ur, rr_up_u, [0 0.5 0]);

% --- Uprights (kingpin lines) ---
plot3([fl_up_l(1), fl_up_u(1)], [fl_up_l(2), fl_up_u(2)], [fl_up_l(3), fl_up_u(3)], 'k-', 'LineWidth', 3);
plot3([fr_up_l(1), fr_up_u(1)], [fr_up_l(2), fr_up_u(2)], [fr_up_l(3), fr_up_u(3)], 'k-', 'LineWidth', 3);
plot3([rl_up_l(1), rl_up_u(1)], [rl_up_l(2), rl_up_u(2)], [rl_up_l(3), rl_up_u(3)], 'k-', 'LineWidth', 3);
plot3([rr_up_l(1), rr_up_u(1)], [rr_up_l(2), rr_up_u(2)], [rr_up_l(3), rr_up_u(3)], 'k-', 'LineWidth', 3);

% --- Chassis mount points ---
front_mounts = [fl_lf; fl_lr; fl_uf; fl_ur; fr_lf; fr_lr; fr_uf; fr_ur];
rear_mounts  = [rl_lf; rl_lr; rl_uf; rl_ur; rr_lf; rr_lr; rr_uf; rr_ur];
scatter3(front_mounts(:,1), front_mounts(:,2), front_mounts(:,3), 80, 'r', 'filled', 'MarkerEdgeColor', 'k');
scatter3(rear_mounts(:,1),  rear_mounts(:,2),  rear_mounts(:,3),  80, 'g', 'filled', 'MarkerEdgeColor', 'k');

% --- Upright points ---
uprights = [fl_up_l; fl_up_u; fr_up_l; fr_up_u; rl_up_l; rl_up_u; rr_up_l; rr_up_u];
scatter3(uprights(:,1), uprights(:,2), uprights(:,3), 60, 'b', 'filled');

% --- Wheel centers ---
wcs = [fl_wc; fr_wc; rl_wc; rr_wc];
scatter3(wcs(:,1), wcs(:,2), wcs(:,3), 120, 'm', 'p', 'filled');

% --- Draw tire circles ---
tire_r = 0.228;
theta = linspace(0, 2*pi, 60);
for w = 1:4
    wc = wcs(w, :);
    % Circle in X-Z plane at Y=wc(2)
    cx = wc(1) + tire_r * cos(theta);
    cz = wc(3) + tire_r * sin(theta);
    cy = ones(size(theta)) * wc(2);
    plot3(cx, cy, cz, '-', 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
end

% --- Ground line ---
plot3([min(wcs(:,1))-0.3, max(wcs(:,1))+0.3], [0, 0], [0, 0], 'k--', 'LineWidth', 0.5);

% --- Axle lines ---
plot3([fl_wc(1), fr_wc(1)], [fl_wc(2), fr_wc(2)], [fl_wc(3), fr_wc(3)], 'r--', 'LineWidth', 1);
plot3([rl_wc(1), rr_wc(1)], [rl_wc(2), rr_wc(2)], [rl_wc(3), rr_wc(3)], 'g--', 'LineWidth', 1);

% --- Labels ---
text(fl_wc(1), fl_wc(2)-0.05, fl_wc(3)-0.05, 'FL', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
text(fr_wc(1), fr_wc(2)+0.05, fr_wc(3)-0.05, 'FR', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'r');
text(rl_wc(1), rl_wc(2)-0.05, rl_wc(3)-0.05, 'RL', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0.5 0]);
text(rr_wc(1), rr_wc(2)+0.05, rr_wc(3)-0.05, 'RR', 'FontSize', 12, 'FontWeight', 'bold', 'Color', [0 0.5 0]);

% --- Annotations ---
text(0, 0, -0.05, sprintf('WB = %.3f m', WB), 'FontSize', 10, 'Color', [0.3 0.3 0.3]);
text(fl_wc(1), 0, fl_wc(3)+0.35, sprintf('Front Track = %.3f m', abs(fr_wc(2)-fl_wc(2))), 'FontSize', 9, 'Color', 'r');
text(rl_wc(1), 0, rl_wc(3)+0.35, sprintf('Rear Track = %.3f m', abs(rr_wc(2)-rl_wc(2))), 'FontSize', 9, 'Color', [0 0.5 0]);

legend({'Front Lower Arm', '', 'Front Upper Arm', '', ...
        'Rear Lower Arm', '', 'Rear Upper Arm', '', ...
        'Upright', '', '', '', ...
        'Front Mounts', 'Rear Mounts', 'Upright Pts', 'Wheel Centers'}, ...
       'Location', 'eastoutside');

view([-45, 25]);
rotate3d on;

fprintf('\n✓ Visualization complete. Check figure window.\n');
