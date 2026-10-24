% PLOT_A_ARM_GEOMETRY - Visualize the optimized A-Arm structure
clear; clc; close all;
addpath(fullfile('..', 'matlab'));
json_file = resolve_default_vehicle_json(fullfile(fileparts(mfilename('fullpath')), '..'), ...
    'RequireExportReady', true, ...
    'RequireSuspensionHardpoints', true, ...
    'RequiredHardpointCorners', {'FL'});
if ~isfile(json_file)
    error('Run main_a_arm_optimizer first to generate %s', json_file);
end

artifact = load_vehicle_artifact(json_file);
fl = artifact.params.suspension.kc.corners.FL.hardpoints;

figure('Name', 'Optimized FSAE A-Arm Geometry (Front Left)', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal; view(3);

% Plot Chassis Points
U_F = fl.upper_arm_chassis_fore;
U_R = fl.upper_arm_chassis_aft;
L_F = fl.lower_arm_chassis_fore;
L_R = fl.lower_arm_chassis_aft;

% Plot Upright Points
U_U = fl.upper_ball_joint;
L_U = fl.lower_ball_joint;

% Wheel Center & Contact Patch
W_C = fl.wheel_center;
TCP = fl.contact_patch;

% Draw Arms (Upper = Red, Lower = Blue)
plot3([U_F(1), U_U(1)], [U_F(2), U_U(2)], [U_F(3), U_U(3)], 'r-', 'LineWidth', 2);
plot3([U_R(1), U_U(1)], [U_R(2), U_U(2)], [U_R(3), U_U(3)], 'r-', 'LineWidth', 2);
plot3([L_F(1), L_U(1)], [L_F(2), L_U(2)], [L_F(3), L_U(3)], 'b-', 'LineWidth', 2);
plot3([L_R(1), L_U(1)], [L_R(2), L_U(2)], [L_R(3), L_U(3)], 'b-', 'LineWidth', 2);

% Draw Chassis Mount Base Lines
plot3([U_F(1), U_R(1)], [U_F(2), U_R(2)], [U_F(3), U_R(3)], 'k--', 'LineWidth', 1);
plot3([L_F(1), L_R(1)], [L_F(2), L_R(2)], [L_F(3), L_R(3)], 'k--', 'LineWidth', 1);

% Draw Kingpin Axis
plot3([U_U(1), L_U(1)], [U_U(2), L_U(2)], [U_U(3), L_U(3)], 'm-', 'LineWidth', 3);

% Draw Wheel Center and Upright body connection
plot3([U_U(1), W_C(1)], [U_U(2), W_C(2)], [U_U(3), W_C(3)], 'k-', 'LineWidth', 1.5);
plot3([L_U(1), W_C(1)], [L_U(2), W_C(2)], [L_U(3), W_C(3)], 'k-', 'LineWidth', 1.5);

% Draw Tire Cylinder Mockup
theta = linspace(0, 2*pi, 30);
y_w = W_C(2);
tire_radius = norm(W_C - TCP);
for t = theta
    px = W_C(1) + tire_radius * cos(t);
    pz = W_C(3) + tire_radius * sin(t);
    plot3(px, y_w, pz, 'k.', 'MarkerSize', 5);
end

% Labels
scatter3(U_U(1), U_U(2), U_U(3), 50, 'r', 'filled');
scatter3(L_U(1), L_U(2), L_U(3), 50, 'b', 'filled');
scatter3(W_C(1), W_C(2), W_C(3), 80, 'k', 'filled', 'MarkerEdgeColor', 'w');

xlabel('X (Lon) [m]');
ylabel('Y (Lat) [m]');
zlabel('Z (Vert) [m]');
title('Front Left A-Arm Spatial Geometry');
legend('Upper Arm', 'Upper Arm', 'Lower Arm', 'Lower Arm', 'Chassis', 'Chassis', 'Kingpin', 'Location', 'best');
