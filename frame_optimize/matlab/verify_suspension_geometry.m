%% VERIFY_SUSPENSION_GEOMETRY
% Helper script to visualize the output of load_suspension_hardpoints.m
% and compare it against the expected Frame dimensions.

clear; clc; close all;

script_dir = fileparts(mfilename('fullpath'));
data_dir = fullfile(script_dir, '..', 'data');
json_path = resolve_suspension_geometry_source(fullfile(script_dir, '..', '..'));

fprintf('Loading suspension points from: %s\n', json_path);

% User requested FBH offset: 500mm forward of wheel center, height offset 0 (or from ground)
offset_x = 0.500;
offset_z = 0.000;
[fbh_nodes, mount_nodes, upright_nodes, success] = load_suspension_hardpoints(json_path, offset_x, offset_z);

if ~success
    error('Failed to load suspension points!');
end

% Display Coordinates
labels = {
    '1: FL Lower F', '2: FR Lower F', '3: FL Upper F', '4: FR Upper F', ...
    '5: FL Lower R', '6: FR Lower R', '7: FL Upper R', '8: FR Upper R'
};

fprintf('\nExtracted Suspension Mount Coordinates (Frame CS):\n');
for i = 1:8
    fprintf('  %s: [%.3f, %.3f, %.3f]\n', labels{i}, mount_nodes(i, 1), mount_nodes(i, 2), mount_nodes(i, 3));
end

% Plotting
figure('Name', 'Suspension Mount Verification', 'Position', [100, 100, 800, 600]);
hold on; grid on; axis equal;
xlabel('X [m] (Positive is Rearward)'); 
ylabel('Y [m] (Left is Negative, Right is Positive)'); 
zlabel('Z [m] (Up)');
title('Front Suspension Chassis Mounts Visualization');

% Draw the Bulkhead reference box (Nodes 25-28 from frame)
% Use the fbh_nodes returned by the function which are positioned based on offsets
fbh = [
    fbh_nodes(1, :); % Lower left
    fbh_nodes(2, :); % Lower right
    fbh_nodes(4, :); % Upper right
    fbh_nodes(3, :); % Upper left
    fbh_nodes(1, :); % Close loop
];
plot3(fbh(:,1), fbh(:,2), fbh(:,3), 'k-', 'LineWidth', 2);
text(fbh_nodes(3,1), 0, fbh_nodes(3,3)+0.05, 'Front Bulkhead Frame', 'HorizontalAlignment', 'center', 'Color', 'k');

% Draw Ground
[Xg, Yg] = meshgrid([-0.5, 0.5], [-0.5, 0.5]);
Zg = zeros(2, 2);
surf(Xg, Yg, Zg, 'FaceColor', [0.9 0.9 0.9], 'FaceAlpha', 0.5, 'EdgeColor', 'none');

% Plot Suspension Mounts
scatter3(mount_nodes(:,1), mount_nodes(:,2), mount_nodes(:,3), 80, 'b', 'filled');

for i = 1:8
    text(mount_nodes(i,1)+0.01, mount_nodes(i,2)+0.01, mount_nodes(i,3)+0.01, ...
         sprintf('%d', i), 'FontSize', 12, 'Color', 'b', 'FontWeight', 'bold');
end

% Draw lines connecting F & R points for FL (Left Side)
plot3([mount_nodes(1,1), mount_nodes(5,1)], [mount_nodes(1,2), mount_nodes(5,2)], [mount_nodes(1,3), mount_nodes(5,3)], 'r--', 'LineWidth', 1.5); % Lower FL
plot3([mount_nodes(3,1), mount_nodes(7,1)], [mount_nodes(3,2), mount_nodes(7,2)], [mount_nodes(3,3), mount_nodes(7,3)], 'g--', 'LineWidth', 1.5); % Upper FL

% Draw lines connecting F & R points for FR (Right Side)
plot3([mount_nodes(2,1), mount_nodes(6,1)], [mount_nodes(2,2), mount_nodes(6,2)], [mount_nodes(2,3), mount_nodes(6,3)], 'r--', 'LineWidth', 1.5); % Lower FR
plot3([mount_nodes(4,1), mount_nodes(8,1)], [mount_nodes(4,2), mount_nodes(8,2)], [mount_nodes(4,3), mount_nodes(8,3)], 'g--', 'LineWidth', 1.5); % Upper FR

% Extract Upright points
% 1: FL Lower, 2: FR Lower, 3: FL Upper, 4: FR Upper, 5: FL WC, 6: FR WC
scatter3(upright_nodes(:,1), upright_nodes(:,2), upright_nodes(:,3), 80, 'm', 'filled', 'square');

% Draw A-Arms FL
plot3([mount_nodes(1,1), upright_nodes(1,1)], [mount_nodes(1,2), upright_nodes(1,2)], [mount_nodes(1,3), upright_nodes(1,3)], 'r-', 'LineWidth', 2); % Lower F
plot3([mount_nodes(5,1), upright_nodes(1,1)], [mount_nodes(5,2), upright_nodes(1,2)], [mount_nodes(5,3), upright_nodes(1,3)], 'r-', 'LineWidth', 2); % Lower R
plot3([mount_nodes(3,1), upright_nodes(3,1)], [mount_nodes(3,2), upright_nodes(3,2)], [mount_nodes(3,3), upright_nodes(3,3)], 'g-', 'LineWidth', 2); % Upper F
plot3([mount_nodes(7,1), upright_nodes(3,1)], [mount_nodes(7,2), upright_nodes(3,2)], [mount_nodes(7,3), upright_nodes(3,3)], 'g-', 'LineWidth', 2); % Upper R

% Draw A-Arms FR
plot3([mount_nodes(2,1), upright_nodes(2,1)], [mount_nodes(2,2), upright_nodes(2,2)], [mount_nodes(2,3), upright_nodes(2,3)], 'r-', 'LineWidth', 2); % Lower F
plot3([mount_nodes(6,1), upright_nodes(2,1)], [mount_nodes(6,2), upright_nodes(2,2)], [mount_nodes(6,3), upright_nodes(2,3)], 'r-', 'LineWidth', 2); % Lower R
plot3([mount_nodes(4,1), upright_nodes(4,1)], [mount_nodes(4,2), upright_nodes(4,2)], [mount_nodes(4,3), upright_nodes(4,3)], 'g-', 'LineWidth', 2); % Upper F
plot3([mount_nodes(8,1), upright_nodes(4,1)], [mount_nodes(8,2), upright_nodes(4,2)], [mount_nodes(8,3), upright_nodes(4,3)], 'g-', 'LineWidth', 2); % Upper R

% Draw Wheel Centers
scatter3(upright_nodes(5:6,1), upright_nodes(5:6,2), upright_nodes(5:6,3), 100, 'k', 'filled', 'hexagram');

legend('Frame Reference', 'Ground', 'Chassis Mounts', 'Lower Inner Axis', 'Upper Inner Axis', 'Upright Node', 'Lower A-arm', '', 'Upper A-arm', '', '', '', '', '', 'Wheel Center', 'Location', 'best');
view(3); rotate3d on;

fprintf('\nRun this script in MATLAB to see the visualization: verify_suspension_geometry\n');
