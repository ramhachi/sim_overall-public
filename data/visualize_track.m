
% Visualize Track Data
clear; clc; close all;

% Load data
filename = 'ASE_autocross.csv';
if ~isfile(filename)
    error('File %s not found. Run extract_ac_track.py first.', filename);
end

data = readtable(filename);

% Extract columns
% AC Coordinates:
% X: Lateral/Longitudinal?
% Y: Vertical (Up)
% Z: Longitudinal/Lateral?
% Usually X and Z are the ground plane.

x = data.x;
y = data.y; % Height
z = data.z;
dist = data.dist;

figure('Name', 'Track Visualization');
plot(x, z, 'b-', 'LineWidth', 1.5);
hold on;
plot(x(1), z(1), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start
plot(x(end), z(end), 'rx', 'MarkerSize', 10, 'LineWidth', 2); % End
grid on;
axis equal;
xlabel('X [m]');
ylabel('Z [m]');
title('Track Geometry (Top Down)');
legend('Track Path', 'Start', 'End');

% Plot Height interpretation
figure('Name', 'Track Elevation');
plot(dist, y, 'r-', 'LineWidth', 1.5);
grid on;
xlabel('Distance [m]');
ylabel('Height (Y) [m]');
title('Track Elevation Profile');

% Plot Diff of distance to verify
diff_dist = diff(dist);
fprintf('Mean step distance: %.4f m\n', mean(diff_dist));
fprintf('Max step distance: %.4f m\n', max(diff_dist));
fprintf('Min step distance: %.4f m\n', min(diff_dist));

% Check total length
total_dist = dist(end);
fprintf('Total Track Length: %.2f m\n', total_dist);

% Save Plot
saveas(gcf, 'track_visualization.png');
disp('Saved track_visualization.png');
