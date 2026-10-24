data = readtable('../laptime_sim_ASE_autocross_verification.csv');
track = readtable('../data/ASE_autocross_sim.csv');

figure('Visible', 'off');
hold on; axis equal; grid on;

% Plot track points
plot(track.x, track.y, 'k.-', 'DisplayName', 'Track Waypoints');
plot(track.x(1), track.y(1), 'kv', 'MarkerSize', 10, 'DisplayName', 'Track Start');

% Plot vehicle trajectory
scatter(data.x, data.y, 20, data.speed, 'filled', 'DisplayName', 'Vehicle Path');
cb = colorbar;
ylabel(cb, 'Speed [m/s]');

% Start/End
plot(data.x(1), data.y(1), 'go', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Sim Start');
plot(data.x(end), data.y(end), 'rs', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', 'Sim End');

legend;
title('Debug Trajectory: ASE Autocross');
xlabel('X [m]'); ylabel('Y [m]');

saveas(gcf, 'debug_traj.png');
exit;
