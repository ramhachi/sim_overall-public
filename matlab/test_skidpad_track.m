% Formula SAE Skidpad Track Verification v3
% FIXED: Each circle is FULL (2piR), not half (piR)
% Total track = 4piR
clear; close all; clc;

R = 9.125;  % Racing line radius
full_circ = 2 * pi * R;  % One full circle
n_points = 500;

% Total track: right circle (2piR) + left circle (2piR) = 4piR
s_vals = linspace(0, 2*full_circ, n_points);
x = zeros(size(s_vals));
y = zeros(size(s_vals));
theta = zeros(size(s_vals));
kappa = zeros(size(s_vals));

for i = 1:n_points
    s = s_vals(i);
    
    if s <= full_circ
        % Right circle: center (+R, 0), CLOCKWISE
        % Start (0,0) = angle alpha=pi from center
        % CW: alpha decreases from pi by s/R
        alpha = pi - s / R;   % pi -> pi-2pi = -pi
        x(i) = R + R * cos(alpha);
        y(i) = R * sin(alpha);
        theta(i) = alpha - pi/2;   % CW tangent direction
        kappa(i) = -1/R;
    else
        % Left circle: center (-R, 0), COUNTERCLOCKWISE
        % Start (0,0) = angle alpha=0 from center
        % CCW: alpha increases from 0 by s2/R
        s2 = s - full_circ;
        alpha = s2 / R;   % 0 -> 2pi
        x(i) = -R + R * cos(alpha);
        y(i) = R * sin(alpha);
        theta(i) = alpha + pi/2;   % CCW tangent direction
        kappa(i) = 1/R;
    end
end

% Verification output
fprintf('=== SAE Skidpad Track Verification v3 ===\n');
fprintf('Radius: %.3f m\n', R);
fprintf('Track length: %.3f m (= 4piR)\n', 2*full_circ);
fprintf('\n--- Key Points ---\n');
fprintf('Start (s=0):       x=%.4f, y=%.4f, theta=%.1f deg\n', x(1), y(1), rad2deg(theta(1)));

% Quarter of right circle (s = piR/2)
idx = round(n_points * (pi*R/2) / (2*full_circ));
fprintf('s=piR/2:           x=%.3f, y=%.3f [expect ~(R, R)=(%.3f, %.3f)]\n', x(idx), y(idx), R, R);

% Half of right circle (s = piR)
idx = round(n_points * (pi*R) / (2*full_circ));
fprintf('s=piR:             x=%.3f, y=%.3f [expect (2R, 0)=(%.3f, 0)]\n', x(idx), y(idx), 2*R);

% End of right circle (s = 2piR)
idx_cross = round(n_points * full_circ / (2*full_circ));
fprintf('s=2piR (cross):    x=%.4f, y=%.4f, theta=%.1f deg [expect (0, 0)]\n', ...
    x(idx_cross), y(idx_cross), rad2deg(theta(idx_cross)));

% Quarter of left circle
idx = round(n_points * (full_circ + pi*R/2) / (2*full_circ));
fprintf('s=2piR+piR/2:      x=%.3f, y=%.3f [expect ~(-R, R)=(%.3f, %.3f)]\n', x(idx), y(idx), -R, R);

% Half of left circle
idx = round(n_points * (full_circ + pi*R) / (2*full_circ));
fprintf('s=3piR:            x=%.3f, y=%.3f [expect (-2R, 0)=(%.3f, 0)]\n', x(idx), y(idx), -2*R);

% End
fprintf('End (s=4piR):      x=%.4f, y=%.4f, theta=%.1f deg [expect (0, 0)]\n', ...
    x(end), y(end), rad2deg(theta(end)));
fprintf('Closure error:     %.6f m\n', sqrt((x(end)-x(1))^2 + (y(end)-y(1))^2));

% Heading at crossing
fprintf('\n--- Heading Continuity at Self-Intersection ---\n');
th_before = theta(idx_cross);
th_after = theta(idx_cross + 1);
% Normalize
jump = abs(th_after - th_before);
if jump > pi, jump = 2*pi - jump; end
fprintf('Before: %.2f deg,  After: %.2f deg,  Jump: %.2f deg\n', ...
    rad2deg(th_before), rad2deg(th_after), rad2deg(jump));

% Verdict
start_err = sqrt(x(1)^2 + y(1)^2);
cross_err = sqrt(x(idx_cross)^2 + y(idx_cross)^2);
close_err = sqrt((x(end)-x(1))^2 + (y(end)-y(1))^2);

fprintf('\n=== RESULT ===\n');
if start_err < 0.1 && cross_err < 0.5 && close_err < 0.5 && rad2deg(jump) < 5
    fprintf('PASS: All checks passed!\n');
else
    fprintf('FAIL:\n');
    if start_err >= 0.1, fprintf('  Start error: %.4f m\n', start_err); end
    if cross_err >= 0.5, fprintf('  Crossing error: %.4f m\n', cross_err); end
    if close_err >= 0.5, fprintf('  Closure error: %.4f m\n', close_err); end
    if rad2deg(jump) >= 5, fprintf('  Heading jump: %.2f deg\n', rad2deg(jump)); end
end

% Plot
figure('Position', [100 100 1200 800]);
subplot(2,2,[1 3]);
hold on;

% Draw inner/outer boundaries
th = linspace(0, 2*pi, 200);
R_inner = 7.625;
R_outer = 10.625;
fill([R + R_outer*cos(th), R + R_inner*cos(fliplr(th))], ...
     [R_outer*sin(th), R_inner*sin(fliplr(th))], ...
     [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineStyle', '--');
fill([-R + R_outer*cos(th), -R + R_inner*cos(fliplr(th))], ...
     [R_outer*sin(th), R_inner*sin(fliplr(th))], ...
     [0.9 0.9 0.9], 'EdgeColor', 'k', 'LineStyle', '--');

% Track centerline
plot(x, y, 'r-', 'LineWidth', 2.5);
plot(x(1), y(1), 'go', 'MarkerSize', 15, 'LineWidth', 3, 'DisplayName', 'Start (0,0)');
plot(R, 0, 'b+', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', 'Right center');
plot(-R, 0, 'b+', 'MarkerSize', 20, 'LineWidth', 2, 'DisplayName', 'Left center');

axis equal; grid on;
title('SAE Skidpad Layout', 'FontSize', 14, 'FontWeight', 'bold');
xlabel('X [m]'); ylabel('Y [m]');
legend('Location','best');

subplot(2,2,2);
plot(s_vals, rad2deg(theta), 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Arc length s [m]'); ylabel('Heading [deg]');
title('Heading vs Arc Length');
xline(full_circ, 'r--', 'cross', 'LineWidth', 1.5);

subplot(2,2,4);
plot(s_vals, kappa, 'b-', 'LineWidth', 1.5);
grid on;
xlabel('Arc length s [m]'); ylabel('Curvature [1/m]');
title('Curvature Profile');
xline(full_circ, 'r--', 'cross', 'LineWidth', 1.5);
ylim([-0.15 0.15]);

saveas(gcf, 'skidpad_verification.png');
fprintf('Saved: skidpad_verification.png\n');
