% Quick test of figure-8 track generation
radius = 25;
kappa_ccw = 1.0 / radius;
half_circumference = pi * radius;
total_length = 2 * half_circumference;
n_points = 500;

% Numerical integration
s_vals = linspace(0, total_length, n_points);
x = zeros(size(s_vals));
y = zeros(size(s_vals));
theta = pi/2;  % North

% Initial position: (R, 0)
x(1) = radius;
y(1) = 0;

for i = 2:length(s_vals)
    ds = s_vals(i) - s_vals(i-1);
    theta = theta + kappa_ccw * ds;
    x(i) = x(i-1) + cos(theta) * ds;
    y(i) = y(i-1) + sin(theta) * ds;
end

% Close loop
x(end) = x(1);
y(end) = y(1);

% Plot
figure;
plot(x, y, 'r-', 'LineWidth', 2);
hold on;
plot(x(1), y(1), 'go', 'MarkerSize', 15);
plot(0, radius, 'b+', 'MarkerSize', 20);
plot(0, -radius, 'b+', 'MarkerSize', 20);
plot(0, 0, 'mx', 'MarkerSize', 20);
grid on;
axis equal;
title(sprintf('Figure-8 Track Test (R=%d m)', radius));
xlabel('X [m]');
ylabel('Y [m]');
legend('Track', 'Start (25,0)', 'Upper Center (0,25)', 'Lower Center (0,-25)', 'Self-Intersection (0,0)');

fprintf('Start: (%.2f, %.2f)\n', x(1), y(1));
fprintf('End: (%.2f, %.2f)\n', x(end), y(end));
fprintf('Midpoint (πR): (%.2f, %.2f)\n', x(floor(n_points/2)), y(floor(n_points/2)));
fprintf('Quarter (πR/2): (%.2f, %.2f)\n', x(floor(n_points/4)), y(floor(n_points/4)));
fprintf('3Quarter (3πR/2): (%.2f, %.2f)\n', x(floor(3*n_points/4)), y(floor(3*n_points/4)));
