% MATLAB Script to Visualize 14-DOF C++ Simulation Results

% --- 1. Load Data ---
% Load the simulation data from the CSV file
dataTable = readtable('../output/14dof_result.csv');

% Extract columns into variables for easier access
time = dataTable.time;
x = dataTable.x;
y = dataTable.y;
psi = dataTable.psi;
vx = dataTable.vx;
vy = dataTable.vy;
phi = dataTable.phi;
theta = dataTable.theta;
ax = dataTable.ax;
ay = dataTable.ay;
Fz_FL = dataTable.Fz_FL;
Fz_FR = dataTable.Fz_FR;
Fz_RL = dataTable.Fz_RL;
Fz_RR = dataTable.Fz_RR;

% --- 2. Create Visualizations ---
% Create a new figure window
figure('Name', '14-DOF C++ Simulation Results', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 800]);

% 2.1. Vehicle Trajectory (X-Y Plot)
subplot(2, 3, 1);
plot(y, x);
title('Vehicle Trajectory');
xlabel('Y Position (m)');
ylabel('X Position (m)');
grid on;
axis equal; % Ensure aspect ratio is 1:1

% 2.2. Longitudinal Velocity (vx) vs. Time
subplot(2, 3, 2);
plot(time, vx);
title('Longitudinal Velocity vs. Time');
xlabel('Time (s)');
ylabel('Velocity (m/s)');
grid on;

% 2.3. Yaw Angle (psi) vs. Time
subplot(2, 3, 3);
plot(time, rad2deg(psi)); % Convert radians to degrees for better intuition
title('Yaw Angle vs. Time');
xlabel('Time (s)');
ylabel('Yaw Angle (deg)');
grid on;

% 2.4. Lateral Acceleration (ay) vs. Time
subplot(2, 3, 4);
plot(time, ay);
title('Lateral Acceleration vs. Time');
xlabel('Time (s)');
ylabel('Lateral Accel (m/s^2)');
grid on;

% 2.5. Wheel Vertical Loads (Fz) vs. Time
subplot(2, 3, 5);
hold on;
plot(time, Fz_FL, 'DisplayName', 'FL');
plot(time, Fz_FR, 'DisplayName', 'FR');
plot(time, Fz_RL, 'DisplayName', 'RL');
plot(time, Fz_RR, 'DisplayName', 'RR');
hold off;
title('Vertical Wheel Loads vs. Time');
xlabel('Time (s)');
ylabel('Force (N)');
legend('show');
grid on;

% 2.6. Roll and Pitch Angles vs. Time
subplot(2, 3, 6);
hold on;
plot(time, rad2deg(phi), 'DisplayName', 'Roll (phi)');
plot(time, rad2deg(theta), 'DisplayName', 'Pitch (theta)');
hold off;
title('Roll & Pitch Angles vs. Time');
xlabel('Time (s)');
ylabel('Angle (deg)');
legend('show');
grid on;

% Add a main title to the figure
sgtitle('Visualization of C++ 14-DOF Simulation Results', 'FontSize', 16, 'FontWeight', 'bold');

% --- End of Script ---
