function bounds = get_rl_observation_bounds(vehicleParams)
% GET_RL_OBSERVATION_BOUNDS Shared RL observation bounds for MATLAB and export.

ensure_rl_driver_paths();

if nargin < 1
    vehicleParams = struct();
end

vehicle_bounds = getVehicleParamBounds();
vehicle_params = resolve_rl_vehicle_params(vehicleParams);
if vehicle_params.steering.torque_mode
    steer_bound = vehicle_params.steering.T_driver_max;
else
    steer_bound = vehicle_params.steering.max_angle;
end
cd_a_bounds = sort([
    compute_aero_cd_a(vehicle_bounds.aero.CL_A(1), 'cd0', 0.30, 'k_induced', 0.10, 'power', 2.0), ...
    compute_aero_cd_a(vehicle_bounds.aero.CL_A(2), 'cd0', 0.30, 'k_induced', 0.10, 'power', 2.0)
]);

bounds = [
    0, 40;       % 1: vx [m/s]
    -10, 10;     % 2: vy [m/s]
    -3, 3;       % 3: yaw rate [rad/s]
    -15, 15;     % 4: ax [m/s^2]
    -15, 15;     % 5: ay [m/s^2]
    -0.15, 0.15; % 6: roll [rad]
    -steer_bound, steer_bound; % 7: steering command input [Nm or rad]
    0, 1;        % 8: throttle
    0, 1;        % 9: brake
    -0.3, 0.3;   % 10: slip ratio FL
    -0.3, 0.3;   % 11: slip ratio FR
    -0.3, 0.3;   % 12: slip ratio RL
    -0.3, 0.3;   % 13: slip ratio RR
    -0.5, 0.5;   % 14: alpha_f [rad]
    -0.5, 0.5;   % 15: alpha_r [rad]
    -5, 5;       % 16: lateral deviation [m]
    -1, 1;       % 17: heading error [rad]
    -0.2, 0.2;   % 18: curvature now [1/m]
    -0.2, 0.2;   % 19: lookahead kappa 1
    -0.2, 0.2;   % 20: lookahead kappa 2
    -0.2, 0.2;   % 21: lookahead kappa 3
    -0.2, 0.2;   % 22: lookahead kappa 4
    -0.2, 0.2;   % 23: lookahead kappa 5
    0, 5;        % 24: dist to left edge [m]
    0, 5;        % 25: dist to right edge [m]
    0, 1;        % 26: lap progress
    1, 10;       % 27: lookahead width 1
    1, 10;       % 28: lookahead width 2
    1, 10;       % 29: lookahead width 3
    1, 10;       % 30: lookahead width 4
    -0.10, 0.10; % 31: frame twist [rad]
    -3.0, 3.0;   % 32: frame twist rate [rad/s]
    -3.0, 3.0;   % 33: steering wheel angle [rad]
    -20.0, 20.0; % 34: steering wheel rate [rad/s]
    vehicle_bounds.mass.m_total;           % 35: mass total [kg]
    vehicle_bounds.aero.CL_A;              % 36: CLA [m^2]
    cd_a_bounds;                           % 37: CDA [m^2]
    vehicle_bounds.tire.mu_y_max;          % 38: mu_y max [-]
    vehicle_bounds.tire.mu_x_max;          % 39: mu_x max [-]
    vehicle_bounds.suspension.k_spring_f;  % 40: front spring [N/m]
    vehicle_bounds.suspension.k_arb_f;     % 41: front ARB [Nm/rad]
    vehicle_bounds.steering.stiffness;     % 42: steering stiffness [Nm/rad]
    vehicle_bounds.suspension.K_chassis;   % 43: frame torsional stiffness [Nm/deg]
    vehicle_bounds.suspension.C_chassis;   % 44: frame damping [Nm*s/rad]
    vehicle_bounds.suspension.I_twist      % 45: frame twist inertia [kg*m^2]
];
end
