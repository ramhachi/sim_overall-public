function bounds = getVehicleParamBounds()
    % GETVEHICLEPARAMBOUNDS Core RL vehicle-randomization bounds aligned to optimizer ranges.
    %
    % Domain Randomization で使用するパラメータ範囲定義。
    % build_full_vehicle_design_schema.m の canonical bounds に合わせる。

    this_dir = fileparts(mfilename('fullpath'));
    addpath(fileparts(this_dir));
    
    bounds = struct();
    
    % Chassis
    bounds.mass.m_total      = [250, 350];
    bounds.mass.front_dist   = [0.40, 0.60];
    bounds.mass.h_cg         = [0.25, 0.40];
    bounds.mass.wheelbase    = [1.53, 1.85];
    bounds.mass.track_f      = [1.10, 1.45];
    bounds.mass.track_r      = [1.10, 1.45];
    
    % Suspension
    bounds.suspension.k_spring_f = [10000, 90000];
    bounds.suspension.k_spring_r = [10000, 90000];
    bounds.suspension.c0_f       = [100, 3000];
    bounds.suspension.c0_r       = [100, 3000];
    bounds.suspension.k_arb_f    = [0, 5000];
    bounds.suspension.k_arb_r    = [0, 5000];
    bounds.suspension.camber_f   = [-10, 0];
    bounds.suspension.camber_r   = [-3, 0];
    bounds.suspension.toe_f      = [-3, 3];
    bounds.suspension.toe_r      = [-1.5, 1.5];
    bounds.suspension.motion_ratio_f = [0.50, 1.20];
    bounds.suspension.motion_ratio_r = [0.50, 1.20];
    bounds.suspension.camber_gain_f = [-1.0, 0.0];
    bounds.suspension.camber_gain_r = [-1.0, 0.0];
    bounds.suspension.K_chassis = [1000, 3500];
    bounds.suspension.C_chassis = [1.0, 20.0];
    bounds.suspension.I_twist = [0.5, 4.0];
    
    % Aero
    bounds.aero.CL_A = [0, 5.0];
    bounds.aero.aero_balance = [0.30, 0.70];
    
    % Powertrain
    bounds.powertrain.lsd_preload = [0, 200];
    bounds.powertrain.lsd_ramp_accel = [10, 60];
    bounds.powertrain.lsd_ramp_decel = [10, 80];
    bounds.powertrain.final_ratio = [3.0, 10.0];

    % Tire
    bounds.tire.mu_x_max = [1.1, 2.2];
    bounds.tire.mu_y_max = [1.3, 3.0];

    % Steering
    bounds.steering.stiffness = [1000, 30000];

    % Extended package tunables
    bounds.extra_tunable_specs = build_extended_vehicle_tunable_specs();
end
