function params = generateRandomVehicleParams(bounds, strength, base_params)
    % GENERATERANDOMVEHICLEPARAMS ランダムな車両パラメータ構造体を生成
    %
    % params = generateRandomVehicleParams(bounds, strength)
    % params = generateRandomVehicleParams(bounds, strength, base_params)
    %
    % 入力:
    %   bounds   : getVehicleParamBounds() の出力構造体
    %   strength : 0.0（ベース近傍）〜 1.0（全範囲からサンプル）
    %
    % build_full_vehicle_design_schema.m の設計変数範囲を元に
    % Domain Randomization 用のランダムパラメータを生成する。
    ensure_rl_driver_paths();

    if nargin < 3 || isempty(base_params)
        params = create_vehicle_baseline_template();
    else
        params = base_params;
    end
    
    % Chassis
    params.mass.m_total = randInRange(bounds.mass.m_total, strength);
    
    front_dist = randInRange(bounds.mass.front_dist, strength);
    wb = randInRange(bounds.mass.wheelbase, strength);
    params.mass.wheelbase = wb;
    params.mass.b = wb * front_dist;
    params.mass.a = wb - params.mass.b;
    params.mass.h_cg = randInRange(bounds.mass.h_cg, strength);
    params.mass.track_f = randInRange(bounds.mass.track_f, strength);
    params.mass.track_r = randInRange(bounds.mass.track_r, strength);
    
    % 慣性モーメント（質量と寸法からスケーリング）
    scale_m = params.mass.m_total / 250.0;
    params.mass.Ixx = 32.0 * scale_m;
    params.mass.Iyy = 95.0 * scale_m;
    params.mass.Izz = 85.0 * scale_m;
    
    % Suspension
    k_f = randInRange(bounds.suspension.k_spring_f, strength);
    k_r = randInRange(bounds.suspension.k_spring_r, strength);
    params.suspension.k_spring = [k_f, k_f, k_r, k_r];
    params.suspension.c0 = mean([ ...
        randInRange(bounds.suspension.c0_f, strength), ...
        randInRange(bounds.suspension.c0_r, strength)]);
    params.suspension.k_arb_f = randInRange(bounds.suspension.k_arb_f, strength);
    params.suspension.k_arb_r = randInRange(bounds.suspension.k_arb_r, strength);
    params.suspension.motion_ratio = [ ...
        randInRange(bounds.suspension.motion_ratio_f, strength), ...
        randInRange(bounds.suspension.motion_ratio_f, strength), ...
        randInRange(bounds.suspension.motion_ratio_r, strength), ...
        randInRange(bounds.suspension.motion_ratio_r, strength)];
    params.suspension.camber_gain = [ ...
        randInRange(bounds.suspension.camber_gain_f, strength), ...
        randInRange(bounds.suspension.camber_gain_f, strength), ...
        randInRange(bounds.suspension.camber_gain_r, strength), ...
        randInRange(bounds.suspension.camber_gain_r, strength)];
    
    cam_f = randInRange(bounds.suspension.camber_f, strength) * pi/180;
    cam_r = randInRange(bounds.suspension.camber_r, strength) * pi/180;
    params.suspension.static_camber = [cam_f, cam_f, cam_r, cam_r];
    
    toe_f = randInRange(bounds.suspension.toe_f, strength) * pi/180;
    toe_r = randInRange(bounds.suspension.toe_r, strength) * pi/180;
    params.suspension.static_toe = [-toe_f, toe_f, -toe_r, toe_r];
    
    % Aero
    cl_a = randInRange(bounds.aero.CL_A, strength);
    params.aero.CL_A = cl_a;
    params.aero.CD_A = compute_aero_cd_a(cl_a, 'cd0', 0.30, 'k_induced', 0.10, 'power', 2.0);
    params.aero.aero_balance = randInRange(bounds.aero.aero_balance, strength);
    
    % Powertrain
    params.powertrain.lsd_preload = randInRange(bounds.powertrain.lsd_preload, strength);
    params.powertrain.lsd_ramp_accel = randInRange(bounds.powertrain.lsd_ramp_accel, strength);
    params.powertrain.lsd_ramp_decel = randInRange(bounds.powertrain.lsd_ramp_decel, strength);
    params.powertrain.final_ratio = randInRange(bounds.powertrain.final_ratio, strength);

    % Tire
    params.tire.mu_x_max = randInRange(bounds.tire.mu_x_max, strength);
    params.tire.mu_y_max = randInRange(bounds.tire.mu_y_max, strength);

    % Steering
    params.steering.stiffness = randInRange(bounds.steering.stiffness, strength);

    if isfield(bounds, 'extra_tunable_specs') && ~isempty(bounds.extra_tunable_specs)
        specs = bounds.extra_tunable_specs;
        x = zeros(1, numel(specs));
        for i = 1:numel(specs)
            specs(i).index = i;
            x(i) = randInRange([specs(i).lower, specs(i).upper], strength);
        end
        params = apply_vehicle_extra_tunables(params, x, specs);
    end
end

function val = randInRange(range, strength)
    mid = (range(1) + range(2)) / 2;
    half = (range(2) - range(1)) / 2;
    val = mid + (rand * 2 - 1) * half * strength;
    val = max(range(1), min(range(2), val));
end
