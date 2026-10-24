function params = apply_tradeoff_design(base_params, x, config, design_context)
    % APPLY_TRADEOFF_DESIGN
    % ConceptTradeOffs.m の設定に基づき、離散選択肢(Aeroなど)と連続変数を適用する。
    %
    % Inputs:
    %   base_params: 基準構造体
    %   x: 設計変数ベクトル
    %   config: ConceptTradeOffs() で取得した設定構造体
    %
    % Variable Mapping (Custom for Discrete Support):
    % -- Shared Concept --
    % 1.  Mass Base [kg] (Chassis only)
    % 2.  Front Dist [%]
    % 3.  CG Height [m]
    % 4.  Wheelbase [m]
    % 5.  Track Width F [m]
    % 6.  Track Width R [m]
    % 7.  Final Drive Ratio [-]
    % 8.  [DISCRETE] Aero Package ID (1, 2, 3...)
    % 9+. Shared vehicle-package tunables (see build_extended_vehicle_tunable_specs)
    %
    % -- Tunable Setup (Skidpad / Accel) --
    % (8 + shared_tunable_count + 1) ... : Suspension & LSD & Aero Setting
    
    params = base_params;
    if nargin < 4 || isempty(design_context)
        design_context = struct();
    end

    shared_tunable_count = 0;
    if isfield(design_context, 'shared_extra_specs') && ~isempty(design_context.shared_extra_specs)
        shared_tunable_count = numel(design_context.shared_extra_specs);
    end
    tunable_offset = 8 + shared_tunable_count;

    %% 1. Decode Discrete Variables
    % Aero Package
    aero_id = round(x(8)); % Integer variable
    % 範囲外ガード
    aero_id = max(1, min(length(config.aero_packages), aero_id));
    aero_pkg = config.aero_packages(aero_id);
    
    %% 2. Chassis & Geometry (Shared)
    m_base = x(1);
    front_dist = x(2);
    h_cg = x(3);
    wb = x(4);
    track_f = x(5);
    track_r = x(6);
    final_drive = x(7);
    
    % --- Mass Calculation ---
    % Track growth adds both sprung structure and axle-local unsprung mass.
    base_track = 1.20;
    base_unsprung = [12, 12, 14, 14];
    sprung_penalty = 6.0;
    if isfield(config, 'track_width_sprung_penalty')
        sprung_penalty = config.track_width_sprung_penalty;
    end
    unsprung_penalty = 1.5;
    if isfield(config, 'track_width_unsprung_penalty')
        unsprung_penalty = config.track_width_unsprung_penalty;
    end
    [track_mass_penalty, m_unsprung] = compute_track_width_mass_distribution( ...
        base_unsprung, track_f, track_r, ...
        'base_track', base_track, ...
        'sprung_penalty_per_meter', sprung_penalty, ...
        'unsprung_penalty_per_meter', unsprung_penalty);

    mass_total = m_base + aero_pkg.mass_penalty + track_mass_penalty;
    
    params.mass.m_total = mass_total;
    
    % Unsprung Mass
    params.mass.m_unsprung = m_unsprung;
    params.mass.m_sprung = params.mass.m_total - sum(params.mass.m_unsprung);
    
    % CG & Distribution
    params.mass.wheelbase = wb;
    params.mass.b = wb * front_dist;
    params.mass.a = wb - params.mass.b;
    params.mass.h_cg = h_cg + (mass_total - m_base) * config.hcg_mass_sensitivity;
    params.mass.track_f = track_f;
    params.mass.track_r = track_r;
    
    % Inertia Scaling
    scale_mass = mass_total / 250.0;
    params.mass.Ixx = 30.0 * scale_mass * (mean([track_f, track_r])/1.2)^2;
    params.mass.Iyy = 90.0 * scale_mass * (wb/1.55)^2;
    params.mass.Izz = 80.0 * scale_mass;

    %% 3. Tunable Variables
    % 9:  Spring F
    % 10: Spring R
    % 11: Damper F
    % 12: Damper R
    % 13: ARB F
    % 14: ARB R
    % 15: Camber F
    % 16: Camber R
    % 17: Toe F
    % 18: Toe R
    % 19: LSD Preload
    % 20: LSD Ramp Acc
    % 21: LSD Ramp Dec
    % 22: Aero Fraction (0.0 - 1.0)
    % 23: Aero Balance Front Fraction
    
    params.suspension.k_spring = [x(tunable_offset + 1), x(tunable_offset + 1), x(tunable_offset + 2), x(tunable_offset + 2)];
    params.suspension.c0 = (x(tunable_offset + 3) + x(tunable_offset + 4)) / 2.0; % Avg
    params.suspension.k_arb_f = x(tunable_offset + 5);
    params.suspension.k_arb_r = x(tunable_offset + 6);
    
    params.suspension.static_camber = [x(tunable_offset + 7), x(tunable_offset + 7), x(tunable_offset + 8), x(tunable_offset + 8)] * (pi/180.0);
    params.suspension.static_toe    = [-x(tunable_offset + 9), x(tunable_offset + 9), -x(tunable_offset + 10), x(tunable_offset + 10)] * (pi/180.0);
    
    params.powertrain.lsd_preload = x(tunable_offset + 11);
    params.powertrain.lsd_ramp_accel = x(tunable_offset + 12);
    params.powertrain.lsd_ramp_decel = x(tunable_offset + 13);
    params.powertrain.final_ratio = final_drive;
    
    % Apply Custom Engine Map (if available)
    if isfield(config, 'engine_map')
        params.powertrain.engine_rpm_points = config.engine_map.rpm;
        params.powertrain.engine_torque_curve = config.engine_map.torque;
    end
    
    %% 4. Aero (Discrete + Tunable)
    aero_frac = x(tunable_offset + 14); % 0.0 to 1.0
    
    % Setup CL/CD based on selected package and fraction
    % CL = Max * Fraction
    % CD = Base + Induced * Fraction^2
    
    cl_val = aero_pkg.max_cl_a * aero_frac;
    
    % Induced Drag Model
    cd_val = compute_aero_cd_a(cl_val, 'cd0', aero_pkg.cd_a_base, 'k_induced', 0.10, 'power', 2.0);
    
    params.aero.CL_A = cl_val;
    params.aero.CD_A = cd_val;
    
    % Balance is independently tunable, but still constrained by package capability.
    aero_balance = front_dist;
    if length(x) >= tunable_offset + 15
        aero_balance = x(tunable_offset + 15);
    end
    params.aero.aero_balance = max(aero_pkg.balance_min, min(aero_pkg.balance_max, aero_balance));

    if isfield(design_context, 'shared_extra_specs') && ~isempty(design_context.shared_extra_specs)
        params = apply_vehicle_extra_tunables(params, x, design_context.shared_extra_specs);
    end
    
    %% 5. Tire
    if ~isfield(params, 'tire') || isempty(params.tire)
        params.tire = struct();
    end
    if ~isfield(params.tire, 'mu_y_max') || isempty(params.tire.mu_y_max)
        params.tire.mu_y_max = 2.0;
    end
    if ~isfield(params.tire, 'mu_x_max') || isempty(params.tire.mu_x_max)
        params.tire.mu_x_max = 1.6;
    end

end
