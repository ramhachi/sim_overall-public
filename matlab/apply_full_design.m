function params = apply_full_design(base_params, x, design_context)
    % APPLY_FULL_DESIGN 全設計変数（ジオメトリ、LSD、POU、DRS含む）からのパラメータ生成
    %
    % params = apply_full_design(base_params, x, design_context)
    %
    % Inputs:
    %   base_params: 基準構造体
    %   x: 設計変数ベクトル
    %
    % 設計変数リスト:
    % 1.  Mass Total [kg]
    % 2.  Front Dist [%] (0.0-1.0)
    % 3.  CG Height [m]
    % 4.  Wheelbase [m]
    % 5.  Track Width F [m]
    % 6.  Track Width R [m]
    % 7.  Spring F [N/m]
    % 8.  Spring R [N/m]
    % 9.  Damper F [Ns/m]
    % 10. Damper R [Ns/m]
    % 11. ARB F [Nm/rad]
    % 12. ARB R [Nm/rad]
    % 13. Camber F [deg]
    % 14. Camber R [deg]
    % 15. Toe F [deg]
    % 16. Toe R [deg]
    % 17. Aero CLA [m^2]
    % 18. LSD Preload [Nm]
    % 19. LSD Ramp Accel [deg]
    % 20. LSD Ramp Decel [deg]
    % 21. Final Drive Ratio [-]
    % 22. POU Jack C1 F [m/rad]
    % 23. POU Jack C2 F [m/rad²]
    % 24. POU Camber C1 F [rad/rad]
    % 25. POU Camber C2 F [rad/rad²]
    % 26. Steering Stiffness [Nm/rad]
    % 27. Motion Ratio F [-]
    % 28. Motion Ratio R [-]
    % 29. Camber Gain F [rad/m]
    % 30. Camber Gain R [rad/m]
    % 31. Chassis Torsional Stiffness [Nm/deg]
    % 32. Aero Balance Front Fraction [-]
    % 33+. Shared vehicle-package tunables (see build_extended_vehicle_tunable_specs)

    if nargin < 3 || isempty(design_context)
        design_context = struct();
    end

    params = base_params;
    base_mass = base_params.mass;

    %% 1. Chassis & Geometry
    m_base_target = x(1);
    front_dist = x(2);
    h_cg = x(3);
    wb = x(4);
    track_f = x(5);
    track_r = x(6);

    % トレッド拡大で増える重量は、サブフレーム側の sprung mass と、
    % アップライト/アーム側の unsprung mass を一貫した一つのモデルで見積もる。
    base_unsprung = [12, 12, 14, 14];
    [width_mass_penalty, m_unsprung] = compute_track_width_mass_distribution( ...
        base_unsprung, track_f, track_r);
    
    % トレードオフ: フレーム剛性向上による重量増
    % 基準剛性 1000 Nm/deg とし、それ以上なら 1000 Nm/deg あたり約 4.0 kg 増加と仮定
    chassis_stiffness = 1000; % Default
    if length(x) >= 31
        chassis_stiffness = x(31);
    end
    chassis_weight_penalty = max(0, (chassis_stiffness - 1000) / 1000.0 * 4.0);
    
    % 最終的な質量
    params.mass.m_total = m_base_target + width_mass_penalty + chassis_weight_penalty;
    
    % 重心位置計算
    params.mass.wheelbase = wb;
    params.mass.b = wb * front_dist;
    params.mass.a = wb - params.mass.b;
    params.mass.h_cg = h_cg;
    params.mass.track_f = track_f;
    params.mass.track_r = track_r;
    
    % 慣性モーメントスケーリング (重量と寸法に依存)
    % ベース車両の実測/設計慣性を基準に相似スケールする。絶対式で上書きすると
    % ベース設計ベクトルでもプリセットの慣性に戻らず、LPV ベースラインが変質する。
    base_track = mean([base_mass.track_f, base_mass.track_r]);
    if isfield(base_mass, 'Ixx') && isfield(base_mass, 'Iyy') && isfield(base_mass, 'Izz') ...
            && base_mass.m_total > 0 && base_mass.wheelbase > 0 && base_track > 0
        scale_mass = params.mass.m_total / base_mass.m_total;
        scale_wb = wb / base_mass.wheelbase;
        scale_track = mean([track_f, track_r]) / base_track;
        params.mass.Ixx = base_mass.Ixx * scale_mass * scale_track^2;
        params.mass.Iyy = base_mass.Iyy * scale_mass * scale_wb^2;
        params.mass.Izz = base_mass.Izz * scale_mass * ...
            (scale_wb^2 + scale_track^2) / 2.0;
    else
        scale_mass = params.mass.m_total / 250.0;
        scale_size = ((wb/1.55)^2 + (mean([track_f, track_r])/1.2)^2) / 2.0;
        params.mass.Ixx = 32.0 * scale_mass * (mean([track_f, track_r])/1.2)^2;
        params.mass.Iyy = 95.0 * scale_mass * (wb/1.55)^2;
        params.mass.Izz = 85.0 * scale_mass * scale_size;
    end

    % バネ下重量は track-width model と同じ一貫した見積もりを使う
    params.mass.m_unsprung = m_unsprung;
    params.mass.m_sprung = params.mass.m_total - sum(params.mass.m_unsprung);

    %% 2. Suspension
    params.suspension.k_spring = [x(7), x(7), x(8), x(8)];
    params.suspension.c0 = x(9);  % Front Damper (簡易的に前後同じor別変数化)
    % 配列インデックス対応: 9=F, 10=R と仮定して実装修正
    % Damper F/R
    % Bouc-Wenパラメータのc0は単一値だが、シミュレータが前後別に対応していない場合は平均orFront優先
    % ※ 現在のVehicleParams.hppは c0 が単一値。前後別にするならC++改修が必要。
    %    ここでは「平均値」または「Front値」として設定しておき、将来拡張に備える。
    params.suspension.c0 = (x(9) + x(10)) / 2.0; 
    
    params.suspension.k_arb_f = x(11);
    params.suspension.k_arb_r = x(12);
    
    % Alignment (deg -> rad)
    params.suspension.static_camber = [x(13), x(13), x(14), x(14)] * (pi/180.0);
    params.suspension.static_toe    = [-x(15), x(15), x(16), -x(16)] * (pi/180.0);
    
    %% 3. Aero (Trade-off)
    cl_a = x(17);
    cd_a = compute_aero_cd_a(cl_a, 'cd0', 0.30, 'k_induced', 0.10, 'power', 2.0);
    aero_balance = front_dist;
    if length(x) >= 32
        aero_balance = x(32);
    end
    
    params.aero.CL_A = cl_a;
    params.aero.CD_A = cd_a;
    params.aero.aero_balance = max(0.20, min(0.80, aero_balance));
    
    %% 4. Powertrain
    params.powertrain.lsd_preload = x(18);
    params.powertrain.lsd_ramp_accel = x(19);
    params.powertrain.lsd_ramp_decel = x(20);
    params.powertrain.final_ratio = x(21);
    
    % 5. Tire (Track Width & Load Sensitivity Proxy)
    if ~isfield(params, 'tire') || isempty(params.tire)
        params.tire = struct();
    end

    if isfield(design_context, 'optimize_tire_selection') && design_context.optimize_tire_selection ...
            && isfield(design_context, 'tire_candidates') && ~isempty(design_context.tire_candidates) ...
            && isfield(design_context, 'tire_var_index') && ~isempty(design_context.tire_var_index) ...
            && length(x) >= design_context.tire_var_index
        tire_index = round(x(design_context.tire_var_index));
        tire_index = min(max(tire_index, 1), numel(design_context.tire_candidates));
        tire_candidate = design_context.tire_candidates(tire_index);
        params.tire.model_type = 'pinn';
        params.tire.model_name = tire_candidate.name;
        params.tire.model_dir = tire_candidate.model_dir;
        params.tire.model_manifest = tire_candidate.model_manifest;
        params.tire.mu_x_max = tire_candidate.mu_x_max;
        params.tire.mu_y_max = tire_candidate.mu_y_max;
    end

    if ~isfield(params.tire, 'model_type') || isempty(params.tire.model_type)
        error('apply_full_design requires a deployable PINN tire model; TMeasy fallback is disabled.');
    end

    % トレッド幅効果は車両モデル側の荷重移動で既に表現されるため、
    % optimizer 側では μ を上乗せしない。
    base_mu_y = 2.0;
    if isfield(params.tire, 'mu_y_max') && ~isempty(params.tire.mu_y_max)
        base_mu_y = params.tire.mu_y_max;
    end
    base_mu_x = 1.6;
    if isfield(params.tire, 'mu_x_max') && ~isempty(params.tire.mu_x_max)
        base_mu_x = params.tire.mu_x_max;
    end
    params.tire.mu_y_max = base_mu_y;
    params.tire.mu_x_max = base_mu_x;

    %% 6. POU (Pushrod on Upright) Parameters
    % POU係数が25変数(POU4変数)に変更になった場合
    if length(x) >= 25
        pou_jack_c1_f = x(22);
        pou_jack_c2_f = x(23);
        pou_camber_c1_f = x(24);
        pou_camber_c2_f = x(25);
        
        % 左右対称に適用（C1は一次係数なので左右で符号反転）
        params.suspension.pou_jack_c1 = [pou_jack_c1_f, -pou_jack_c1_f, 0, 0];
        params.suspension.pou_jack_c2 = [pou_jack_c2_f,  pou_jack_c2_f, 0, 0];
        params.suspension.pou_camber_c1 = [pou_camber_c1_f, -pou_camber_c1_f, 0, 0];
        params.suspension.pou_camber_c2 = [pou_camber_c2_f,  pou_camber_c2_f, 0, 0];
    else
        % 後方互換性フォールバック
        params.suspension.pou_jack_c1 = [0, 0, 0, 0];
        params.suspension.pou_jack_c2 = [0, 0, 0, 0];
        params.suspension.pou_camber_c1 = [0, 0, 0, 0];
        params.suspension.pou_camber_c2 = [0, 0, 0, 0];
    end

    %% 7. Steering
    if length(x) >= 26
        params.steering.stiffness = x(26);
    end

    %% 7.5. Suspension Geometry Additions
    if length(x) >= 30
        mr_f = x(27);
        mr_r = x(28);
        cg_f = x(29);
        cg_r = x(30);
        
        params.suspension.motion_ratio = [mr_f, mr_f, mr_r, mr_r];
        params.suspension.camber_gain = [cg_f, cg_f, cg_r, cg_r];
    else
        % Default values if not provided
        params.suspension.motion_ratio = [0.85, 0.85, 0.90, 0.90];
        params.suspension.camber_gain = [-0.4, -0.4, -0.35, -0.35];
    end

    %% 8. Frame Flexibility
    if length(x) >= 31
        params.suspension.K_chassis = x(31);
    else
        params.suspension.K_chassis = 1e6; % Default to rigid
    end

    %% 8.5. Shared vehicle-package tunables
    if isfield(design_context, 'extra_tunable_specs') && ~isempty(design_context.extra_tunable_specs)
        params = apply_vehicle_extra_tunables(params, x, design_context.extra_tunable_specs);
    end

    %% 9. DRS (Drag Reduction System)
    % 空力効果（CL/CD修正）はイベント依存のため full_vehicle_cost 側で処理
    % ここでは設計変数として有効化した場合のみ、物理的トレードオフ（重量）を扱う
    if isfield(design_context, 'drs_var_index') && ~isempty(design_context.drs_var_index) ...
            && length(x) >= design_context.drs_var_index
        drs_enabled = round(x(design_context.drs_var_index));  % 0=搭載なし, 1=搭載あり

        if drs_enabled
            % ----- バネ上重量ペナルティ -----
            % DRS機構の内訳:
            %   - アクチュエータ (サーボモータ + リンケージ): ~1.0 kg
            %   - 改造リアウイング (フラップ + ヒンジ構造): ~0.8 kg
            %   - 配線・電子部品 (ECU, センサ類): ~0.7 kg
            %   合計: 2.5 kg (バネ上質量に加算)
            drs_weight_kg = 2.5;

            params.mass.m_total  = params.mass.m_total  + drs_weight_kg;
            params.mass.m_sprung = params.mass.m_sprung + drs_weight_kg;

            % ----- 慣性モーメント更新 -----
            % DRS機構の搭載位置: リアウイング (リアアクスル後方 +0.3m, CG上方 +0.5m)
            % 平行軸定理: I_new = I_old + m * r^2
            %   r_long = CGからリアアクスル後端までの距離
            %   r_vert = CG高からリアウイング高さまでの距離
            r_long = params.mass.a + 0.3;   % CGから後方への距離 [m] (a = 前軸→CG)
            r_vert = 0.5;                   % CG高より上方 [m] (ウイング高 ≈ CG+0.5m)

            % Iyy: 前後方向の質量オフセット → ピッチ慣性増加
            params.mass.Iyy = params.mass.Iyy + drs_weight_kg * (r_long^2 + r_vert^2);
            % Izz: 前後方向の質量オフセット → ヨー慣性増加 (後ろに重心が移動する方向)
            params.mass.Izz = params.mass.Izz + drs_weight_kg * r_long^2;
            % Ixx: 垂直方向の質量オフセット → ロール慣性増加
            params.mass.Ixx = params.mass.Ixx + drs_weight_kg * r_vert^2;
        end

        params.drs.enabled = logical(drs_enabled);
    elseif ~isfield(params, 'drs') || ~isfield(params.drs, 'enabled')
        params.drs.enabled = false;
    end

    params = apply_kc_static_alignment(params);
end
