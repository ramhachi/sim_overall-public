%% vehicle_3dof_model.m
% 3DOFプレーナービークルモデル（C++との検証用）
%
% 自由度: x, y, ψ（ヨー）
% 状態変数: [x, y, psi, vx, vy, r, omega(4), tire_xi(4), tire_eta(4)]

function [t, state, data] = vehicle_3dof_model(params, t_span, dt, input_func)
    % 初期状態
    % x, y, psi, vx, vy, r, omega[4], tire_xi[4], tire_eta[4]
    x0 = zeros(18, 1);
    x0(4) = params.v0;  % 初期縦速度
    for i = 1:4
        x0(6+i) = params.v0 / params.tire.radius;  % 初期車輪速度
    end
    
    % 静的荷重計算
    W = params.mass.m_sprung * 9.81;
    Wf = W * params.mass.b / params.mass.L;
    Wr = W * params.mass.a / params.mass.L;
    static_Fz = [Wf/2, Wf/2, Wr/2, Wr/2];
    
    % RK4積分
    t = t_span(1):dt:t_span(2);
    n_steps = length(t);
    state = zeros(n_steps, 18);
    state(1, :) = x0';
    
    % データ記録用
    data.Fx = zeros(n_steps, 4);
    data.Fy = zeros(n_steps, 4);
    data.ax = zeros(n_steps, 1);
    data.ay = zeros(n_steps, 1);
    
    for k = 1:(n_steps-1)
        x = state(k, :)';
        input = input_func(t(k));
        
        % RK4
        [k1, ~] = derivatives(x, input, params, static_Fz);
        [k2, ~] = derivatives(x + 0.5*dt*k1, input, params, static_Fz);
        [k3, ~] = derivatives(x + 0.5*dt*k2, input, params, static_Fz);
        [k4, forces] = derivatives(x + dt*k3, input, params, static_Fz);
        
        state(k+1, :) = (x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4))';
        
        % 力を記録
        data.Fx(k, :) = forces.Fx;
        data.Fy(k, :) = forces.Fy;
        data.ax(k) = forces.ax;
        data.ay(k) = forces.ay;
    end
    
    % 最終ステップのデータ
    data.Fx(end, :) = data.Fx(end-1, :);
    data.Fy(end, :) = data.Fy(end-1, :);
    data.ax(end) = data.ax(end-1);
    data.ay(end) = data.ay(end-1);
end

function [dxdt, forces] = derivatives(x, input, params, static_Fz)
    % 状態変数の展開
    % x(1:3) = [x, y, psi]
    % x(4:6) = [vx, vy, r]
    % x(7:10) = omega[4]
    % x(11:14) = tire_xi[4]
    % x(15:18) = tire_eta[4]
    
    psi = x(3);
    vx = x(4);
    vy = x(5);
    r = x(6);
    omega = x(7:10);
    tire_xi = x(11:14);
    tire_eta = x(15:18);
    
    % 車輪位置
    a = params.mass.a;
    b = params.mass.b;
    tf = params.mass.track_f / 2;
    tr = params.mass.track_r / 2;
    
    x_pos = [a, a, -b, -b];
    y_pos = [-tf, tf, -tr, tr];
    
    % ステアリング角（前輪同角で簡略化）
    delta = input.steering;
    steer = [delta, delta, 0, 0];
    
    % 各輪のタイヤ力計算
    Fx_wheel = zeros(1, 4);
    Fy_wheel = zeros(1, 4);
    dxi = zeros(4, 1);
    deta = zeros(4, 1);
    
    for i = 1:4
        % 接地点速度（車両座標系）
        vx_contact = vx - r * y_pos(i);
        vy_contact = vy + r * x_pos(i);
        
        % ホイール座標系に変換
        c = cos(steer(i));
        s = sin(steer(i));
        vx_wheel = c * vx_contact + s * vy_contact;
        vy_wheel = -s * vx_contact + c * vy_contact;
        
        % スリップ速度
        omega_r = omega(i) * params.tire.radius;
        vsx = vx_wheel - omega_r;  % 縦スリップ速度
        vsy = vy_wheel;            % 横スリップ速度
        vcx = abs(omega_r);        % 転がり速度
        
        % TMeasy力計算
        [Fx_tire, Fy_tire, dxi(i), deta(i)] = tmeasy_force(...
            tire_xi(i), tire_eta(i), static_Fz(i), vsx, vsy, max(vcx, 0.1), params.tire);
        
        % ホイール→車両座標変換
        Fx_wheel(i) = c * Fx_tire - s * Fy_tire;
        Fy_wheel(i) = s * Fx_tire + c * Fy_tire;
    end
    
    % 合計力とモーメント
    Fx_total = sum(Fx_wheel);
    Fy_total = sum(Fy_wheel);
    Mz_total = sum(x_pos .* Fy_wheel - y_pos .* Fx_wheel);
    
    % 空力抵抗
    v2 = vx^2 + vy^2;
    Fx_aero = -0.5 * params.aero.rho * v2 * params.aero.CD_A;
    Fx_total = Fx_total + Fx_aero;
    
    % 運動方程式
    m = params.mass.m_sprung;
    Izz = params.mass.Izz;
    
    ax = Fx_total / m + r * vy;
    ay = Fy_total / m - r * vx;
    dr = Mz_total / Izz;
    
    % 位置の微分（慣性座標系）
    c_psi = cos(psi);
    s_psi = sin(psi);
    dx = c_psi * vx - s_psi * vy;
    dy = s_psi * vx + c_psi * vy;
    dpsi = r;
    
    % 車輪回転の微分
    Iw = params.mass.Iw;
    R = params.tire.radius;
    
    engine_torque = input.throttle * params.powertrain.max_torque;
    drive_torque = engine_torque * params.powertrain.gear_ratio * params.powertrain.efficiency / 2;
    
    domega = zeros(4, 1);
    for i = 1:4
        if i >= 3  % 後輪駆動
            T_drive = drive_torque;
        else
            T_drive = 0;
        end
        
        % ホイール座標系でのFx
        c = cos(steer(i));
        s = sin(steer(i));
        Fx_w = c * Fx_wheel(i) + s * Fy_wheel(i);
        T_tire = -Fx_w * R;
        
        domega(i) = (T_drive + T_tire) / Iw;
    end
    
    % 出力
    dxdt = [dx; dy; dpsi; ax; ay; dr; domega; dxi; deta];
    
    forces.Fx = Fx_wheel;
    forces.Fy = Fy_wheel;
    forces.ax = ax;
    forces.ay = ay + vx * r;  % 遠心力補正
end

function [Fx, Fy, dxi, deta] = tmeasy_force(xi, eta, Fz, vsx, vsy, vcx, tire)
    % TMeasyタイヤモデル
    
    % タイヤパラメータ
    mu_x = tire.mu_x_max;
    mu_y = tire.mu_y_max;
    Cx = tire.Cx;
    Cy = tire.Cy;
    Lx = tire.Lx;
    Ly = tire.Ly;
    v_N = tire.v_N;
    
    % 変形量の時間微分
    dxi = vsx - (vcx / Lx) * xi;
    deta = vsy - (vcx / Ly) * eta;
    
    % 正規化変形量
    xi_norm = Cx * xi / (mu_x * Fz + 1);
    eta_norm = Cy * eta / (mu_y * Fz + 1);
    
    % 合成スリップ
    s = sqrt(xi_norm^2 + eta_norm^2);
    
    % 摩擦係数（飽和特性）
    if s < 1
        mu_eff = s * (2 - s);
    else
        mu_eff = 1;
    end
    
    % 力計算
    if s > 1e-6
        Fx = -mu_eff * mu_x * Fz * xi_norm / s;
        Fy = -mu_eff * mu_y * Fz * eta_norm / s;
    else
        Fx = 0;
        Fy = 0;
    end
end
