%% aero_map_4d.m
% 4次元エアロマップクラス
% 入力: RH_f, RH_r, roll, yaw
% 出力: CL, CD, CY, CoP_x, CoP_dist

classdef aero_map_4d < handle
    properties
        % パラメータ
        rho = 1.225      % 空気密度 [kg/m³]
        A_ref = 1.0      % 参照面積 [m²]
        L_ref = 1.6      % 参照長さ [m]
        x_cg = 0.8       % CG位置 [m from rear axle]
        
        % デフォルト係数
        CL_default = -2.5
        CD_default = 1.0
        CoP_default = 0.45
        
        % グリッド
        grid_rh_f
        grid_rh_r
        grid_roll
        grid_yaw
        
        % データ配列
        CL_data
        CD_data
        CY_data
        CoP_x_data
        CoP_dist_data
    end
    
    methods
        function obj = aero_map_4d()
            obj.init_default_grid();
            obj.init_sample_data();
        end
        
        function init_default_grid(obj)
            % フロントRH: 20-60mm
            obj.grid_rh_f = [20, 30, 40, 50, 60];
            % リアRH: 30-80mm
            obj.grid_rh_r = [30, 40, 50, 60, 70, 80];
            % ロール: -5deg〜+5deg
            obj.grid_roll = deg2rad([-5, -2.5, 0, 2.5, 5]);
            % ヨー: -15deg〜+15deg
            obj.grid_yaw = deg2rad([-15, -10, -5, 0, 5, 10, 15]);
            
            % データ配列確保
            n = [length(obj.grid_rh_f), length(obj.grid_rh_r), ...
                length(obj.grid_roll), length(obj.grid_yaw)];
            obj.CL_data = zeros(n);
            obj.CD_data = zeros(n);
            obj.CY_data = zeros(n);
            obj.CoP_x_data = zeros(n);
            obj.CoP_dist_data = 0.5 * ones(n);
        end
        
        function init_sample_data(obj)
            CL_base = obj.CL_default;
            CD_base = obj.CD_default;
            CoP_base = obj.CoP_default;
            
            for i = 1:length(obj.grid_rh_f)
                rh_f = obj.grid_rh_f(i);
                for j = 1:length(obj.grid_rh_r)
                    rh_r = obj.grid_rh_r(j);
                    for k = 1:length(obj.grid_roll)
                        roll = obj.grid_roll(k);
                        for l = 1:length(obj.grid_yaw)
                            yaw = obj.grid_yaw(l);
                            
                            % ライドハイト効果
                            rh_effect = 1 + 0.015 * (40 - rh_f) + 0.010 * (55 - rh_r);
                            rh_effect = max(0.6, min(1.4, rh_effect));
                            
                            % ロール効果
                            roll_loss = 1 - 3 * abs(roll);
                            roll_loss = max(0.7, min(1.0, roll_loss));
                            
                            % ヨー効果
                            yaw_effect = cos(yaw * 2);
                            yaw_effect = max(0.6, min(1.0, yaw_effect));
                            CY_val = sin(yaw * 2) * 0.5;
                            
                            % 係数計算
                            obj.CL_data(i,j,k,l) = CL_base * rh_effect * roll_loss * yaw_effect;
                            obj.CD_data(i,j,k,l) = CD_base * (1 + 2 * yaw^2);
                            obj.CY_data(i,j,k,l) = CY_val;
                            
                            % 圧力中心
                            CoP = CoP_base;
                            CoP = CoP + (rh_f - 40) * 0.003;
                            CoP = CoP - (rh_r - 55) * 0.002;
                            obj.CoP_x_data(i,j,k,l) = CoP;
                            
                            % 前後配分
                            dist = 0.5 + (rh_f - rh_r + 15) * 0.003;
                            obj.CoP_dist_data(i,j,k,l) = max(0.35, min(0.65, dist));
                        end
                    end
                end
            end
        end
        
        function coef = get_coeffs(obj, rh_f_mm, rh_r_mm, roll_rad, yaw_rad)
            % 4D補間で係数を取得
            coef.CL = obj.interp4d(obj.CL_data, rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
            coef.CD = obj.interp4d(obj.CD_data, rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
            coef.CY = obj.interp4d(obj.CY_data, rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
            coef.CoP_x = obj.interp4d(obj.CoP_x_data, rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
            coef.CoP_dist = obj.interp4d(obj.CoP_dist_data, rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
        end
        
        function forces = compute_forces(obj, velocity, rh_f_mm, rh_r_mm, roll_rad, yaw_rad)
            % 動圧
            q = 0.5 * obj.rho * velocity^2;
            
            % 係数取得
            coef = obj.get_coeffs(rh_f_mm, rh_r_mm, roll_rad, yaw_rad);
            
            % 力計算
            forces.Fx = -q * obj.A_ref * coef.CD;
            forces.Fy = q * obj.A_ref * coef.CY;
            forces.Fz = q * obj.A_ref * coef.CL;
            
            % モーメント
            lever = coef.CoP_x - obj.x_cg;
            forces.My = forces.Fz * lever;
            forces.Mz = forces.Fy * lever;
            
            % 前後配分
            forces.Fz_front = forces.Fz * coef.CoP_dist;
            forces.Fz_rear = forces.Fz * (1 - coef.CoP_dist);
        end
        
        function val = interp4d(obj, data, rh_f, rh_r, roll, yaw)
            % 4D線形補間
            val = interpn(obj.grid_rh_f, obj.grid_rh_r, obj.grid_roll, obj.grid_yaw, ...
                data, ...
                max(obj.grid_rh_f(1), min(obj.grid_rh_f(end), rh_f)), ...
                max(obj.grid_rh_r(1), min(obj.grid_rh_r(end), rh_r)), ...
                max(obj.grid_roll(1), min(obj.grid_roll(end), roll)), ...
                max(obj.grid_yaw(1), min(obj.grid_yaw(end), yaw)), ...
                'linear');
        end
    end
end
