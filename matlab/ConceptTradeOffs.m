% ConceptTradeOffs.m
% 車両コンセプトのトレードオフ設定ファイル (ユーザー編集用)
% ここで「エアロの有無」「タイヤ変更」「エンジン変更」などの選択肢を定義します。

function config = ConceptTradeOffs()
    config = struct();

    %% 1. エアロパッケージ (Discrete Choice)
    % 選択肢: 1=None, 2=Light, 3=Heavy
    config.aero_packages = [
        % ID 1: None (No Wings)
        struct('name', 'None', ...
               'mass_penalty', 0.0, ...    % [kg]
               'max_cl_a',     0.1, ...    % [m^2] Body lift only
               'cd_a_base',    0.30, ...   % [m^2]
               'balance_min',  0.4, ...    % Front limit
               'balance_max',  0.6), ...   % Rear limit
               
        % ID 2: Light (Small Wings for Efficiency)
        struct('name', 'Light', ...
               'mass_penalty', 8.0, ...
               'max_cl_a',     1.5, ...
               'cd_a_base',    0.50, ...
               'balance_min',  0.3, ...
               'balance_max',  0.7), ...

        % ID 3: Heavy (Max Downforce for Skidpad)
        struct('name', 'Heavy', ...
               'mass_penalty', 20.0, ...
               'max_cl_a',     4.0, ...
               'cd_a_base',    0.70, ...
               'balance_min',  0.2, ...
               'balance_max',  0.8)
    ];

    %% 2. トレッド幅感度 (Continuous Trade-off)
    % トレッド拡幅でアーム/アップライト/ハブ周りが重くなる分を
    % sprung / unsprung に分けて見積もる。
    config.track_width_sprung_penalty = 6.0;    % [kg/m] axle-level sprung structure growth
    config.track_width_unsprung_penalty = 1.5;  % [kg/m] per-corner upright / arm growth
    % Legacy fallback fields (kept for compatibility with older scripts)
    config.track_width_penalty = config.track_width_sprung_penalty;
    config.unsprung_penalty = config.track_width_unsprung_penalty;
    
    % グリップ感度 (VelocityProfileが固定μのため、実効μとして補正)
    % 1m広げると有効摩擦係数がどれだけ上がるか
    config.mu_gain_per_meter = 0.5;    % +0.5 mu / +1.0 m width
    
    %% 3. 重心高感度
    % 質量増減に伴う重心高の変化係数 (簡易モデル)
    % 重くなると重心も少し上がる傾向などを入れる場合に利用
    config.hcg_mass_sensitivity = 0.0; % 0.0001 [m/kg] など

    %% 4. エンジンマップ設定 (Custom CSV)
    config.engine_map = struct();
    % CSV Data (test_04.CSV) - Smoothed Low End
    config.engine_map.rpm = [1000, 3000, 5000, 7000, 7100, 7200, 7300, 7400, 7500, 7600, 7700, 7800, 7900, 8000, 8100, 8200, 8300, 8400, 8500, 8600, 8700, 8800, 8900, 9000, 9100, 9200, 9300, 9400, 9500, 9600, 9700, 9800, 9900, 10000, 10100, 10200, 10300, 10400, 10500, 10600, 10700, 10800, 10900, 11000, 11100, 11200, 11300, 11400, 11500, 11600, 11700, 11800, 11900, 12000, 14000];
    config.engine_map.torque = [12.0, 13.0, 14.0, 14.6, 15.0, 16.1, 16.7, 17.8, 19.5, 22.9, 25.9, 29.0, 35.2, 38.6, 39.2, 39.5, 40.2, 40.7, 41.4, 42.1, 42.8, 43.4, 43.5, 43.4, 43.2, 43.3, 43.8, 44.2, 44.6, 44.8, 44.8, 44.7, 44.4, 44.2, 43.7, 43.3, 43.0, 42.5, 42.2, 41.8, 41.4, 41.0, 40.8, 40.6, 40.4, 39.9, 38.9, 38.0, 37.2, 36.6, 36.2, 36.1, 36.1, 36.2, 30.0];
    
end
