classdef FSAEEnvironment < rl.env.MATLABEnvironment
    % FSAEENVIRONMENT RL Toolbox準拠の強化学習環境
    %
    % 14DOF車両シミュレータをDocker内TCPサーバー経由で制御する。
    %
    % 行動空間: 2次元 [steer_command(-1~1), accel(-1~1)]
    %   accel > 0 → スロットル, accel < 0 → ブレーキ
    %   同時踏み問題を根本的に排除
    %
    % 使用例:
    %   env = FSAEEnvironment('circle');
    %   obs = reset(env);
    %   [obs, reward, isDone, info] = step(env, [0.1; 0.5]);
    
    properties
        % シミュレーション設定
        ControlDt = 0.020       % 制御周期 [s] (50Hz)
        DeltaMax = 0.45         % 角度モード時の最大タイヤ舵角 [rad]
        MaxEpisodeSteps = 3000  % 最大ステップ数 (60秒分)
        
        TrackName = 'circle'
        TrackLength = 0         % トラック全長 [m]
        
        % 報酬重み（v8最適化: 速度+精度バランス）
        W_progress = 8.0        % 進行報酬（主要学習シグナル、大幅増加）
        W_speed = 0.5           % 速度ボーナス（増加: 速く走る動機）
        W_boundary = 30.0       % 境界ペナルティ（大幅増加: コース逸脱を強く罰す）
        W_centering = 2.0       % センターライン追従（増加: トラック中央を保つ）
        W_slip = 0.5            % スリップペナルティ
        W_spin = 15.0           % スピンペナルティ
        W_jerk = 0.08           % 操舵滑らかさ（増加: スムーズな操舵を促す）
        W_steerReversal = 0.6   % 急な切り返しペナルティ
        W_lap = 200.0           % ラップ完了ボーナス（大幅増加）
        W_heading = 2.0         % ヘディングペナルティ（増加: 進行方向の精度向上）
        W_alive = 0.2           % 生存ボーナス
        W_lowSpeedBrake = 0.0
        W_previewSteer = 0.0
        W_twist = 0.25
        W_steerRate = 0.02
        W_steerCommand = 0.02
        W_steerSaturation = 0.20
        TrackEdgePenaltyStartRatio = 0.5
        OffTrackMargin = 0.05
        OffTrackGraceSteps = 0
        StallMinSpeed = 1.0
        StallGraceSteps = 150
        CrawlWindowSteps = 0
        CrawlMinProgress = 0.0
        LowSpeedBrakeThreshold = 0.0
        PreviewSteerSpeedThreshold = 0.0
        PreviewCurvatureThreshold = 0.0
        RollingStartSpeed = 0.0
        RollingStartThrottle = 0.35
        RollingStartMaxSteps = 80
        RollingStartHoldEpisodes = 0
        RollingStartFadeEpisodes = 0
        SteerReversalDeadband = 0.12
        
        % TCP接続設定
        ServerHost = 'localhost'
        ServerPort = 22345
        
        % Domain Randomization設定
        DomainRandEnabled = false
        DomainRandInterval = 100  % Nエピソードごとにパラメータ変更
        DomainRandStrength = 0.3  % 0.0～1.0
        DomainRandBounds = []     % struct (getVehicleParamBounds出力)
        EpisodeIndexOffset = 0    % カリキュラム全体でのグローバルepisodeオフセット
        
        % トラックプール: 複数トラックからランダム選択
        TrackPool = {}            % cell array of track names (空なら固定トラック)

        % 車両諸元
        BaseVehicleParams = struct()
        CurrentVehicleParams = struct()
    end
    
    properties (SetAccess = private)
        Client          % FSAESimClient
        StepCount = 0
        PrevAction = [0; 0]
        PrevS = 0
        LapCompleted = false
        EpisodeCount = 0  % Domain Rand用エピソードカウント
        LastTerminationReason = 'running'
        RecentProgressHistory = zeros(0, 1)
        OffTrackCount = 0
    end
    
    methods
        function obj = FSAEEnvironment(trackName, varargin)
            this_dir = fileparts(mfilename('fullpath'));
            addpath(fileparts(this_dir));

            baseVehicleParams = resolve_rl_vehicle_params();
            if nargin > 1 && isstruct(varargin{1})
                baseVehicleParams = resolve_rl_vehicle_params(varargin{1});
            end

            obs_bounds = get_rl_observation_bounds(baseVehicleParams);
            obs_dim = size(obs_bounds, 1);

            % 観測空間: torque-mode + frame-aware observations, normalized to [-1, 1]
            obsInfo = rlNumericSpec([obs_dim 1], ...
                'LowerLimit', -ones(obs_dim,1), ...
                'UpperLimit',  ones(obs_dim,1));
            obsInfo.Name = 'observations';
            obsInfo.Description = '正規化された車両状態・トラック情報・車両諸元';
            
            % 行動空間: 2次元 [steer_command(-1~1), accel(-1~1)]
            actInfo = rlNumericSpec([2 1], ...
                'LowerLimit', [-1; -1], ...
                'UpperLimit', [ 1;  1]);
            actInfo.Name = 'actions';
            actInfo.Description = 'ステアリング・加減速';
            
            obj = obj@rl.env.MATLABEnvironment(obsInfo, actInfo);
            obj.TrackName = trackName;
            obj.ServerPort = get_rl_server_host_port();

            obj.BaseVehicleParams = baseVehicleParams;
            
            % TCPクライアント作成・接続
            obj.Client = FSAESimClient(obj.ServerHost, obj.ServerPort);
            obj.Client.connect();
            
            % 車両パラメータ
            obj.CurrentVehicleParams = obj.BaseVehicleParams;
            paramsJson = jsonencode(obj.CurrentVehicleParams);
            
            % 初期化
            obj.TrackLength = obj.Client.init(trackName, paramsJson);
            fprintf('FSAEEnvironment: track=%s, length=%.1fm\n', ...
                trackName, obj.TrackLength);
        end
        
        function [obs, reward, isDone, loggedSignals] = step(obj, action)
            % 2D行動をデコード → 3Dコマンド(steer-input, throttle, brake)に変換
            steerInput = obj.decodeSteeringAction(action(1));
            accel = max(-1, min(1, action(2)));
            if accel >= 0
                throttle = accel;
                brake = 0;
            else
                throttle = 0;
                brake = -accel;
            end
            
            % C++サーバーでステップ実行（プロトコルは3D: steer-input,throttle,brake）
            [rawObs, info] = obj.Client.step(steerInput, throttle, brake);
            obj.StepCount = obj.StepCount + 1;
            
            % NaN or Inf detection (prevent MATLAB training crash on simulator divergence)
            if any(isnan(rawObs)) || any(isinf(rawObs)) || any(isnan(info)) || any(isinf(info))
                isDone = true;
                terminationReason = 'spin';
                reward = -500.0;
                rawObs(isnan(rawObs) | isinf(rawObs)) = 0.0;
                info(isnan(info) | isinf(info)) = 0.0;
                lapJustCompleted = false;
            else
                [ds, lapJustCompleted] = obj.computeProgressDelta(info(4), info(1));
                obj.recordRecentProgress(ds);
                [isDone, terminationReason] = obj.checkTermination(info, lapJustCompleted);
                reward = obj.computeReward(rawObs, info, action, ds, lapJustCompleted, terminationReason);
            end
            
            % Clamp reward to prevent exploding gradients and network weight corruption
            reward = max(-1000.0, min(1000.0, reward));
            
            if lapJustCompleted
                obj.LapCompleted = true;
            end
            obj.LastTerminationReason = terminationReason;

            % 観測の正規化
            obs = obj.normalizeObservation(rawObs);
            
            % ログ
            loggedSignals.vx = info(1);
            loggedSignals.vy = info(2);
            loggedSignals.yawRate = info(3);
            loggedSignals.s = info(4);
            loggedSignals.lateralDev = info(5);
            loggedSignals.headingError = info(6);
            loggedSignals.distLeft = info(7) - info(5);
            loggedSignals.distRight = info(8) + info(5);
            loggedSignals.roll = info(9);
            loggedSignals.bodySlipAngle = info(10);
            loggedSignals.maxSlipAngle = info(11);
            loggedSignals.lapProgress = info(4) / obj.TrackLength;
            loggedSignals.reward = reward;
            loggedSignals.steerInput = rawObs(7);
            loggedSignals.throttle = rawObs(8);
            loggedSignals.brake = rawObs(9);
            loggedSignals.completedLap = obj.LapCompleted;
            
            obj.PrevAction = action;
            obj.PrevS = info(4);
        end
        
        function obs = reset(obj)
            obj.EpisodeCount = obj.EpisodeCount + 1;
            globalEpisode = obj.EpisodeIndexOffset + obj.EpisodeCount;
            
            % トラックプールからランダム選択
            needReinit = false;
            if ~isempty(obj.TrackPool)
                idx = randi(length(obj.TrackPool));
                newTrack = obj.TrackPool{idx};
                if ~strcmp(newTrack, obj.TrackName)
                    obj.TrackName = newTrack;
                    needReinit = true;
                end
            end
            
            % Domain Randomization or track change: re-init
            shouldRandomize = obj.DomainRandEnabled && ~isempty(obj.DomainRandBounds) ...
                && mod(globalEpisode - 1, obj.DomainRandInterval) == 0;

            if shouldRandomize
                obj.CurrentVehicleParams = generateRandomVehicleParams( ...
                    obj.DomainRandBounds, obj.DomainRandStrength, obj.BaseVehicleParams);
                veh_obs = get_rl_vehicle_observation_vector(obj.CurrentVehicleParams);
                fprintf(['FSAEEnvironment DR episode %d: m=%.1fkg wb=%.3fm CL_A=%.3f CD_A=%.3f ' ...
                    'mu_y=%.3f mu_x=%.3f kspring_f=%.1f k_arb_f=%.1f\n'], ...
                    globalEpisode, veh_obs(1), obj.CurrentVehicleParams.mass.wheelbase, ...
                    veh_obs(2), veh_obs(3), veh_obs(4), veh_obs(5), veh_obs(6), veh_obs(7));
                needReinit = true;
            end

            if needReinit
                paramsJson = jsonencode(obj.CurrentVehicleParams);
                obj.TrackLength = obj.Client.init(obj.TrackName, paramsJson);
            end
            
            rawObs = obj.Client.reset();
            prevS = 0;
            [rollingStartSpeed, rollingStartThrottle, rollingStartMaxSteps] = obj.getEffectiveRollingStart();
            if rollingStartSpeed > 0 && rollingStartThrottle > 0 && rollingStartMaxSteps > 0
                [rawObs, prevS] = obj.applyRollingStart(rawObs, rollingStartSpeed, rollingStartThrottle, rollingStartMaxSteps);
            end
            obs = obj.normalizeObservation(rawObs);
            
            obj.StepCount = 0;
            obj.PrevAction = [0; 0];
            obj.PrevS = prevS;
            obj.LapCompleted = false;
            obj.LastTerminationReason = 'running';
            obj.RecentProgressHistory = zeros(0, 1);
            obj.OffTrackCount = 0;
        end
        
        function delete(obj)
            if ~isempty(obj.Client)
                obj.Client.disconnect();
            end
        end
    end
    
    methods (Access = private)
        function reward = computeReward(obj, rawObs, info, action, ds, lapJustCompleted, terminationReason)
            % info: [vx, vy, r, s, n, d_heading, w_left, w_right, phi, bodySlip, maxSlipAngle, trackLength]
            vx = info(1);
            n = info(5);
            d_heading = info(6);
            w_left = info(7);
            w_right = info(8);
            yawRate = info(3);
            maxSlipAngle = info(11);
            phiTwist = rawObs(31);
            dphiTwist = rawObs(32);
            steeringWheelRate = rawObs(34);
            distLeft = w_left - n;
            distRight = w_right + n;
            
            halfWidth = (w_left + w_right) / 2;
            
            % === 正の報酬（行動を促す）===
            
            % 進行報酬: 主要な学習シグナル（方向一致ボーナス付き）
            cos_heading = max(0, cos(d_heading));
            R_progress = obj.W_progress * ds * cos_heading;
            
            % 速度ボーナス（前進を促す、ただし上限あり）
            R_speed = obj.W_speed * min(vx, 20) * obj.ControlDt;

            % 低速時のブレーキ偏り抑制
            brakeCmd = max(-action(2), 0.0);
            P_lowSpeedBrake = 0;
            if obj.W_lowSpeedBrake > 0 && obj.LowSpeedBrakeThreshold > 0 && vx < obj.LowSpeedBrakeThreshold
                speedDeficit = obj.LowSpeedBrakeThreshold - max(vx, 0.0);
                P_lowSpeedBrake = -obj.W_lowSpeedBrake * brakeCmd * speedDeficit * obj.ControlDt;
            end

            % 先読み曲率に対する操舵方向整合性
            R_previewSteer = 0;
            if obj.W_previewSteer > 0
                curvatureSamples = rawObs(18:22);
                curvatureRef = 0.35 * curvatureSamples(1) ...
                    + 0.25 * curvatureSamples(2) ...
                    + 0.18 * curvatureSamples(3) ...
                    + 0.12 * curvatureSamples(4) ...
                    + 0.10 * curvatureSamples(5);
                if abs(curvatureRef) >= obj.PreviewCurvatureThreshold ...
                        && (obj.PreviewSteerSpeedThreshold <= 0 || vx <= obj.PreviewSteerSpeedThreshold)
                    curvatureBound = 0.2;
                    % Under the current server steering convention, the
                    % reward-compatible steering sign is opposite the
                    % preview-curvature sign.
                    steerAlignment = -action(1) * max(-1.0, min(1.0, curvatureRef / curvatureBound));
                    R_previewSteer = obj.W_previewSteer * steerAlignment * obj.ControlDt;
                end
            end
            
            % センターライン追従: 距離に応じた段階的報酬
            if halfWidth > 0.1
                ratio = abs(n) / halfWidth;
                if ratio < 0.5
                    R_centering = obj.W_centering * (1.0 - ratio) * obj.ControlDt;
                else
                    R_centering = obj.W_centering * max(0, 1.0 - ratio) * 0.5 * obj.ControlDt;
                end
            else
                R_centering = 0;
            end
            
            % 生存ボーナス（時間でスケーリングして停車ハックを防ぐ）
            R_alive = obj.W_alive * obj.ControlDt;
            
            % ラップ完了ボーナス（速いほど高い報酬）
            R_lap = 0;
            if lapJustCompleted
                lapTime = obj.StepCount * obj.ControlDt;
                R_lap = obj.W_lap * (60.0 / max(lapTime, 1.0));
            end
            
            % === 負のペナルティ（危険行動を抑制）===
            
            % ヘディングエラーペナルティ（二乗）
            P_heading = -obj.W_heading * (d_heading / pi)^2;
            
            % 境界ペナルティ: 指数関数的増加（トラック端に近づくほど急激に増加、発散防止のためclamp）
            if halfWidth > 0.1
                boundary_ratio = abs(n) / halfWidth;
                if boundary_ratio > obj.TrackEdgePenaltyStartRatio
                    excess = min(5.0, boundary_ratio - obj.TrackEdgePenaltyStartRatio);
                    P_boundary = -obj.W_boundary * (exp(4.0 * excess) - 1.0) * obj.ControlDt;
                else
                    P_boundary = 0;
                end
                edge_clearance = min(distLeft, distRight);
                if edge_clearance < 0.25
                    P_boundary = P_boundary - obj.W_boundary * (0.25 - edge_clearance)^2;
                end
            else
                P_boundary = 0;
            end
            
            % スリップペナルティ
            slip_excess = max(0, abs(maxSlipAngle) - deg2rad(10));
            P_slip = -obj.W_slip * slip_excess;
            
            % 操舵滑らかさペナルティ（ジャーク + 大舵角ペナルティ）
            dSteer = action(1) - obj.PrevAction(1);
            P_jerk = -obj.W_jerk * (dSteer^2 + 0.1 * action(1)^2);

            P_steerReversal = 0;
            if obj.W_steerReversal > 0
                reversalDeadband = max(0.0, min(0.95, obj.SteerReversalDeadband));
                if abs(action(1)) > reversalDeadband && abs(obj.PrevAction(1)) > reversalDeadband ...
                        && action(1) * obj.PrevAction(1) < 0
                    reversalMagnitude = (abs(action(1)) - reversalDeadband) ...
                        * (abs(obj.PrevAction(1)) - reversalDeadband);
                    reversalDelta = abs(dSteer);
                    P_steerReversal = -obj.W_steerReversal * reversalMagnitude * reversalDelta;
                end
            end
            
            % スピンペナルティ
            P_spin = 0;
            if abs(yawRate) > 2.0
                P_spin = -obj.W_spin * (abs(yawRate) - 2.0);
            end

            P_twist = 0;
            if obj.W_twist > 0
                P_twist = -obj.W_twist * (abs(phiTwist) / 0.10)^2 ...
                        -0.25 * obj.W_twist * (abs(dphiTwist) / 3.0)^2;
            end

            P_steerRate = 0;
            if obj.W_steerRate > 0
                P_steerRate = -obj.W_steerRate * (abs(steeringWheelRate) / 20.0)^2;
            end

            P_steerCommand = 0;
            if obj.W_steerCommand > 0
                steerAbs = abs(action(1));
                P_steerCommand = -obj.W_steerCommand * steerAbs^2;
                satExcess = max(0, steerAbs - 0.90) / 0.10;
                P_steerCommand = P_steerCommand - obj.W_steerSaturation * satExcess^2;
            end

            P_terminal = obj.terminalPenalty(terminationReason);
            
            reward = R_progress + R_speed + R_centering + R_alive + R_lap + R_previewSteer ...
                   + P_heading + P_boundary + P_slip + P_jerk + P_steerReversal ...
                   + P_spin + P_twist + P_steerRate + P_steerCommand ...
                   + P_lowSpeedBrake + P_terminal;
        end
        
        function [isDone, terminationReason] = checkTermination(obj, info, lapJustCompleted)
            vx = info(1);
            n = info(5);
            yawRate = info(3);
            bodySlipAngle = info(10);
            w_left = info(7);
            w_right = info(8);
            distLeft = w_left - n;
            distRight = w_right + n;
            
            % スピン検出: yawRate + bodySlipAngle
            isSpun = abs(yawRate) > 3.0 && abs(bodySlipAngle) > deg2rad(45);
            
            % コース逸脱。OffTrackGraceSteps > 0 の場合は laptime_sim と同じく
            % 短時間の逸脱を即失敗にせず、recovery controller が戻す余地を残す。
            isBeyondTrack = distLeft <= obj.OffTrackMargin || distRight <= obj.OffTrackMargin;
            if isBeyondTrack
                obj.OffTrackCount = obj.OffTrackCount + 1;
            else
                obj.OffTrackCount = 0;
            end
            isOffTrack = obj.OffTrackCount > max(0, round(obj.OffTrackGraceSteps));
            
            % 後退
            isReversing = vx < -1.0;
            
            % 低速停滞（3秒以上進めていない場合）
            stallProgressThreshold = min(20.0, 0.05 * obj.TrackLength);
            isStopped = abs(vx) < obj.StallMinSpeed && obj.StepCount >= obj.StallGraceSteps ...
                && info(4) < stallProgressThreshold;

            % 低速クロール: 一度だけ前進して stall 判定を回避する抜け道を防ぐ
            isCrawling = false;
            if obj.CrawlWindowSteps > 0 && obj.CrawlMinProgress > 0 ...
                    && obj.StepCount >= max(obj.StallGraceSteps, obj.CrawlWindowSteps) ...
                    && numel(obj.RecentProgressHistory) >= obj.CrawlWindowSteps
                recentProgress = sum(obj.RecentProgressHistory(end-obj.CrawlWindowSteps+1:end));
                isCrawling = recentProgress < obj.CrawlMinProgress;
            end
            
            % 最大ステップ超過
            isTimeout = obj.StepCount >= obj.MaxEpisodeSteps;
            
            if lapJustCompleted
                terminationReason = 'lap';
            elseif isOffTrack
                terminationReason = 'offtrack';
            elseif isSpun
                terminationReason = 'spin';
            elseif isReversing
                terminationReason = 'reverse';
            elseif isStopped || isCrawling
                terminationReason = 'stall';
            elseif isTimeout
                terminationReason = 'timeout';
            else
                terminationReason = 'running';
            end

            isDone = ~strcmp(terminationReason, 'running');
        end

        function [ds, lapJustCompleted] = computeProgressDelta(obj, s, vx)
            ds = s - obj.PrevS;
            lapJustCompleted = ds < -obj.TrackLength / 2 && vx >= 0.0;
            if ds < -obj.TrackLength / 2
                ds = ds + obj.TrackLength;
            elseif ds > obj.TrackLength / 2
                ds = ds - obj.TrackLength;
            end
        end

        function penalty = terminalPenalty(~, terminationReason)
            switch terminationReason
                case 'offtrack'
                    penalty = -200.0;
                case 'spin'
                    penalty = -150.0;
                case 'reverse'
                    penalty = -100.0;
                case 'stall'
                    penalty = -120.0;
                case 'timeout'
                    penalty = -80.0;
                otherwise
                    penalty = 0.0;
            end
        end

        function steerInput = decodeSteeringAction(obj, steerAction)
            steerAction = max(-1.0, min(1.0, steerAction));
            if isfield(obj.CurrentVehicleParams, 'steering') && ...
                    isfield(obj.CurrentVehicleParams.steering, 'torque_mode') && ...
                    obj.CurrentVehicleParams.steering.torque_mode
                torqueMax = obj.CurrentVehicleParams.steering.T_driver_max;
                steerInput = steerAction * torqueMax;
            else
                maxAngle = obj.DeltaMax;
                if isfield(obj.CurrentVehicleParams, 'steering') && isfield(obj.CurrentVehicleParams.steering, 'max_angle')
                    maxAngle = obj.CurrentVehicleParams.steering.max_angle;
                end
                steerInput = steerAction * maxAngle;
            end
        end
        
        function obs = normalizeObservation(obj, raw)
            bounds = get_rl_observation_bounds(obj.CurrentVehicleParams);
            raw_vehicle = get_rl_vehicle_observation_vector(obj.CurrentVehicleParams);
            raw_full = [raw(:); raw_vehicle(:)];
            obs = 2 * (raw_full - bounds(:,1)) ./ (bounds(:,2) - bounds(:,1)) - 1;
            obs = max(-1, min(1, obs));
        end

        function recordRecentProgress(obj, ds)
            if obj.CrawlWindowSteps <= 0
                return;
            end

            obj.RecentProgressHistory(end + 1, 1) = ds;
            maxHistory = max(obj.CrawlWindowSteps, obj.StallGraceSteps);
            if numel(obj.RecentProgressHistory) > maxHistory
                obj.RecentProgressHistory = obj.RecentProgressHistory(end-maxHistory+1:end);
            end
        end

        function [speedTarget, throttle, maxSteps] = getEffectiveRollingStart(obj)
            speedTarget = max(0.0, obj.RollingStartSpeed);
            throttle = max(0.0, min(1.0, obj.RollingStartThrottle));
            maxSteps = max(0, round(obj.RollingStartMaxSteps));
            if speedTarget <= 0 || throttle <= 0 || maxSteps <= 0
                speedTarget = 0.0;
                throttle = 0.0;
                maxSteps = 0;
                return;
            end

            holdEpisodes = max(0, round(obj.RollingStartHoldEpisodes));
            fadeEpisodes = max(0, round(obj.RollingStartFadeEpisodes));
            if obj.EpisodeCount <= holdEpisodes
                scale = 1.0;
            elseif fadeEpisodes <= 0
                scale = 0.0;
            elseif obj.EpisodeCount > holdEpisodes + fadeEpisodes
                scale = 0.0;
            else
                remainingFadeEpisodes = holdEpisodes + fadeEpisodes - obj.EpisodeCount;
                scale = double(remainingFadeEpisodes) / double(fadeEpisodes);
            end

            scale = max(0.0, min(1.0, scale));
            speedTarget = speedTarget * scale;
            throttle = throttle * scale;
            maxSteps = max(0, ceil(maxSteps * scale));
            if speedTarget <= 0 || throttle <= 0 || maxSteps <= 0
                speedTarget = 0.0;
                throttle = 0.0;
                maxSteps = 0;
            end
        end

        function [rawObs, currentS] = applyRollingStart(obj, rawObs, speedTarget, throttle, maxSteps)
            currentS = 0.0;
            for warmStep = 1:maxSteps
                [rawObs, info] = obj.Client.step(0.0, throttle, 0.0);
                currentS = info(4);
                vx = info(1);
                distLeft = info(7) - info(5);
                distRight = info(8) + info(5);
                if vx >= speedTarget
                    break;
                end
                if distLeft <= obj.OffTrackMargin || distRight <= obj.OffTrackMargin
                    break;
                end
            end
        end
    end
end
