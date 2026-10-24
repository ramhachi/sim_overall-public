function geom = define_suspension_geometry(wheel_index, vehicle_params, allow_missing_tie_rod)
% DEFINE_SUSPENSION_GEOMETRY - Defines double-wishbone suspension hardpoints
%
% Full double-wishbone model: each A-arm has front/rear chassis mounts + upright
% ball joint (3 points per arm). Tie rod connects rack to upright steering arm.
% Kingpin axis is DERIVED from upper/lower ball joints (not hardcoded).
%
% INPUTS:
%   wheel_index    - 0=FL, 1=FR, 2=RL, 3=RR
%   vehicle_params - (optional) VehicleParams struct from JSON
%
% OUTPUTS:
%   geom - Struct containing (10 hardpoints per wheel):
%     Upper A-arm:
%       .upper_arm_front_chassis  [3x1] front chassis mount
%       .upper_arm_rear_chassis   [3x1] rear chassis mount
%       .upper_arm_upright        [3x1] upright ball joint
%     Lower A-arm:
%       .lower_arm_front_chassis  [3x1] front chassis mount
%       .lower_arm_rear_chassis   [3x1] rear chassis mount
%       .lower_arm_upright        [3x1] upright ball joint
%     Tie rod:
%       .tie_rod_chassis          [3x1] rack-side (inner)
%       .tie_rod_upright          [3x1] upright-side (outer)
%     Derived quantities:
%       .upper_arm_axis           [3x1] A-arm rotation axis (unit vector)
%       .upper_arm_axis_origin    [3x1] point on rotation axis
%       .lower_arm_axis           [3x1]
%       .lower_arm_axis_origin    [3x1]
%       .kingpin_axis.origin      [3x1] lower BJ position
%       .kingpin_axis.direction   [3x1] L_U -> U_U direction (unit)
%       .kingpin_length           scalar |U_U - L_U|
%
% COORDINATE SYSTEM: SAE J670e (X forward, Y right, Z down)

    if nargin < 1
        wheel_index = 0;
    end

    % Get basic vehicle dimensions
    if nargin >= 2
        if isfield(vehicle_params, 'params') && isfield(vehicle_params.params, 'mass')
            mass_params = vehicle_params.params.mass;
        elseif isfield(vehicle_params, 'mass')
            mass_params = vehicle_params.mass;
        else
            mass_params = struct();
        end

        if isfield(mass_params, 'wheelbase')
            wheelbase = mass_params.wheelbase;
        else
            wheelbase = 1.535;
        end
        if isfield(mass_params, 'track_f')
            track_f = mass_params.track_f;
        else
            track_f = 1.200;
        end
        if isfield(mass_params, 'track_r')
            track_r = mass_params.track_r;
        else
            track_r = 1.100;
        end
    else
        wheelbase = 1.535;
        track_f = 1.200;
        track_r = 1.100;
    end

    % Determine wheel position
    is_front = (wheel_index < 2);
    is_left = (mod(wheel_index, 2) == 0);

    if is_front
        track = track_f;
    else
        track = track_r;
    end

    if is_left
        y_sign = -1;
    else
        y_sign = 1;
    end
    y_wheel = y_sign * track / 2;

    if is_front
        x_wheel = 0.60;
    else
        x_wheel = -0.93;
    end

    geom.wheel_index = wheel_index;
    geom.is_front = is_front;
    geom.is_left = is_left;
    geom.x_wheel = x_wheel;
    geom.y_wheel = y_wheel;
    geom.z_wheel = -0.228;

    % ========================================
    % LAYER 1&2 INTEGRATION: Read from artifact params.suspension.kc if available
    % ========================================
    if nargin < 3 || isempty(allow_missing_tie_rod)
        allow_missing_tie_rod = false;
    end
    use_json_geom = false;
    if nargin >= 2
        if isfield(vehicle_params, 'params')
            params_root = vehicle_params.params;
        else
            params_root = vehicle_params;
        end

        if isfield(params_root, 'suspension') && isfield(params_root.suspension, 'kc') ...
                && isfield(params_root.suspension.kc, 'corners')
            wheel_names = {'FL', 'FR', 'RL', 'RR'};
            wn = wheel_names{wheel_index + 1};
            corners = params_root.suspension.kc.corners;
            if isfield(corners, wn) && isfield(corners.(wn), 'hardpoints')
                corner_meta = corners.(wn);
                validate_corner_quality(corner_meta, wheel_index, allow_missing_tie_rod);
                hp = corners.(wn).hardpoints;
                if isfield(hp, 'wheel_center')
                    x_wheel = hp.wheel_center(1);
                    y_wheel = hp.wheel_center(2);
                    geom.x_wheel = x_wheel;
                    geom.y_wheel = y_wheel;
                    if numel(hp.wheel_center) >= 3
                        geom.z_wheel = hp.wheel_center(3);
                    end
                end

                geom.upper_arm_front_chassis = hp.upper_arm_chassis_fore(:);
                geom.upper_arm_rear_chassis  = hp.upper_arm_chassis_aft(:);
                geom.lower_arm_front_chassis = hp.lower_arm_chassis_fore(:);
                geom.lower_arm_rear_chassis  = hp.lower_arm_chassis_aft(:);
                geom.upper_arm_upright       = hp.upper_ball_joint(:);
                geom.lower_arm_upright       = hp.lower_ball_joint(:);

                if ~isfield(hp, 'tie_rod_inner') || ~isfield(hp, 'tie_rod_outer') ...
                        || ~any(hp.tie_rod_inner(:) ~= 0) || ~any(hp.tie_rod_outer(:) ~= 0)
                    if ~allow_missing_tie_rod
                        error('POU:MissingTieRodHardpoints', ...
                            ['Vehicle artifact has hardpoints for wheel %d but tie_rod_inner/outer ', ...
                             'are missing or zero. Bump steer cannot be evaluated; rerun the ', ...
                             'steering/A-arm stage or set options.allow_missing_tie_rod=true explicitly.'], ...
                            wheel_index);
                    end
                    warning('POU:MissingTieRodHardpoints', ...
                        'Allowing missing tie-rod hardpoints because allow_missing_tie_rod=true.');
                    geom.tie_rod_chassis = [0; 0; 0];
                    geom.tie_rod_upright = [0; 0; 0];
                else
                    geom.tie_rod_chassis = hp.tie_rod_inner(:);
                    geom.tie_rod_upright = hp.tie_rod_outer(:);
                end

                if isfield(corner_meta, 'evaluation_scope')
                    geom.evaluation_scope = corner_meta.evaluation_scope;
                end
                if isfield(corner_meta, 'quality')
                    geom.quality = corner_meta.quality;
                end
                if isfield(corner_meta, 'source')
                    geom.source = corner_meta.source;
                end
                use_json_geom = true;
                fprintf('Loaded suspension KC hardpoints from artifact for Wheel %d\n', wheel_index);
            end
        end
    end
    
    if ~use_json_geom && nargin >= 2 && ~isempty(vehicle_params)
        error('POU:MissingSuspensionHardpoints', ...
            ['Vehicle artifact does not contain complete suspension KC hardpoints for wheel %d. ', ...
             'Run suspension_kinematics_design and save_suspension_geometry_artifact first.'], wheel_index);
    end

    if ~use_json_geom
        % ========================================
        % UPPER A-ARM (3 points) HARDCODED FALLBACK
        % ========================================
        geom.upper_arm_front_chassis = [x_wheel - 0.10; y_sign * 0.30; -0.22];
        geom.upper_arm_rear_chassis  = [x_wheel + 0.10; y_sign * 0.30; -0.20];
        geom.upper_arm_upright       = [x_wheel - 0.013; y_wheel - y_sign * 0.030; -0.15];
    
        % ========================================
        % LOWER A-ARM (3 points) HARDCODED FALLBACK
        % ========================================
        geom.lower_arm_front_chassis = [x_wheel - 0.12; y_sign * 0.32; 0.03];
        geom.lower_arm_rear_chassis  = [x_wheel + 0.12; y_sign * 0.32; 0.05];
        geom.lower_arm_upright       = [x_wheel; y_wheel; 0.10];
    
        % ========================================
        % TIE ROD (2 points) HARDCODED FALLBACK
        % WARNING: This means Layer 1.5 was not run!
        % ========================================
        geom.tie_rod_chassis = [x_wheel - 0.05; y_sign * 0.35; -0.05];
        geom.tie_rod_upright = [x_wheel - 0.005; y_wheel - y_sign * 0.015; -0.05];
    end

    % ========================================
    % DERIVED: A-arm rotation axes
    % ========================================
    % Upper A-arm axis = line through front and rear chassis mounts
    upper_axis_vec = geom.upper_arm_rear_chassis - geom.upper_arm_front_chassis;
    geom.upper_arm_axis = upper_axis_vec / norm(upper_axis_vec);
    geom.upper_arm_axis_origin = geom.upper_arm_front_chassis;

    % Lower A-arm axis
    lower_axis_vec = geom.lower_arm_rear_chassis - geom.lower_arm_front_chassis;
    geom.lower_arm_axis = lower_axis_vec / norm(lower_axis_vec);
    geom.lower_arm_axis_origin = geom.lower_arm_front_chassis;

    % ========================================
    % DERIVED: Kingpin axis (from ball joints)
    % ========================================
    kp_vec = geom.upper_arm_upright - geom.lower_arm_upright;
    geom.kingpin_length = norm(kp_vec);
    geom.kingpin_axis.origin = geom.lower_arm_upright;
    geom.kingpin_axis.direction = kp_vec / geom.kingpin_length;

    % ========================================
    % WHEEL RIM GEOMETRY
    % ========================================
    geom.rim_radius = 0.254 / 2;       % 10" rim
    geom.rim_clearance_min = 0.005;     % 5mm

    % Brake caliper envelope (hub height, offset outboard)
    geom.caliper.center = [x_wheel; y_wheel + y_sign * 0.10; geom.z_wheel];
    geom.caliper.radius = 0.050;

    % ========================================
    % PUSHROD CONSTRAINTS
    % ========================================
    geom.pushrod_length_min = 0.08;   % Relaxed from 0.10
    geom.pushrod_length_max = 0.90;   % Relaxed to allow for large camber stroke
    geom.rod_end_angle_max = deg2rad(60);  % Relaxed heavily to allow convergence with active targets

    % ========================================
    % DESIGN VARIABLE BOUNDS & CONSTRAINTS
    % ========================================
    % User Configurable Params:
    geom.max_upright_radius = 0.20; % [m] Maximum distance of upright arm connections from wheel center
    geom.min_inboard_clearance = 0.05; % [m] Min clearance inboard from tire center plane for M-pivot
    geom.min_bellcrank_arm_clearance = 0.10; % [m] Min clearance from bellcrank pivot to any A-arm chassis mount
    
    % M-pivot absolute coordinates (chassis frame at zero steer)
    % Center the search space around the nominal wheel hub to ensure the 
    % Upright remains a physically small metal chunk.
    geom.M_pivot_bounds.lower = [x_wheel - 0.15; y_wheel - 0.15; -0.05];  % raised 10cm: was Z lb=0.05
    geom.M_pivot_bounds.upper = [x_wheel + 0.15; y_wheel + 0.15;  0.25];  % raised 10cm: was Z ub=0.35

    % Bellcrank pivot in chassis frame
    % PHYSICAL RULES:
    %  (a) Y: Bellcrank must be INBOARD of A-arm chassis fork Y (not beside or outboard)
    %  (b) Z: Bellcrank must be ABOVE the lower A-arm fork (smaller Z = higher in SAE Z-down)
    %         Ideally between the upper and lower A-arm Z heights.
    chassis_fork_y = geom.lower_arm_front_chassis(2); % Signed Y of lower A-arm fork
    lower_arm_z    = geom.lower_arm_front_chassis(3); % Z of lower A-arm fork
    upper_arm_z    = geom.upper_arm_front_chassis(3); % Z of upper A-arm fork
    if is_left
        % Left: y < 0, inboard means Y > fork_y (less negative)
        geom.bellcrank_bounds.lower = [x_wheel - 0.80; chassis_fork_y; upper_arm_z - 0.30];
        geom.bellcrank_bounds.upper = [x_wheel + 0.80;  0.00; lower_arm_z - 0.02];
    else
        % Right: y > 0, inboard means Y < fork_y (less positive)
        geom.bellcrank_bounds.lower = [x_wheel - 0.80;  0.00; upper_arm_z - 0.30];
        geom.bellcrank_bounds.upper = [x_wheel + 0.80; chassis_fork_y; lower_arm_z - 0.02];
    end

    % ========================================
    % NOMINAL VALUES (Initial Guess)
    % ========================================
    % Start the search near the wheel hub center (shifted slightly inboard & upward)
    geom.M_pivot_initial = [x_wheel; y_wheel - sign(y_wheel)*0.08; 0.15];

    if is_left
        bellcrank_y_initial = -0.35;
    else
        bellcrank_y_initial = 0.35;
    end
    geom.bellcrank_initial = [x_wheel; bellcrank_y_initial; -0.10];

    % ========================================
    % UPRIGHT LOCAL FRAME DEFINITION
    % ========================================
    % Store nominal upright points for Procrustes alignment in the solver.
    % These are the 3 upright-mounted points at zero steer, expressed in
    % chassis frame. The solver will use them to build the local frame.
    geom.upright_points_nominal = [
        geom.upper_arm_upright, ...
        geom.lower_arm_upright, ...
        geom.tie_rod_upright
    ];  % 3x3 matrix, each column is a point
end

function validate_corner_quality(corner_meta, wheel_index, allow_missing_tie_rod)
% Fail fast when downstream-required K&C quality metadata says the corner is not usable.
if allow_missing_tie_rod
    return;
end

if isfield(corner_meta, 'quality') && isfield(corner_meta.quality, 'verdict') ...
        && strcmpi(corner_meta.quality.verdict, 'invalid')
    error('POU:InvalidHardpointQuality', ...
        'Wheel %d hardpoints are marked invalid; rerun the suspension/steering stages.', ...
        wheel_index);
end

if ~isfield(corner_meta, 'evaluation_scope')
    return;
end
scope = corner_meta.evaluation_scope;
if isfield(scope, 'tie_rod') && ~strcmpi(scope.tie_rod, 'evaluated')
    error('POU:TieRodNotEvaluated', ...
        ['Wheel %d hardpoints were generated without an evaluated tie rod ', ...
         '(evaluation_scope.tie_rod=%s). Run the steering stage or set ', ...
         'options.allow_missing_tie_rod=true explicitly.'], ...
        wheel_index, scope.tie_rod);
end
if isfield(scope, 'toe') && ~strcmpi(scope.toe, 'evaluated')
    error('POU:ToeNotEvaluated', ...
        ['Wheel %d hardpoints have no evaluated toe curve ', ...
         '(evaluation_scope.toe=%s). Run the steering stage or set ', ...
         'options.allow_missing_tie_rod=true explicitly.'], ...
        wheel_index, scope.toe);
end
end
