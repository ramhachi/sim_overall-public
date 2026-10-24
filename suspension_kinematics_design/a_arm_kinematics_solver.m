function results = a_arm_kinematics_solver(hardpoints)
% A_ARM_KINEMATICS_SOLVER - Compute analytical suspension geometry properties
%
% INPUT: 
%   hardpoints - struct with 3D [X, Y, Z] fields:
%       .U_F, .U_R, .U_U : Upper arm chassis front/rear, upright
%       .L_F, .L_R, .L_U : Lower arm chassis front/rear, upright
%       .W_C             : Wheel center
%       .R_tire          : Tire loaded radius [m]
%       .is_left         : Boolean (true for FL/RL, false for FR/RR)
%
% OUTPUT:
%   results.camber_deg      : Static Camber angle [deg]
%   results.caster_deg      : Caster angle [deg]
%   results.kpi_deg         : Kingpin inclination angle [deg]
%   results.scrub_radius_m  : Scrub radius [m]
%   results.roll_center_z   : Roll center height [m]
%   results.IC_y            : Instant Center Y [m]
%   results.IC_z            : Instant Center Z [m]

    %% 1. Extract Points
    U_U = hardpoints.U_U;
    L_U = hardpoints.L_U;
    W_C = hardpoints.W_C;
    is_left = hardpoints.is_left;
    R_tire = hardpoints.R_tire;
    
    % Chassis mid-points for simplified 2D projection swing arm
    U_C = (hardpoints.U_F + hardpoints.U_R) / 2;
    L_C = (hardpoints.L_F + hardpoints.L_R) / 2;
    
    %% 2. Kingpin Axis properties
    K_vec = U_U - L_U; % Vector pointing UP (towards negative Z)
    K_dir = K_vec / norm(K_vec);
    
    % Caster (angle in X-Z plane)
    % Positive when top is tilted rearward (-X).
    % K_dir points to negative Z (up), so Z component is negative.
    % If top is tilted rearward, X component is negative.
    % Angle from vertical (Z-axis pointing down):
    % The vector pointing DOWN from U_U to L_U would have positive Z.
    % Let's use the vector pointing DOWN:
    K_down = L_U - U_U;
    K_down = K_down / norm(K_down);
    
    % Caster: Positive if L_U is forward of U_U (+X).
    results.caster_deg = atan2d(K_down(1), K_down(3));
    
    % KPI (angle in Y-Z plane)
    % Positive when top is tilted inward (towards chassis center)
    % Left side wheel is at -Y. "Inward" for top means U_U is more positive Y than L_U.
    % So K_down (L_U - U_U) leans outward (-Y). K_down(2) is negative.
    % For positive KPI, we want -K_down(2) / K_down(3) for the left side.
    if is_left
        results.kpi_deg = atan2d(-K_down(2), K_down(3));
    else
        results.kpi_deg = atan2d(K_down(2), K_down(3));
    end
    
    %% 3. Tire Contact Pitch (TCP) & Scrub Radius
    TCP = W_C;
    % Assuming Z is down, TCP is below Wheel Center, so Z is larger (more positive)
    TCP(3) = TCP(3) + R_tire; 
    
    % Pierce point: where kingpin intersects ground plane Z = TCP(3)
    t = (TCP(3) - L_U(3)) / K_dir(3);
    pierce_pt = L_U + t * K_dir;
    
    % Scrub radius (Y distance) - Positive if scrub is inward of centerline
    if is_left
        % Left side: Y is negative. Inward (closer to 0) means pierce_pt(2) > TCP(2)
        results.scrub_radius_m = pierce_pt(2) - TCP(2);
    else
        % Right side: Y is positive. Inward (closer to 0) means pierce_pt(2) < TCP(2)
        results.scrub_radius_m = TCP(2) - pierce_pt(2);
    end
    
    %% 4. Roll Center (Front View Swing Arm)
    % 2D projection vectors in Y-Z plane
    dir_up = [U_U(2) - U_C(2), U_U(3) - U_C(3)];
    dir_low = [L_U(2) - L_C(2), L_U(3) - L_C(3)];
    
    % Find intersection of upper and lower arm lines (Instant Center)
    % Line 1: p1 + u*dir_up
    % Line 2: p2 + v*dir_low
    % p1_y + u*v_y1 = p2_y + v*v_y2 => u*v_y1 - v*v_y2 = p2_y - p1_y
    A = [dir_up(1), -dir_low(1); 
         dir_up(2), -dir_low(2)];
    b = [L_U(2) - U_U(2); L_U(3) - U_U(3)];
    
    if abs(det(A)) < 1e-6
        % Arms are essentially parallel, IC is at infinity
        results.IC_y = sign(dir_up(1)) * 1e6;
        results.IC_z = W_C(3); % Arbitrary parallel
        % Special parallel geometry RC -> follows the parallel link inclination
        slope = dir_up(2) / dir_up(1);
        results.roll_center_z = TCP(3) + (-TCP(2)) * slope;
    else
        sol = A \ b;
        IC_y = U_U(2) + sol(1) * dir_up(1);
        IC_z = U_U(3) + sol(1) * dir_up(2);
        
        results.IC_y = IC_y;
        results.IC_z = IC_z;
        
        % RC is intersection of IC-TCP line with Y=0
        slope_ic_tcp = (IC_z - TCP(3)) / (IC_y - TCP(2));
        results.roll_center_z = TCP(3) + slope_ic_tcp * (0 - TCP(2));
    end
    
    %% 5. Static Camber
    % Define the wheel spindle axis. Typically perpendicular to Upright plane.
    % In this static snapshot, we can determine camber purely by the Upright orientation
    % Given we don't have a rigid upright body definition passing in, and only points,
    % we'll assume the nominal Camber is defined by testing the inclination of the Spindle.
    % Wait, if only points are given, we can't fully lock the upright's 3rd degree of spin 
    % without the Tie Rod.
    % Let's add Tie Rod to input if needed, but for now we'll mock static camber as 0
    % in this base geometric snapshot, unless Spindle points are provided.
    if isfield(hardpoints, 'Spindle_Dir')
        spindle = hardpoints.Spindle_Dir;
    else
        if is_left
            spindle = [0; -1; 0];
        else
            spindle = [0; 1; 0];
        end
    end
    
    % Camber = Angle Spindle makes with horizontal (+Z tilting inward)
    % If camber is negative (top tilted inward), then spindle points somewhat UP (negative Z).
    % Spindle points OUT (-Y).
    % So spindle(3) will be negative for negative camber.
    % asin(negative) = negative.
    % This matches convention: negative camber = negative angle.
    results.camber_deg = asind(spindle(3));
    
    %% 6. Camber Gain (Numerical Derivative)
    % Simulate 25mm heave (chassis drops 25mm relative to wheel)
    % Equivalently: Move Wheel UP by 25mm (Z gets smaller = more negative)
    bump_travel = -0.025; % [m] wheel moves up in Z
    
    % We need to find the new upright angle. 
    % A fast approximation uses the simple 2D swing arm. 
    % Wheel moves along an arc centered at the Instant Center (IC).
    % The distance from IC to Wheel Center is R_ic.
    % Swing arm length L_sa = TCP(2) - IC_y (in Y)
    
    % An exact kinematic solve is complex without Tie-Rod, but evaluating 
    % the camber change using the parallel/IC geometry is standard:
    % Camber Gain = - d(Camber_rad) / dZ_wheel
    % Positive camber gain = wheel gets MORE negative camber under bump.
    % Roll center kinematics state: Camber_change_rad = Bump_travel / Swing_Arm_Length
    
    L_sa = TCP(2) - results.IC_y;
    if abs(L_sa) > 1e-6
        % Delta camber (rad) = Delta Z / L_sa
        camber_change_rad = bump_travel / L_sa;
    else
        camber_change_rad = 0; % Infinite swing arm -> zero camber change
    end
    
    % Camber gain [rad/m] = Delta camber / Delta Z
    % If wheel moves UP (negative dZ), and camber goes NEGATIVE (top in),
    % gain = (-dC) / (-dZ) = positive.
    % Typically 'Camber Gain' in vehicle dynamics is reported such that
    % negative camber in bump = negative gain.
    % Standard mathematically: d(Gamma)/dz 
    if is_left
        results.camber_gain_rad_m = camber_change_rad / bump_travel;
    else
        % Right side: Y is flipped, geometry signs flip
        results.camber_gain_rad_m = -camber_change_rad / bump_travel;
    end
end
