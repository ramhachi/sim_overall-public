function [jack_curve, camber_curve, pushrod_lengths] = forward_kinematics(design_vars, geom, delta_range)
% FORWARD_KINEMATICS - Compute jack-up and camber curves from hardpoint coordinates
%
% Uses the double-wishbone solver to get upright rigid-body transform,
% then computes M-pivot position in chassis frame and pushrod length.
% Camber is extracted from the change in kingpin axis tilt.
%
% INPUTS:
%   design_vars - [Xm, Ym, Zm, Xb, Yb, Zb] - 6x1 design variables [m]
%   geom        - Suspension geometry struct from define_suspension_geometry()
%   delta_range - Nx1 steering angle array [rad]
%
% OUTPUTS:
%   jack_curve      - Nx1 pushrod displacement [m] (positive = compression)
%   camber_curve    - Nx1 camber angle [rad]
%   pushrod_lengths - Nx1 absolute pushrod length [m]

    M_pivot_upright = design_vars(1:3);
    bellcrank_pivot = design_vars(4:6);

    N = length(delta_range);
    jack_curve = zeros(N, 1);
    camber_curve = zeros(N, 1);
    pushrod_lengths = zeros(N, 1);

    % Nominal state (zero steer)
    upright_nominal = solve_upright_position_3d(0.0, geom);
    M_pivot_chassis_nominal = upright_nominal.R_upright * M_pivot_upright + upright_nominal.t_upright;
    L_pushrod_nominal = norm(M_pivot_chassis_nominal - bellcrank_pivot);

    % Set up nominal wheel spindle vector (points outwards)
    if geom.is_left
        spindle_nominal = [0; -1; 0];
    else
        spindle_nominal = [0; 1; 0];
    end

    for i = 1:N
        delta = delta_range(i);

        upright_state = solve_upright_position_3d(delta, geom);

        % M-pivot in chassis frame via rigid body transform
        M_pivot_chassis = upright_state.R_upright * M_pivot_upright + upright_state.t_upright;

        % Pushrod length
        L_pushrod = norm(M_pivot_chassis - bellcrank_pivot);
        pushrod_lengths(i) = L_pushrod;

        % Jack-up (positive = compression = shorter pushrod)
        jack_curve(i) = -(L_pushrod - L_pushrod_nominal);

        % Camber from wheel spindle tilt
        % The wheel spindle vector defines the outward axle.
        % Positive camber = top of wheel tilts OUT = spindle tilts DOWN (+Z).
        spindle_now = upright_state.R_upright * spindle_nominal;
        camber_curve(i) = asin(spindle_now(3));
    end
end
