function cost = objective_polynomial_fit(design_vars, geom, target_poly, delta_range)
% OBJECTIVE_POLYNOMIAL_FIT - Cost function for fmincon optimization
%
% INPUTS:
%   design_vars - [Xm, Ym, Zm, Xb, Yb, Zb] - 6×1 design variables [m]
%   geom        - Suspension geometry struct
%   target_poly - Struct with polynomial coefficients:
%                 .jack_c1, .jack_c2     - Jack-up coefficients [m/rad, m/rad²]
%                 .camber_c1, .camber_c2 - Camber coefficients [rad/rad, rad/rad²]
%   delta_range - N×1 steering angle array [rad]
%
% OUTPUT:
%   cost - Scalar weighted RMS error (lower is better)
%
% COST FUNCTION:
%   cost = w_jack * rms(jack_error) + w_camber * rms(camber_error)
%   where error is in [mm] for jack, [deg] for camber

    % Weight factors 
    % Structure Note: In a Double Wishbone suspension, POU (M-pivot & bellcrank)
    % ONLY affects Pushrod Jack-up. Camber is fundamentally 100% defined by the Kingpin Axis 
    % (A-arms). Therefore, the POU optimizer mathematically cannot change the Camber curve.
    w_jack = 1.0;    % Jack-up error weight [mm]
    w_camber = 0.0;  % Camber error MUST BE 0.0 here (A-arms are fixed!)

    % Compute geometric curves from hardpoint coordinates
    [jack_curve, camber_curve, ~] = forward_kinematics(design_vars, geom, delta_range);

    % Compute target polynomial curves
    jack_target = target_poly.jack_c1 * delta_range + ...
                  target_poly.jack_c2 * delta_range.^2;
    camber_target = target_poly.camber_c1 * delta_range + ...
                    target_poly.camber_c2 * delta_range.^2;

    % Compute errors
    jack_error = jack_curve - jack_target;      % [m]
    camber_error = camber_curve - camber_target;  % [rad]

    % Convert to convenient units for weighting
    jack_error_mm = jack_error * 1000;          % [mm]
    camber_error_deg = rad2deg(camber_error);   % [deg]

    % RMS and Max errors
    rms_jack = sqrt(mean(jack_error_mm.^2));
    max_jack = max(abs(jack_error_mm));
    rms_camber = sqrt(mean(camber_error_deg.^2));

    % Weighted cost (70% RMS, 30% Max Error to prevent edge drooping)
    cost = w_jack * (0.7 * rms_jack + 0.3 * max_jack) + w_camber * rms_camber;

    % Debug output (optional - comment out for production)
    % fprintf('  Jack RMS: %.2f mm, Camber RMS: %.2f deg, Cost: %.2f\n', ...
    %         rms_jack, rms_camber, cost);
end
