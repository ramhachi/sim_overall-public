function x0 = generate_initial_guess(geom, target_poly, strategy)
% GENERATE_INITIAL_GUESS - Generate smart initial guess based on target polynomial
%
% INPUTS:
%   geom        - Suspension geometry struct
%   target_poly - Target polynomial coefficients
%   strategy    - 'nominal', 'conservative', 'aggressive', or 'random'
%
% OUTPUT:
%   x0 - 6×1 initial guess [Xm, Ym, Zm, Xb, Yb, Zb]

    if nargin < 3
        strategy = 'nominal';
    end

    % Get bounds
    lb = [geom.M_pivot_bounds.lower; geom.bellcrank_bounds.lower];
    ub = [geom.M_pivot_bounds.upper; geom.bellcrank_bounds.upper];

    switch strategy
        case 'nominal'
            % Use nominal values from geometry
            x0 = [geom.M_pivot_initial; geom.bellcrank_initial];

        case 'conservative'
            % Conservative: smaller M-pivot offset, centered bellcrank
            % Focus on feasible region with moderate IR (~0.15-0.25)
            M_pivot = [
                0.02;  % Very small X offset
                -0.005; % Very small Y offset
                -0.015  % Shallow Z offset
            ];

            bellcrank = [
                (lb(4) + ub(4)) / 2;  % Centered X
                (lb(5) + ub(5)) / 2;  % Centered Y
                (lb(6) + ub(6)) / 2   % Centered Z (better than -0.10)
            ];

            x0 = [M_pivot; bellcrank];

        case 'aggressive'
            % Aggressive: larger M-pivot offset for more jack-up
            % But still constrained to feasible region
            jack_c1_abs = abs(target_poly.jack_c1);

            % Moderate X offset based on target (bounded)
            Xm = min(ub(1) * 0.8, 0.06 + jack_c1_abs * 0.3);
            Zm = max(lb(3) + 0.02, -0.04);  % Not too low to maintain feasibility

            M_pivot = [
                Xm;
                -0.015; % Moderate Y offset
                Zm
            ];

            bellcrank = [
                (lb(4) + ub(4)) / 2;
                (lb(5) + ub(5)) / 2;
                (lb(6) + ub(6)) / 2 - 0.05  % Slightly lower but centered
            ];

            x0 = [M_pivot; bellcrank];

        case 'random'
            % Random within bounds
            x0 = lb + (ub - lb) .* rand(6, 1);

        otherwise
            error('Unknown strategy: %s', strategy);
    end

    % Ensure within bounds
    x0 = max(lb, min(ub, x0));
end
