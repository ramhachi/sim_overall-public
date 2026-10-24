function props = computeTubeSectionProperties(D_outer, t, shape)
% COMPUTETUBESECTIONPROPERTIES Calculate section properties for tubes
%
% Syntax:
%   props = computeTubeSectionProperties(D_outer, t, shape)
%
% Inputs:
%   D_outer - Outer diameter (round) or side length (square) [mm]
%   t       - Wall thickness [mm]
%   shape   - 'round' or 'square'
%
% Output:
%   props - Struct with fields:
%           A_mm2   - Cross-sectional area [mm^2]
%           Iy_mm4  - Second moment of area about y-axis [mm^4]
%           Iz_mm4  - Second moment of area about z-axis [mm^4]
%           J_mm4   - Polar moment of area [mm^4]
%           Asy_mm2 - Shear area in y-direction [mm^2] (optional)
%           Asz_mm2 - Shear area in z-direction [mm^2] (optional)
%
% Example:
%   props = computeTubeSectionProperties(25.4, 1.65, 'round');
%   fprintf('Area = %.2f mm2, I = %.1f mm4\n', props.A_mm2, props.Iy_mm4);

    narginchk(3, 3);
    validateattributes(D_outer, {'numeric'}, {'scalar', 'positive', 'finite'});
    validateattributes(t, {'numeric'}, {'scalar', 'positive', 'finite'});
    validateattributes(shape, {'char', 'string'}, {'scalartext'});

    % Validate thickness
    if t >= D_outer / 2
        error('computeTubeSectionProperties:InvalidThickness', ...
              'Thickness t=%.3f must be less than D/2=%.3f', t, D_outer/2);
    end

    D_inner = D_outer - 2*t;

    switch lower(shape)
        case 'round'
            % Circular tube
            % A = π/4 * (D_outer^2 - D_inner^2)
            A_mm2 = pi/4 * (D_outer^2 - D_inner^2);

            % I = π/64 * (D_outer^4 - D_inner^4)
            Iy_mm4 = pi/64 * (D_outer^4 - D_inner^4);
            Iz_mm4 = Iy_mm4;  % Symmetric

            % J = π/32 * (D_outer^4 - D_inner^4) = 2*I for circular section
            J_mm4 = pi/32 * (D_outer^4 - D_inner^4);

            % Shear area (Timoshenko beam theory, κ ≈ 0.5 for thin tubes)
            % As ≈ A / 2 for circular tubes
            Asy_mm2 = A_mm2 / 2;
            Asz_mm2 = Asy_mm2;

        case 'square'
            % Square tube (side length = D_outer)
            % A = D_outer^2 - D_inner^2
            A_mm2 = D_outer^2 - D_inner^2;

            % I = (D_outer^4 - D_inner^4) / 12
            Iy_mm4 = (D_outer^4 - D_inner^4) / 12;
            Iz_mm4 = Iy_mm4;  % Symmetric

            % J for thin-walled square tube (torsion constant)
            % J ≈ 2*t*(D_outer-t)^2 * (D_outer-t) / (D_outer-0.63*t)
            % Simplified: J ≈ 2*t*b^3 where b = D_outer - t
            b = D_outer - t;
            J_mm4 = 2 * t * b^3 / (1 + 0.63 * t / b);

            % Shear area (κ ≈ 0.83 for square sections)
            Asy_mm2 = 0.83 * A_mm2;
            Asz_mm2 = Asy_mm2;

        otherwise
            error('computeTubeSectionProperties:InvalidShape', ...
                  'Shape must be "round" or "square", got: %s', shape);
    end

    % Pack into output struct
    props = struct();
    props.A_mm2 = A_mm2;
    props.Iy_mm4 = Iy_mm4;
    props.Iz_mm4 = Iz_mm4;
    props.J_mm4 = J_mm4;
    props.Asy_mm2 = Asy_mm2;
    props.Asz_mm2 = Asz_mm2;
end
