function size_class = determineSizeClass(I_mm4, A_mm2, D_mm, t_mm)
% DETERMINESIZECLASS Determine FSAE size class (A/B/C/D/None) for a tube
%
% Syntax:
%   size_class = determineSizeClass(I_mm4, A_mm2, D_mm, t_mm)
%
% Inputs:
%   I_mm4 - Second moment of area [mm^4]
%   A_mm2 - Cross-sectional area [mm^2]
%   D_mm  - Outer diameter [mm]
%   t_mm  - Wall thickness [mm]
%
% Output:
%   size_class - String: 'A', 'B', 'C', 'D', or 'None'
%
% Reference:
%   FSAE 2026 Rules F.3.4.1
%
% Example:
%   size = determineSizeClass(52430, 379, 25.4, 1.65);  % Returns 'A'

    narginchk(4, 4);
    validateattributes(I_mm4, {'numeric'}, {'scalar', 'nonnegative', 'finite'});
    validateattributes(A_mm2, {'numeric'}, {'scalar', 'nonnegative', 'finite'});
    validateattributes(D_mm, {'numeric'}, {'scalar', 'nonnegative', 'finite'});
    validateattributes(t_mm, {'numeric'}, {'scalar', 'nonnegative', 'finite'});

    % FSAE 2026 F.3.4.1 thresholds (confirmed from rulebook, 10 Sept 2025)
    % A tube must have ALL FOUR minimum requirements for each Size:
    % Size A: I≥11320 mm4, A≥173 mm2, D≥25.0 mm, t≥2.0 mm
    % Size B: I≥8509  mm4, A≥114 mm2, D≥25.0 mm, t≥1.2 mm
    % Size C: I≥6695  mm4, A≥91  mm2, D≥25.0 mm, t≥1.2 mm
    % Size D: I≥18015 mm4, A≥126 mm2, D≥35.0 mm, t≥1.2 mm

    % Check Size D first. Size D is a separate bent-upper-side-impact
    % specification, not a strength rank. A tube that satisfies both A and D
    % is classified D so that side-impact selection cannot silently use an
    % A-labelled tube outside the D envelope.
    if I_mm4 >= 18015 && A_mm2 >= 126 && D_mm >= 35.0 && t_mm >= 1.2
        size_class = 'D';
    elseif I_mm4 >= 11320 && A_mm2 >= 173 && D_mm >= 25.0 && t_mm >= 2.0
        size_class = 'A';
    elseif I_mm4 >= 8509 && A_mm2 >= 114 && D_mm >= 25.0 && t_mm >= 1.2
        size_class = 'B';
    elseif I_mm4 >= 6695 && A_mm2 >= 91 && D_mm >= 25.0 && t_mm >= 1.2
        size_class = 'C';
    else
        size_class = 'None';
    end
end
