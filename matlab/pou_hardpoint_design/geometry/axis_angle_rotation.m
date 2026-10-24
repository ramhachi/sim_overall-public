function R = axis_angle_rotation(axis, angle)
% AXIS_ANGLE_ROTATION - Compute 3D rotation matrix from axis-angle representation
%
% INPUTS:
%   axis  - 3×1 rotation axis (unit vector)
%   angle - Rotation angle [rad] (right-hand rule)
%
% OUTPUT:
%   R - 3×3 rotation matrix
%
% ALGORITHM: Rodrigues' rotation formula
%   R = I + sin(θ)·K + (1-cos(θ))·K²
%   where K is the skew-symmetric matrix of the axis

    % Ensure axis is unit vector
    axis = axis(:) / norm(axis);

    % Skew-symmetric matrix (cross-product matrix)
    K = [    0,      -axis(3),  axis(2);
         axis(3),        0,     -axis(1);
        -axis(2),  axis(1),         0   ];

    % Rodrigues formula
    I = eye(3);
    R = I + sin(angle) * K + (1 - cos(angle)) * (K * K);
end
