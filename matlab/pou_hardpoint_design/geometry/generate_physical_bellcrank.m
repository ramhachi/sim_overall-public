function phys = generate_physical_bellcrank(x_pushrod_chassis, pushrod_vec_nominal, geom)
% GENERATE_PHYSICAL_BELLCRANK - Maps mathematical pushrod vector to a 
% physical F1/FSAE triangular bellcrank linkage mounted to the frame.
%
% INPUTS:
%   x_pushrod_chassis   - 3x1 coordinate of where the pushrod ends [m]
%                         (previously called bellcrank_pivot mathematically)
%   pushrod_vec_nominal - 3x1 vector pointing from M-pivot UP to x_pushrod
%   geom                - define_suspension_geometry struct
%
% OUTPUT:
%   phys - Struct containing the 3 physical nodes of the Bellcrank assembly:
%       .pushrod_pin - Where the pushrod attaches to the bellcrank (x_pushrod)
%       .frame_pivot - Where the bellcrank rotates on the chassis frame
%       .damper_pin  - Where the shock absorber attaches to the bellcrank

    % 1. The output of the mathematical optimizer is the exact point in 
    % space where the pushrod must terminate to provide the correct jack-up gradient.
    phys.pushrod_pin = x_pushrod_chassis;
    
    % Pushrod line of action (normalized)
    pushrod_dir = pushrod_vec_nominal / norm(pushrod_vec_nominal);
    
    % 2. Define the Bellcrank Frame Pivot (Rotation Axis)
    % A physical bellcrank needs to be anchored to the chassis frame.
    % To efficiently transfer force, the moment arm from the pivot to the 
    % pushrod line of action should be approximately 90 degrees.
    % We will place the frame pivot inboard and slightly forward/aft of the pushrod pin.
    
    % Desired moment arm length (e.g. 50mm)
    moment_arm_length = 0.050; 
    
    % Define a rotation plane normal. For a typical FSAE car, the bellcrank 
    % rotates roughly in a vertical plane or slightly canted.
    % Let's create a vector orthogonal to the pushrod direction.
    % We cross pushrod_dir with the global Y axis to get a vector pointing 
    % roughly longitudinally/vertically.
    if abs(pushrod_dir(2)) > 0.99
        % If pushrod is almost purely lateral, cross with Z
        orthogonal_vec = cross(pushrod_dir, [0; 0; 1]);
    else
        orthogonal_vec = cross(pushrod_dir, [0; sign(pushrod_dir(2)); 0]);
    end
    orthogonal_vec = orthogonal_vec / norm(orthogonal_vec);
    
    % Place the frame pivot orthogonal to the pushrod line of action.
    % We want the pivot to be further inboard (closer to chassis center).
    % Ensure orthogonal_vec points inboard.
    if sign(orthogonal_vec(2)) == sign(phys.pushrod_pin(2))
        orthogonal_vec = -orthogonal_vec; % Flip if it points outboard
    end
    
    phys.frame_pivot = phys.pushrod_pin + moment_arm_length * orthogonal_vec;
    
    % 3. Define the Damper Pin
    % Assume a Motion Ratio (Pushrod displacement / Damper displacement) of ~1.0.
    % This means the damper pin should be roughly the same distance from 
    % the frame pivot as the pushrod pin (moment arm = 50mm).
    % The angle between the two arms of the bellcrank is typically 60-120 degrees.
    bellcrank_angle = deg2rad(90); % 90 degree L-shape rocker
    
    % Axis of rotation of the bellcrank (normal to the plane formed by Pushrod and Pivot)
    rot_axis = cross(pushrod_dir, orthogonal_vec);
    rot_axis = rot_axis / norm(rot_axis);
    
    % Rotate the orthogonal vector by the bellcrank angle to find the damper pin direction
    R_bellcrank = axang2rotm([rot_axis', bellcrank_angle]);
    damper_arm_vec = R_bellcrank * (-orthogonal_vec); % Vector from pivot to damper pin
    
    phys.damper_pin = phys.frame_pivot + moment_arm_length * damper_arm_vec;
    
    % 4. Define the Chassis Damper Mount (for visualization only, approx 200mm damper)
    damper_length = 0.200;
    % Damper points inboard AND upward (negative Z = up in SAE convention) toward
    % the upper frame rail directly above the bellcrank.
    damper_dir = [0; -sign(phys.pushrod_pin(2)) * 0.5; -0.866];
    damper_dir = damper_dir / norm(damper_dir);
    
    phys.damper_chassis_mount = phys.damper_pin + damper_length * damper_dir;
end
