function schema = build_full_vehicle_design_schema()
% BUILD_FULL_VEHICLE_DESIGN_SCHEMA Canonical full-vehicle optimizer contract.
%
% Keep the design-vector names, bounds, and extended tunable indices in one
% place so optimization artifacts can be validated before simulation.

core_bounds = [
    250.0, 350.0;    % 1. Mass Total [kg]
    0.40,  0.60;     % 2. Front Dist [-]
    0.25,  0.40;     % 3. CG Height [m]
    1.53,  1.85;     % 4. Wheelbase [m]
    1.10,  1.45;     % 5. Track Width F [m]
    1.10,  1.45;     % 6. Track Width R [m]
    10000, 90000;    % 7. Spring F [N/m]
    10000, 90000;    % 8. Spring R [N/m]
    100,   3000;     % 9. Damper F [Ns/m]
    100,   3000;     % 10. Damper R [Ns/m]
    0,     5000;     % 11. ARB F [Nm/rad]
    0,     5000;     % 12. ARB R [Nm/rad]
    -5.0,  0.0;      % 13. Camber F [deg]
    -3.0,  0.0;      % 14. Camber R [deg]
    -3.0,  3.0;      % 15. Toe F [deg]
    -1.5,  1.5;      % 16. Toe R [deg]
    0.0,   5.0;      % 17. Aero CLA [m^2]
    0,     200;      % 18. LSD Preload [Nm]
    10,    60;       % 19. LSD Ramp Accel [deg]
    10,    80;       % 20. LSD Ramp Decel [deg]
    3.0,   10.0;     % 21. Final Drive Ratio [-]
    -0.10, 0.10;     % 22. POU Jack C1 F [m/rad]
    -0.10, 0.10;     % 23. POU Jack C2 F [m/rad^2]
    -0.50, 0.50;     % 24. POU Camber C1 F [rad/rad]
    -0.50, 0.50;     % 25. POU Camber C2 F [rad/rad^2]
    1000,  30000;    % 26. Steering Stiffness [Nm/rad]
    0.50,  1.20;     % 27. Motion Ratio F [-]
    0.50,  1.20;     % 28. Motion Ratio R [-]
    -1.00, 0.00;     % 29. Camber Gain F [rad/m]
    -1.00, 0.00;     % 30. Camber Gain R [rad/m]
    500,   4000;     % 31. Chassis Torsional Stiffness [Nm/deg]
    0.30,  0.70;     % 32. Aero Balance Front Fraction [-]
];

core_names = {
    'Mass', 'FrontDist', 'CGHeight', 'Wheelbase', 'TrackF', 'TrackR', ...
    'SpringF', 'SpringR', 'DamperF', 'DamperR', 'ARBF', 'ARBR', ...
    'CamberF', 'CamberR', 'ToeF', 'ToeR', 'AeroCLA', ...
    'LSDPreload', 'LSDRampAcc', 'LSDRampDec', 'FinalDrive', ...
    'POU_Jack_C1_F', 'POU_Jack_C2_F', 'POU_Camber_C1_F', 'POU_Camber_C2_F', ...
    'SteeringStiffness', 'MotionRatioF', 'MotionRatioR', 'CamberGainF', 'CamberGainR', ...
    'ChassisStiffness', 'AeroBalance'
};

extra_specs = build_extended_vehicle_tunable_specs();
extra_specs(strcmp({extra_specs.name}, 'ChassisStiffness')) = [];
extra_bounds = zeros(numel(extra_specs), 2);
extra_names = cell(1, numel(extra_specs));
for i = 1:numel(extra_specs)
    extra_specs(i).index = size(core_bounds, 1) + i;
    extra_bounds(i, :) = [extra_specs(i).lower, extra_specs(i).upper];
    extra_names{i} = extra_specs(i).name;
end

schema = struct( ...
    'version', 'full_vehicle_design/v2', ...
    'core_variable_count', size(core_bounds, 1), ...
    'variable_bounds', [core_bounds; extra_bounds], ...
    'variable_names', { [core_names, extra_names] }, ...
    'extra_tunable_specs', extra_specs);
end
