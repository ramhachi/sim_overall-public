function phys = generate_physical_bellcrank(x_pushrod_chassis, pushrod_vec_nominal, geom)
% GENERATE_PHYSICAL_BELLCRANK Backward-compatible wrapper.
%
% The rocker is now derived once by derive_bellcrank_geometry and used by the
% closed-chain solver (#44). This wrapper keeps older visualization scripts
% working without a second, inconsistent set of hardpoints.

    bc = derive_bellcrank_geometry(x_pushrod_chassis, pushrod_vec_nominal, geom);
    phys = bc;
    phys.pushrod_pin = bc.pushrod_pin;
    phys.frame_pivot = bc.frame_pivot;
    phys.damper_pin = bc.damper_pin;
    phys.damper_chassis_mount = bc.damper_chassis_mount;
end
