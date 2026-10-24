function obs_vehicle = get_rl_vehicle_observation_vector(params)
% GET_RL_VEHICLE_OBSERVATION_VECTOR Return the vehicle-spec obs terms used by RL.

obs_vehicle = [
    params.mass.m_total
    params.aero.CL_A
    params.aero.CD_A
    params.tire.mu_y_max
    params.tire.mu_x_max
    params.suspension.k_spring(1)
    params.suspension.k_arb_f
    params.steering.stiffness
    params.suspension.K_chassis
    params.suspension.C_chassis
    params.suspension.I_twist
];
end
