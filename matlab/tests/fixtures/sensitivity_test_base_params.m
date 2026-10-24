function base_params = sensitivity_test_base_params()
base_params = create_vehicle_baseline_template();
base_params.mass.m_total = 262.0;
base_params.mass.m_sprung = 210.0;
base_params.mass.b = 0.83;
base_params.mass.a = base_params.mass.wheelbase - base_params.mass.b;
base_params.suspension.K_chassis = 1000.0;
base_params.suspension.C_chassis = 6.5;
base_params.suspension.I_twist = 1.8;
base_params.steering.stiffness = 5400.0;
base_params.steering.ackermann = 0.62;
base_params.brake.balance = 0.58;
base_params.powertrain.max_torque = 63.0;
base_params.powertrain.engine_torque_curve = base_params.powertrain.engine_torque_curve ...
    * (base_params.powertrain.max_torque / max(base_params.powertrain.engine_torque_curve));
end
