function s = kc_dense_samples(params, ds)
% KC_DENSE_SAMPLES Export-grade sweep knots (#40): travel over the full
% bump-stop range plus margin at ds.travel_step_m, roll at ds.roll_step_rad,
% steer at ds.steer_step_rad up to params.steering.max_angle. The rack sweep
% uses the same knot count so its steer spacing is ~1 deg.
half = params.suspension.bump_stop_gap + ds.travel_margin_m;
s = struct();
s.TravelSamples = linspace(-half, half, 2 * ceil(half / ds.travel_step_m) + 1);
s.RollSamples = linspace(-ds.roll_range_rad, ds.roll_range_rad, ...
    2 * ceil(ds.roll_range_rad / ds.roll_step_rad) + 1);
nSteer = 2 * ceil(params.steering.max_angle / ds.steer_step_rad) + 1;
s.SteerSamples = linspace(-params.steering.max_angle, params.steering.max_angle, nSteer);
s.RackSamples = linspace(-ds.rack_travel_max_m, ds.rack_travel_max_m, nSteer);
end
