function axle = build_axle_kc(metrics, motionRatio, isFront)
% BUILD_AXLE_KC Runtime kc.front|rear block. Runtime (14DOF) convention:
% roll_center_height is height above ground, derivatives are with respect to
% compression-positive suspension travel. anti_brake/anti_drive are per-axle
% force fractions (#37); the classical dive/rise/squat/lift are bias-weighted.
axle = struct();
axle.roll_center_height = -metrics.roll_center_z;
axle.roll_center_lateral = 0.0;
axle.roll_center_height_per_travel = -metrics.roll_center_height_per_travel;
axle.roll_center_height_per_travel_sq = -metrics.roll_center_height_per_travel_sq;  % [1/m]
axle.roll_center_height_per_roll = -metrics.roll_center_height_per_roll;
axle.roll_center_height_per_roll_sq = -metrics.roll_center_height_per_roll_sq;  % [m/rad^2]
axle.anti_brake_pct = metrics.anti_brake_pct;
axle.anti_drive_pct = metrics.anti_drive_pct;
axle.anti_lift_pct = metrics.anti_lift_pct;
axle.anti_rise_pct = metrics.anti_rise_pct;
axle.anti_dive_pct = metrics.anti_dive_pct;
axle.anti_squat_pct = metrics.anti_squat_pct;
axle.anti_brake_vs_travel = metrics.anti_brake_vs_travel;
axle.anti_drive_vs_travel = metrics.anti_drive_vs_travel;
axle.anti_model = metrics.anti_model;
axle.anti_validated = metrics.anti_validated;
axle.arb_motion_ratio = mean(motionRatio(:));
if isFront
    axle.anti_rise_pct = 0.0;
    axle.anti_squat_pct = 0.0;
else
    axle.anti_lift_pct = 0.0;
    axle.anti_dive_pct = 0.0;
end
end
