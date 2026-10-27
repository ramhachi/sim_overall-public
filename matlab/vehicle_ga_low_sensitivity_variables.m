function names = vehicle_ga_low_sensitivity_variables()
% VEHICLE_GA_LOW_SENSITIVITY_VARIABLES Variables that barely move the lap near the optimum.
%
% OAT around the 15k-evaluation GA best (65.04 s, ga15k_hp_20261027_002804),
% +-10 % and +-25 % of each range: these changed the lap by <= 0.29 s, i.e.
% within the neighbourhood noise (+-1 % jitter mean +0.48 s). Freeze them via
% config.freeze_variables to search ~28 instead of 39 variables; re-check after
% the design moves far (ARB matters again once the springs get soft).
names = {'BumpStopGap', 'BrakeBalance', 'ChassisDamping', 'DamperF', 'ARBR', ...
    'ChassisTwistInertia', 'ARBF', 'BumpStopK', 'MotionRatioR', ...
    'SteeringAckermann', 'LSDRampDec'};
end
