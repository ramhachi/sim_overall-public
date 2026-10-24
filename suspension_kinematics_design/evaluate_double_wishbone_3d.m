function metrics = evaluate_double_wishbone_3d(geom, varargin)
% EVALUATE_DOUBLE_WISHBONE_3D Evaluate double-wishbone geometry with 3D travel / roll / steer sweeps.

p = inputParser;
addParameter(p, 'TrackWidth', 2 * abs(geom.W_C(2)), @isnumeric);
addParameter(p, 'Wheelbase', 1.6, @isnumeric);
addParameter(p, 'CGHeight', 0.28, @isnumeric);
addParameter(p, 'CGToAxle', 0.8, @isnumeric);
addParameter(p, 'BrakeBiasFront', 0.65, @isnumeric);
addParameter(p, 'DriveBiasRear', 1.0, @isnumeric);
addParameter(p, 'StaticToe', 0.0, @isnumeric);
addParameter(p, 'StaticCamber', 0.0, @isnumeric);
addParameter(p, 'SteeringStiffness', 5000.0, @isnumeric);
addParameter(p, 'TravelSamples', linspace(-0.03, 0.03, 7), @isnumeric);
addParameter(p, 'RollSamples', deg2rad(linspace(-4.0, 4.0, 5)), @isnumeric);
addParameter(p, 'SteerSamples', deg2rad(linspace(-10.0, 10.0, 5)), @isnumeric);
% RackSamples [m] drives the closed-link steering sweep when a tie rod is present.
addParameter(p, 'RackSamples', linspace(-0.05, 0.05, 5), @isnumeric);
addParameter(p, 'UseTieRod', isfield(geom, 'tie_rod_chassis') && isfield(geom, 'tie_rod_upright'), @islogical);
addParameter(p, 'ResidualTolerance', 5e-4, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'BranchToeTolerance', 0.05, @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'BranchCamberTolerance', deg2rad(2.0), @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'BranchKingpinTolerance', deg2rad(2.0), @(x) isnumeric(x) && isscalar(x) && x > 0);
addParameter(p, 'ComplianceModel', struct(), @isstruct);
parse(p, varargin{:});
opts = p.Results;
eval_start = tic;

nominalPose = build_nominal_pose(geom);
staticMetrics = compute_static_metrics(geom, nominalPose, opts.StaticToe);
staticCamberOffsetRad = opts.StaticCamber;
staticMetrics.camber_deg = staticMetrics.camber_deg + rad2deg(staticCamberOffsetRad);
nominalCamberRad = compute_camber_from_pose(geom, nominalPose) + staticCamberOffsetRad;

travelSamples = reshape(opts.TravelSamples, 1, []);
% Runtime convention (14DOF): positive travel = compression. The sweep input
% is a Z-down wheel-center offset, so compression is the negated sample.
travelFromEq = -travelSamples;
camberTravel = zeros(size(travelSamples));
toeTravel = zeros(size(travelSamples));
travelKingpin = zeros(3, numel(travelSamples));
travelFlags = false(size(travelSamples));
travelResiduals = zeros(size(travelSamples));
travelIterations = zeros(size(travelSamples));
travelExitflags = zeros(size(travelSamples));
travelGuess = [];
travelWheelCenter = zeros(3, numel(travelSamples));
travelContactPatch = zeros(3, numel(travelSamples));
travelSampleValues = repmat(struct('z_offset_m', 0, 'camber_rad', 0, 'toe_rad', 0, ...
    'residual_norm', 0, 'iterations', 0, 'exitflag', 0, 'converged', false), 1, numel(travelSamples));
for i = 1:numel(travelSamples)
    pose = solve_pose_for_wheel_center_z(geom, geom.W_C(3) + travelSamples(i), ...
        opts.UseTieRod, travelGuess, opts.ResidualTolerance);
    if pose.converged
        travelGuess = pose.solver_vars;
    end
    travelFlags(i) = pose.converged;
    travelResiduals(i) = pose.residual_norm;
    travelIterations(i) = pose.iterations;
    travelExitflags(i) = pose.exitflag;
    camberTravel(i) = compute_camber_from_pose(geom, pose) + staticCamberOffsetRad;
    toeTravel(i) = compute_toe_from_pose(geom, pose) + opts.StaticToe;
    travelKingpin(:, i) = pose.U_U - pose.L_U;
    travelWheelCenter(:, i) = pose.W_C;
    travelContactPatch(:, i) = compute_contact_patch(geom, pose);
    travelSampleValues(i).z_offset_m = travelSamples(i);
    travelSampleValues(i).camber_rad = camberTravel(i);
    travelSampleValues(i).toe_rad = toeTravel(i);
    travelSampleValues(i).residual_norm = pose.residual_norm;
    travelSampleValues(i).iterations = pose.iterations;
    travelSampleValues(i).exitflag = pose.exitflag;
    travelSampleValues(i).converged = pose.converged;
end
travelConverged = all(travelFlags);
travelBranchContinuous = branch_is_continuous(camberTravel, toeTravel, travelKingpin, ...
    opts.BranchCamberTolerance, opts.BranchToeTolerance, opts.BranchKingpinTolerance);
travelStatus = sample_status(travelFlags, travelResiduals, travelIterations, travelExitflags);

steerSamples = reshape(opts.SteerSamples, 1, []);
rackSamples = reshape(opts.RackSamples, 1, []);
useClosedLink = opts.UseTieRod && isfield(geom, 'tie_rod_chassis') ...
    && isfield(geom, 'tie_rod_upright') && ~isempty(rackSamples);
if useClosedLink
    nSteerSamples = numel(rackSamples);
else
    nSteerSamples = numel(steerSamples);
end
camberSteer = zeros(1, nSteerSamples);
toeSteer = zeros(1, nSteerSamples);
steerActualAngles = zeros(1, nSteerSamples);
steerRack = zeros(1, nSteerSamples);
steerKingpin = zeros(3, nSteerSamples);
steerFlags = false(1, nSteerSamples);
steerResiduals = zeros(1, nSteerSamples);
steerIterations = zeros(1, nSteerSamples);
steerExitflags = zeros(1, nSteerSamples);
steerGuess = [];
steerSampleStatus = cell(1, nSteerSamples);
steerSampleValues = repmat(struct('rack_displacement_m', 0, 'steer_angle_rad', 0, ...
    'camber_rad', 0, 'toe_rad', 0, 'residual_norm', 0, ...
    'iterations', 0, 'exitflag', 0, 'converged', false), 1, nSteerSamples);
for i = 1:nSteerSamples
    if useClosedLink
        pose = solve_pose_for_rack(geom, rackSamples(i), geom.W_C(3), ...
            steerGuess, opts.ResidualTolerance);
        if pose.converged
            steerGuess = pose.solver_vars;
        end
        steerRack(i) = rackSamples(i);
        steerActualAngles(i) = pose.steer_angle;
        steerFlags(i) = pose.converged;
        steerSampleStatus{i} = struct( ...
            'mode', 'closed_link_rack', ...
            'rack_displacement', rackSamples(i), ...
            'steer_angle', pose.steer_angle, ...
            'converged', pose.converged, ...
            'exitflag', pose.exitflag, ...
            'residual_norm', pose.residual_norm, ...
            'iterations', pose.iterations);
    else
        pose = solve_pose_for_steer_rigid(geom, steerSamples(i));
        steerActualAngles(i) = steerSamples(i);
        steerFlags(i) = pose.converged;
        steerSampleStatus{i} = struct( ...
            'mode', 'rigid_proxy', ...
            'target_steer_angle', steerSamples(i), ...
            'steer_angle', steerSamples(i), ...
            'rack_displacement', 0.0, ...
            'converged', pose.converged, ...
            'exitflag', pose.exitflag, ...
            'residual_norm', pose.residual_norm, ...
            'iterations', pose.iterations);
    end
    steerResiduals(i) = pose.residual_norm;
    steerIterations(i) = pose.iterations;
    steerExitflags(i) = pose.exitflag;
    camberSteer(i) = compute_camber_from_pose(geom, pose) + staticCamberOffsetRad - nominalCamberRad;
    toeSteer(i) = compute_toe_from_pose(geom, pose) - steerActualAngles(i);
    steerKingpin(:, i) = pose.U_U - pose.L_U;
    steerSampleValues(i).rack_displacement_m = steerRack(i);
    steerSampleValues(i).steer_angle_rad = steerActualAngles(i);
    steerSampleValues(i).camber_rad = camberSteer(i);
    steerSampleValues(i).toe_rad = toeSteer(i);
    steerSampleValues(i).residual_norm = steerResiduals(i);
    steerSampleValues(i).iterations = steerIterations(i);
    steerSampleValues(i).exitflag = steerExitflags(i);
    steerSampleValues(i).converged = steerFlags(i);
end
steerConverged = all(steerFlags);
steerBranchContinuous = all(isfinite(steerActualAngles)) && ...
    all(diff(steerActualAngles) > -1e-6);
steerStatus = sample_status(steerFlags, steerResiduals, steerIterations, steerExitflags);
if useClosedLink
    steerStatus.mode = 'closed_link_rack';
    steerStatus.rack_samples = rackSamples;
else
    steerStatus.mode = 'rigid_proxy';
    steerStatus.target_steer_angles = steerSamples;
end
steerStatus.rack_displacements = steerRack;
steerStatus.actual_steer_angles = steerActualAngles;
if ~useClosedLink
    steerStatus.steer_errors = steerActualAngles - steerSamples;
end
steerStatus.samples = steerSampleStatus;

rightGeom = mirror_wishbone_geometry(geom);
rollSamples = reshape(opts.RollSamples, 1, []);
rollCenterHeights = zeros(size(rollSamples));
leftRollCamber = zeros(size(rollSamples));
leftRollToe = zeros(size(rollSamples));
halfTrack = 0.5 * max(opts.TrackWidth, 1e-6);
rollConverged = true;
rollFlags = false(size(rollSamples));
rollResiduals = zeros(size(rollSamples));
rollIterations = zeros(size(rollSamples));
rollExitflags = zeros(size(rollSamples));
rollLeftGuess = [];
rollRightGuess = [];
rollSampleValues = repmat(struct('roll_rad', 0, 'roll_center_height_m', 0, ...
    'left_camber_rad', 0, 'left_toe_rad', 0, 'residual_norm', 0, ...
    'iterations', 0, 'exitflag', 0, 'converged', false), 1, numel(rollSamples));
for i = 1:numel(rollSamples)
    phi = rollSamples(i);
    dzLeft = halfTrack * phi;
    dzRight = -halfTrack * phi;
    leftPose = solve_pose_for_wheel_center_z(geom, geom.W_C(3) + dzLeft, ...
        opts.UseTieRod, rollLeftGuess, opts.ResidualTolerance);
    rightPose = solve_pose_for_wheel_center_z(rightGeom, rightGeom.W_C(3) + dzRight, ...
        opts.UseTieRod, rollRightGuess, opts.ResidualTolerance);
    if leftPose.converged
        rollLeftGuess = leftPose.solver_vars;
    end
    if rightPose.converged
        rollRightGuess = rightPose.solver_vars;
    end
    rollFlags(i) = leftPose.converged && rightPose.converged;
    rollResiduals(i) = max(leftPose.residual_norm, rightPose.residual_norm);
    rollIterations(i) = max(leftPose.iterations, rightPose.iterations);
    rollExitflags(i) = min(leftPose.exitflag, rightPose.exitflag);
    rollConverged = rollConverged && rollFlags(i);
    rollCenterHeights(i) = compute_axle_roll_center_height(geom, rightGeom, leftPose, rightPose);
    leftRollCamber(i) = compute_camber_from_pose(geom, leftPose) + staticCamberOffsetRad;
    leftRollToe(i) = compute_toe_from_pose(geom, leftPose) + opts.StaticToe;
    rollSampleValues(i).roll_rad = phi;
    rollSampleValues(i).roll_center_height_m = rollCenterHeights(i);
    rollSampleValues(i).left_camber_rad = leftRollCamber(i);
    rollSampleValues(i).left_toe_rad = leftRollToe(i);
    rollSampleValues(i).residual_norm = rollResiduals(i);
    rollSampleValues(i).iterations = rollIterations(i);
    rollSampleValues(i).exitflag = rollExitflags(i);
    rollSampleValues(i).converged = rollFlags(i);
end
rollStatus = sample_status(rollFlags, rollResiduals, rollIterations, rollExitflags);

camberTravelCoeffs = polyfit(travelFromEq, camberTravel, min(2, numel(travelSamples) - 1));
toeTravelCoeffs = polyfit(travelFromEq, toeTravel, min(2, numel(travelSamples) - 1));
camberSteerCoeffs = polyfit(steerActualAngles, camberSteer, min(2, nSteerSamples - 1));
toeSteerCoeffs = polyfit(steerActualAngles, toeSteer, min(2, nSteerSamples - 1));
[rcTravelValues, rcConverged, rcStatus] = sample_rc_with_heave(geom, rightGeom, ...
    travelSamples, opts.UseTieRod, opts.ResidualTolerance);
rcTravelCoeffs = polyfit(travelFromEq, rcTravelValues, min(1, numel(travelSamples) - 1));
rcRollCoeffs = polyfit(rollSamples, rollCenterHeights, min(1, numel(rollSamples) - 1));

polyfitQuality = struct( ...
    'travel_camber_max_error_rad', max_fit_error(camberTravelCoeffs, travelFromEq, camberTravel), ...
    'travel_toe_max_error_rad', max_fit_error(toeTravelCoeffs, travelFromEq, toeTravel), ...
    'steer_camber_max_error_rad', max_fit_error(camberSteerCoeffs, steerActualAngles, camberSteer), ...
    'steer_toe_max_error_rad', max_fit_error(toeSteerCoeffs, steerActualAngles, toeSteer), ...
    'rc_travel_max_error_m', max_fit_error(rcTravelCoeffs, travelFromEq, rcTravelValues), ...
    'rc_roll_max_error_m', max_fit_error(rcRollCoeffs, rollSamples, rollCenterHeights));
polyfitQuality.fit_ok = polyfitQuality.travel_camber_max_error_rad <= 0.02 ...
    && polyfitQuality.travel_toe_max_error_rad <= 0.02 ...
    && polyfitQuality.steer_camber_max_error_rad <= 0.02 ...
    && polyfitQuality.steer_toe_max_error_rad <= 0.02 ...
    && polyfitQuality.rc_travel_max_error_m <= 0.005 ...
    && polyfitQuality.rc_roll_max_error_m <= 0.005;

kcChecks = struct( ...
    'travel_sample_count', numel(travelSamples), ...
    'roll_sample_count', numel(rollSamples), ...
    'steer_sample_count', nSteerSamples, ...
    'travel_residual_ok', all(travelResiduals <= opts.ResidualTolerance), ...
    'roll_residual_ok', all(rollResiduals <= opts.ResidualTolerance), ...
    'steer_residual_ok', all(steerResiduals <= opts.ResidualTolerance), ...
    'travel_toe_extrema_count', count_extrema(toeTravel), ...
    'travel_camber_extrema_count', count_extrema(camberTravel), ...
    'travel_toe_range_rad', max(toeTravel) - min(toeTravel), ...
    'travel_camber_range_rad', max(camberTravel) - min(camberTravel), ...
    'roll_center_range_m', [min(rollCenterHeights), max(rollCenterHeights)], ...
    'steer_angle_range_rad', max(steerActualAngles) - min(steerActualAngles));

% Side-view swing-arm IC from the projected chassis pivot axes. Outboard
% brakes react at the contact patch, inboard drive at the wheel centre.
svic = compute_side_view_ic(geom);
leverRatio = opts.Wheelbase / max(opts.CGHeight, 1e-6);
antiBrakeTravel = zeros(size(travelSamples));
antiDriveTravel = zeros(size(travelSamples));
for i = 1:numel(travelSamples)
    antiBrakeTravel(i) = side_view_tan_theta(svic, travelContactPatch(:, i), geom.is_front) * leverRatio;
    antiDriveTravel(i) = side_view_tan_theta(svic, travelWheelCenter(:, i), geom.is_front) * leverRatio;
end
anti = compute_anti_metrics( ...
    side_view_tan_theta(svic, staticMetrics.contact_patch, geom.is_front) * leverRatio, ...
    side_view_tan_theta(svic, nominalPose.W_C, geom.is_front) * leverRatio, ...
    geom.is_front, opts.BrakeBiasFront, opts.DriveBiasRear);
antiBrakeCoeffs = polyfit(travelFromEq, antiBrakeTravel, min(2, numel(travelSamples) - 1));
antiDriveCoeffs = polyfit(travelFromEq, antiDriveTravel, min(2, numel(travelSamples) - 1));
compliance = estimate_compliance_proxy(geom, opts.ComplianceModel, opts.SteeringStiffness);

metrics = struct();
metrics.camber_deg = staticMetrics.camber_deg;
metrics.caster_deg = staticMetrics.caster_deg;
metrics.kpi_deg = staticMetrics.kpi_deg;
metrics.scrub_radius_m = staticMetrics.scrub_radius_m;
metrics.mechanical_trail_m = staticMetrics.mechanical_trail_m;
metrics.caster_trail_m = staticMetrics.mechanical_trail_m;
metrics.roll_center_z = staticMetrics.roll_center_z;
metrics.IC_y = staticMetrics.IC_y;
metrics.IC_z = staticMetrics.IC_z;
metrics.contact_patch = staticMetrics.contact_patch;
metrics.kingpin_pierce_point = staticMetrics.kingpin_pierce_point;
metrics.travel_samples = travelSamples;
metrics.steer_samples = steerActualAngles;
metrics.steer_kingpin_directions = steerKingpin;
if useClosedLink
    metrics.steer_target_samples = [];
    metrics.rack_samples = rackSamples;
else
    metrics.steer_target_samples = steerSamples;
    metrics.rack_samples = [];
end
metrics.rack_displacements = steerRack;
metrics.roll_samples = rollSamples;
metrics.camber_vs_travel = ensure_quadratic(camberTravelCoeffs);
metrics.toe_vs_travel = ensure_quadratic(toeTravelCoeffs);
metrics.camber_vs_steer = ensure_quadratic(camberSteerCoeffs);
metrics.toe_vs_steer = ensure_quadratic(toeSteerCoeffs);
% Roll gains are residuals after removing what camber/toe_vs_travel already
% give at the roll-induced travel (runtime evaluates both; #38). The single-
% corner roll pose equals a heave pose, so the residual is only fit error.
rollTravelFromEq = -halfTrack * rollSamples;
rollCamberResidual = leftRollCamber - polyval(camberTravelCoeffs, rollTravelFromEq);
rollToeResidual = leftRollToe - polyval(toeTravelCoeffs, rollTravelFromEq);
metrics.roll_camber_gain = safe_linear_coeff(rollSamples, rollCamberResidual);
metrics.roll_steer_gain = safe_linear_coeff(rollSamples, rollToeResidual);
metrics.roll_camber_gain_total = safe_linear_coeff(rollSamples, leftRollCamber);
metrics.roll_steer_gain_total = safe_linear_coeff(rollSamples, leftRollToe);
metrics.roll_gain_definition = 'residual_after_travel_curve';
metrics.roll_center_height_per_travel = rcTravelCoeffs(1);
metrics.roll_center_height_per_roll = rcRollCoeffs(1);
metrics.jacking_force_per_fy = staticMetrics.roll_center_z / max(halfTrack, 1e-6);
metrics.anti_dive_pct = anti.dive;
metrics.anti_squat_pct = anti.squat;
metrics.anti_lift_pct = anti.lift;
metrics.anti_rise_pct = anti.rise;
metrics.anti_brake_pct = anti.brake;
metrics.anti_drive_pct = anti.drive;
% anti_brake/anti_drive are per-axle force fractions (tan(theta)*L/h, no bias):
% the runtime applies them to each wheel's own Fx. dive/rise/squat/lift are
% the classical bias-weighted percentages for reporting.
metrics.anti_brake_vs_travel = ensure_quadratic(antiBrakeCoeffs);
metrics.anti_drive_vs_travel = ensure_quadratic(antiDriveCoeffs);
metrics.anti_brake_curve = antiBrakeTravel;
metrics.anti_drive_curve = antiDriveTravel;
metrics.side_view_ic = svic;
metrics.anti_model = 'side_view_pivot_axes_ic';
metrics.anti_validated = false;
metrics.jacking_model = 'roll_center_derived_proxy';
metrics.jacking_validated = false;
metrics.compliance = compliance;
metrics.compliance_model = 'arm_axial_stiffness_proxy';
metrics.compliance_validated = false;
metrics.sample_values = struct( ...
    'travel', travelSampleValues, ...
    'roll', rollSampleValues, ...
    'steer', steerSampleValues);
metrics.polyfit_quality = polyfitQuality;
metrics.kc_checks = kcChecks;
metrics.travel_camber_curve = camberTravel;
metrics.travel_toe_curve = toeTravel;
metrics.steer_camber_curve = camberSteer;
metrics.steer_toe_curve = toeSteer;
metrics.roll_center_curve = rollCenterHeights;
metrics.camber_gain_rad_m = metrics.camber_vs_travel(2);
metrics.solver_status = struct( ...
    'travel', travelStatus, ...
    'steer', steerStatus, ...
    'roll', rollStatus, ...
    'rc_heave', rcStatus, ...
    'branch_continuous', travelBranchContinuous && steerBranchContinuous, ...
    'travel_branch_continuous', travelBranchContinuous, ...
    'steer_branch_continuous', steerBranchContinuous, ...
    'branch_checks', struct( ...
        'travel_toe_tolerance_rad', opts.BranchToeTolerance, ...
        'travel_camber_tolerance_rad', opts.BranchCamberTolerance, ...
        'travel_kingpin_tolerance_rad', opts.BranchKingpinTolerance), ...
    'residual_tolerance', opts.ResidualTolerance, ...
    'max_residual_norm', max([travelStatus.max_residual_norm, ...
        steerStatus.max_residual_norm, rollStatus.max_residual_norm, ...
        rcStatus.max_residual_norm]));
metrics.max_residual_norm = metrics.solver_status.max_residual_norm;
metrics.fsolve_calls = numel(travelSamples) + 2 * numel(travelSamples) ...
    + 2 * numel(rollSamples) + (useClosedLink * nSteerSamples);
metrics.solver_status.fsolve_calls = metrics.fsolve_calls;
metrics.converged = travelConverged && steerConverged && rollConverged && ...
    rcConverged && travelBranchContinuous && steerBranchContinuous;
metrics.elapsed_s = toc(eval_start);
end

function [values, converged, status] = sample_rc_with_heave(leftGeom, rightGeom, ...
        travelSamples, useTieRod, residualTolerance)
values = zeros(size(travelSamples));
flags = false(size(travelSamples));
residuals = zeros(size(travelSamples));
iterations = zeros(size(travelSamples));
exitflags = zeros(size(travelSamples));
leftGuess = [];
rightGuess = [];
for i = 1:numel(travelSamples)
    dz = travelSamples(i);
    leftPose = solve_pose_for_wheel_center_z(leftGeom, leftGeom.W_C(3) + dz, ...
        useTieRod, leftGuess, residualTolerance);
    rightPose = solve_pose_for_wheel_center_z(rightGeom, rightGeom.W_C(3) + dz, ...
        useTieRod, rightGuess, residualTolerance);
    if leftPose.converged
        leftGuess = leftPose.solver_vars;
    end
    if rightPose.converged
        rightGuess = rightPose.solver_vars;
    end
    flags(i) = leftPose.converged && rightPose.converged;
    residuals(i) = max(leftPose.residual_norm, rightPose.residual_norm);
    iterations(i) = max(leftPose.iterations, rightPose.iterations);
    exitflags(i) = min(leftPose.exitflag, rightPose.exitflag);
    values(i) = compute_axle_roll_center_height(leftGeom, rightGeom, leftPose, rightPose);
end
converged = all(flags);
status = sample_status(flags, residuals, iterations, exitflags);
end

function pose = build_nominal_pose(geom)
pose.U_U = geom.U_U;
pose.L_U = geom.L_U;
pose.W_C = geom.W_C;
pose.R_upright = eye(3);
pose.t_upright = zeros(3, 1);
pose.converged = true;
if isfield(geom, 'tie_rod_upright')
    pose.TR_U = geom.tie_rod_upright;
else
    pose.TR_U = [];
end
end

function pose = solve_pose_for_wheel_center_z(geom, targetWheelZ, useTieRod, ...
        initialGuess, residualTolerance, tieRodLength)
if nargin < 4
    initialGuess = [];
end
if nargin < 5 || isempty(residualTolerance)
    residualTolerance = 1e-6;
end
if nargin < 6
    tieRodLength = [];
end

U0 = geom.U_U;
L0 = geom.L_U;
W0 = geom.W_C;
L_kingpin = norm(U0 - L0);
options = optimoptions('fsolve', 'Display', 'off', ...
    'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12);

if useTieRod && isfield(geom, 'tie_rod_upright') && isfield(geom, 'tie_rod_chassis')
    TR0 = geom.tie_rod_upright;
    if isempty(tieRodLength)
        tieLength = norm(geom.tie_rod_chassis - TR0);
    else
        tieLength = tieRodLength;
    end
    guess = normalize_initial_guess(initialGuess, 3);
    [x, fval, exitflag, output] = fsolve(@(vars) travel_residual_3(vars, geom, ...
        U0, L0, W0, TR0, L_kingpin, tieLength, targetWheelZ), guess, options);
    theta_u = x(1);
    theta_l = x(2);
    phi = x(3);
else
    guess = normalize_initial_guess(initialGuess, 2);
    [x, fval, exitflag, output] = fsolve(@(vars) travel_residual_2(vars, geom, ...
        U0, L0, W0, L_kingpin, targetWheelZ), guess, options);
    theta_u = x(1);
    theta_l = x(2);
    phi = 0.0;
    TR0 = [];
end

[U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, theta_u, theta_l, phi);
pose = struct();
pose.U_U = U_U;
pose.L_U = L_U;
pose.R_upright = R_upright;
pose.t_upright = t_upright;
pose.W_C = R_upright * W0 + t_upright;
pose.solver_vars = x(:);
pose.fval = fval(:);
pose.raw_residual_norm = norm(fval, inf);
if numel(fval) == 3
    residual_scale = [max(L_kingpin^2, 1e-6); 1.0; max(tieLength^2, 1e-6)];
else
    residual_scale = [max(L_kingpin^2, 1e-6); 1.0];
end
pose.residual_scale = residual_scale(:);
pose.residual_norm = max(abs(fval(:) ./ pose.residual_scale));
pose.iterations = output.iterations;
pose.exitflag = exitflag;
pose.converged = exitflag > 0 && all(isfinite(x)) && ...
    pose.residual_norm <= residualTolerance;
if ~isempty(TR0)
    pose.TR_U = R_upright * TR0 + t_upright;
else
    pose.TR_U = [];
end
end

function guess = normalize_initial_guess(initialGuess, n)
if isempty(initialGuess) || numel(initialGuess) ~= n || ~all(isfinite(initialGuess(:)))
    guess = zeros(n, 1);
else
    guess = initialGuess(:);
end
end

function status = sample_status(flags, residuals, iterations, exitflags)
status = struct();
status.converged = all(flags);
status.failed_samples = find(~flags);
status.exitflags = exitflags(:)';
status.residual_norms = residuals(:)';
status.iterations = iterations(:)';
finite_residuals = residuals(isfinite(residuals));
status.max_residual_norm = max([finite_residuals(:); 0.0]);
end

function ok = branch_is_continuous(camber, toe, kingpin, camberTol, toeTol, kingpinTol)
ok = true;
if numel(camber) < 2 || numel(toe) < 2
    return;
end
if any(~isfinite(camber)) || any(~isfinite(toe))
    ok = false;
    return;
end
if max(abs(diff(unwrap(camber(:))))) > camberTol
    ok = false;
end
if max(abs(diff(unwrap(toe(:))))) > toeTol
    ok = false;
end
if size(kingpin, 2) >= 2
    kn = kingpin ./ max(vecnorm(kingpin, 2, 1), 1e-9);
    dots = sum(kn(:, 1:end-1) .* kn(:, 2:end), 1);
    if any(dots < cos(kingpinTol))
        ok = false;
    end
end
end

function pose = solve_pose_for_rack(geom, rackDisplacement, targetWheelZ, initialGuess, residualTolerance)
geomSteer = geom;
geomSteer.tie_rod_chassis = geom.tie_rod_chassis + [0; rackDisplacement; 0];
tieLength = norm(geom.tie_rod_chassis - geom.tie_rod_upright);
pose = solve_pose_for_wheel_center_z(geomSteer, targetWheelZ, true, ...
    initialGuess, residualTolerance, tieLength);
pose.rack_displacement = rackDisplacement;
pose.steer_angle = compute_steer_from_pose(geom, pose);
end

function steerAngle = compute_steer_from_pose(geom, pose)
spindleNominal = get_spindle_nominal(geom);
spindleNow = pose.R_upright * spindleNominal;
v0 = spindleNominal(1:2);
v = spindleNow(1:2);
steerAngle = atan2(v0(1) * v(2) - v0(2) * v(1), dot(v0, v));
end

function pose = solve_pose_for_steer_rigid(geom, steerAngle)
U0 = geom.U_U;
L0 = geom.L_U;
kp_dir = (U0 - L0) / max(norm(U0 - L0), 1e-9);
R_kp = axis_angle_rotation_local(kp_dir, steerAngle);
t_up = L0 - R_kp * L0;
pose = struct();
pose.U_U = R_kp * U0 + t_up;
pose.L_U = R_kp * L0 + t_up;
pose.W_C = R_kp * geom.W_C + t_up;
pose.R_upright = R_kp;
pose.t_upright = t_up;
pose.solver_vars = [];
pose.fval = zeros(0, 1);
pose.residual_norm = 0.0;
pose.iterations = 0;
pose.exitflag = 1;
pose.converged = true;
if isfield(geom, 'tie_rod_upright')
    pose.TR_U = R_kp * geom.tie_rod_upright + t_up;
else
    pose.TR_U = [];
end
end

function res = travel_residual_2(vars, geom, U0, L0, W0, L_kingpin, targetWheelZ)
theta_u = vars(1);
theta_l = vars(2);
[U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, theta_u, theta_l, 0.0);
W_C = R_upright * W0 + t_upright;
res = [
    dot(U_U - L_U, U_U - L_U) - L_kingpin^2;
    W_C(3) - targetWheelZ
];
end

function res = travel_residual_3(vars, geom, U0, L0, W0, TR0, L_kingpin, tieLength, targetWheelZ)
theta_u = vars(1);
theta_l = vars(2);
phi = vars(3);
[U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, theta_u, theta_l, phi);
W_C = R_upright * W0 + t_upright;
TR_U = R_upright * TR0 + t_upright;
res = [
    dot(U_U - L_U, U_U - L_U) - L_kingpin^2;
    W_C(3) - targetWheelZ;
    dot(geom.tie_rod_chassis - TR_U, geom.tie_rod_chassis - TR_U) - tieLength^2
];
end

function [U_U, L_U, R_upright, t_upright] = build_upright_state(U0, L0, geom, theta_u, theta_l, phi)
U_U = rotate_point_around_axis_local(U0, geom.upper_arm_axis, geom.upper_arm_axis_origin, theta_u);
L_U = rotate_point_around_axis_local(L0, geom.lower_arm_axis, geom.lower_arm_axis_origin, theta_l);
kp_ref = geom.kingpin_axis.direction;
kp_new = (U_U - L_U) / max(norm(U_U - L_U), 1e-9);
R_align = rotation_between_vectors_local(kp_ref, kp_new);
R_spin = axis_angle_rotation_local(kp_new, phi);
R_upright = R_spin * R_align;
mid_ref = (U0 + L0) / 2;
mid_cur = (U_U + L_U) / 2;
t_upright = mid_cur - R_upright * mid_ref;
end

function staticMetrics = compute_static_metrics(geom, pose, staticToe)
U_C = (geom.U_F + geom.U_R) / 2;
L_C = (geom.L_F + geom.L_R) / 2;
K_vec = pose.U_U - pose.L_U;
K_dir = K_vec / max(norm(K_vec), 1e-9);
K_down = -K_dir;

staticMetrics = struct();
staticMetrics.caster_deg = atan2d(K_down(1), K_down(3));
if geom.is_left
    staticMetrics.kpi_deg = atan2d(-K_down(2), K_down(3));
else
    staticMetrics.kpi_deg = atan2d(K_down(2), K_down(3));
end

contactPatch = pose.W_C;
contactPatch(3) = contactPatch(3) + geom.R_tire;
staticMetrics.contact_patch = contactPatch;
tp = signed_divide(contactPatch(3) - pose.L_U(3), K_dir(3));
pierce = pose.L_U + tp * K_dir;
staticMetrics.kingpin_pierce_point = pierce;
if geom.is_left
    staticMetrics.scrub_radius_m = pierce(2) - contactPatch(2);
else
    staticMetrics.scrub_radius_m = contactPatch(2) - pierce(2);
end
staticMetrics.mechanical_trail_m = pierce(1) - contactPatch(1);

dirUp = [pose.U_U(2) - U_C(2), pose.U_U(3) - U_C(3)];
dirLow = [pose.L_U(2) - L_C(2), pose.L_U(3) - L_C(3)];
A = [dirUp(1), -dirLow(1); dirUp(2), -dirLow(2)];
b = [pose.L_U(2) - pose.U_U(2); pose.L_U(3) - pose.U_U(3)];
if abs(det(A)) < 1e-9
    if dirUp(1) >= 0
        staticMetrics.IC_y = 1e6;
    else
        staticMetrics.IC_y = -1e6;
    end
    staticMetrics.IC_z = pose.W_C(3);
    slope = signed_divide(dirUp(2), dirUp(1));
    staticMetrics.roll_center_z = contactPatch(3) + (-contactPatch(2)) * slope;
else
    sol = A \ b;
    staticMetrics.IC_y = pose.U_U(2) + sol(1) * dirUp(1);
    staticMetrics.IC_z = pose.U_U(3) + sol(1) * dirUp(2);
    slopeIcTcp = signed_divide(staticMetrics.IC_z - contactPatch(3), staticMetrics.IC_y - contactPatch(2));
    staticMetrics.roll_center_z = contactPatch(3) + slopeIcTcp * (0 - contactPatch(2));
end

staticMetrics.camber_deg = rad2deg(compute_camber_from_pose(geom, pose));
staticMetrics.toe_rad = compute_toe_from_pose(geom, pose) + staticToe;
end

function rcHeight = compute_axle_roll_center_height(leftGeom, rightGeom, leftPose, rightPose)
[leftIC, leftCP] = compute_side_ic(leftGeom, leftPose);
[rightIC, rightCP] = compute_side_ic(rightGeom, rightPose);

A = [leftIC(1) - leftCP(1), -(rightIC(1) - rightCP(1)); ...
     leftIC(2) - leftCP(2), -(rightIC(2) - rightCP(2))];
b = [rightCP(1) - leftCP(1); rightCP(2) - leftCP(2)];
if abs(det(A)) < 1e-9
    rcHeight = 0.5 * (leftCP(2) + rightCP(2));
else
    sol = A \ b;
    rcPoint = leftCP + sol(1) * (leftIC - leftCP);
    rcHeight = rcPoint(2);
end
end

function [ic, cp] = compute_side_ic(geom, pose)
U_C = (geom.U_F + geom.U_R) / 2;
L_C = (geom.L_F + geom.L_R) / 2;
dirUp = [pose.U_U(2) - U_C(2), pose.U_U(3) - U_C(3)];
dirLow = [pose.L_U(2) - L_C(2), pose.L_U(3) - L_C(3)];
A = [dirUp(1), -dirLow(1); dirUp(2), -dirLow(2)];
b = [pose.L_U(2) - pose.U_U(2); pose.L_U(3) - pose.U_U(3)];
if abs(det(A)) < 1e-9
    if dirUp(1) >= 0
        ic = [1e6, pose.W_C(3)];
    else
        ic = [-1e6, pose.W_C(3)];
    end
else
    sol = A \ b;
    ic = [pose.U_U(2) + sol(1) * dirUp(1), pose.U_U(3) + sol(1) * dirUp(2)];
end
cp3 = pose.W_C(3) + geom.R_tire;
cp = [pose.W_C(2), cp3];
ic = [ic(1), ic(2)];
end

function svic = compute_side_view_ic(geom)
% Intersection of the upper and lower pivot axes projected on X-Z. Parallel
% axes put the IC at infinity along their common direction (dir, X > 0).
pu = [geom.U_F(1); geom.U_F(3)];
du = [geom.U_R(1) - geom.U_F(1); geom.U_R(3) - geom.U_F(3)];
pl = [geom.L_F(1); geom.L_F(3)];
dl = [geom.L_R(1) - geom.L_F(1); geom.L_R(3) - geom.L_F(3)];
A = [du, -dl];
dir = du / max(norm(du), 1e-12);
if dir(1) < 0
    dir = -dir;
end
svic = struct('at_infinity', false, 'point_xz', [NaN, NaN], 'dir_xz', dir(:)');
if abs(det(A)) < 1e-9 * max(norm(du) * norm(dl), 1e-12)
    svic.at_infinity = true;
else
    s = A \ (pl - pu);
    svic.point_xz = (pu + s(1) * du)';
end
end

function t = side_view_tan_theta(svic, refPoint, isFront)
% tan(theta) of the reaction line from refPoint to the side-view IC. Positive
% means anti (front: IC behind and above; rear: IC ahead and above, Z down).
if svic.at_infinity
    t = svic.dir_xz(2) / max(svic.dir_xz(1), 1e-12);
    if ~isFront
        t = -t;
    end
    return;
end
dz = refPoint(3) - svic.point_xz(2);
if isFront
    t = signed_divide(dz, refPoint(1) - svic.point_xz(1));
else
    t = signed_divide(dz, svic.point_xz(1) - refPoint(1));
end
end

function anti = compute_anti_metrics(brakeFraction, driveFraction, isFront, brakeBias, driveBias)
if isFront
    brake = brakeFraction;
    drive = driveFraction;
    dive = brakeFraction * brakeBias;
    lift = driveFraction * (1.0 - driveBias);
    rise = 0.0;
    squat = 0.0;
else
    brake = brakeFraction;
    drive = driveFraction;
    rise = brakeFraction * (1.0 - brakeBias);
    squat = driveFraction * driveBias;
    dive = 0.0;
    lift = 0.0;
end
anti = struct('dive', dive, 'squat', squat, 'lift', lift, 'rise', rise, ...
    'brake', brake, 'drive', drive, ...
    'brake_bias_front', brakeBias, 'drive_bias_rear', driveBias);
end

function cp = compute_contact_patch(geom, pose)
% Ground contact point: wheel centre plus the loaded radius along the wheel
% plane's downward direction (includes the R*sin(camber) lateral shift).
spindle = pose.R_upright * get_spindle_nominal(geom);
down = [0; 0; 1] - spindle(3) * spindle;
down = down / max(norm(down), 1e-9);
radius = geom.R_tire;
if isfield(geom, 'R_loaded') && ~isempty(geom.R_loaded)
    radius = geom.R_loaded;
end
cp = pose.W_C + radius * down;
end

function comp = estimate_compliance_proxy(geom, model, steeringStiffness)
if ~isfield(model, 'youngs_modulus'), model.youngs_modulus = 2.0e11; end
if ~isfield(model, 'outer_diameter_m'), model.outer_diameter_m = 0.01905; end
if ~isfield(model, 'wall_thickness_m'), model.wall_thickness_m = 0.0016; end

Do = model.outer_diameter_m;
Di = max(Do - 2 * model.wall_thickness_m, 1e-4);
A = pi / 4 * (Do^2 - Di^2);

U_C = (geom.U_F + geom.U_R) / 2;
L_C = (geom.L_F + geom.L_R) / 2;
upperLen = norm(geom.U_U - U_C);
lowerLen = norm(geom.L_U - L_C);
upperK = model.youngs_modulus * A / max(upperLen, 1e-6);
lowerK = model.youngs_modulus * A / max(lowerLen, 1e-6);
verticalSep = max(abs(geom.U_U(3) - geom.L_U(3)), 1e-6);

upperDir = (geom.U_U - U_C) / max(norm(geom.U_U - U_C), 1e-9);
lowerDir = (geom.L_U - L_C) / max(norm(geom.L_U - L_C), 1e-9);

deltaFy = upperDir(2) / upperK - lowerDir(2) / lowerK;
deltaFx = upperDir(1) / upperK - lowerDir(1) / lowerK;

comp = struct();
comp.model = 'arm_axial_stiffness_proxy';
comp.validated = false;
comp.units = 'toe/camber/caster: rad/N; track/wheelbase: m/N';
comp.basis = 'wheel center, SAE J670e'; ...
comp.sign_convention = 'positive Fx/Fy in SAE axes; upper-minus-lower axial compliance';
comp.toe_per_load = zeros(1, 6);
comp.camber_per_load = zeros(1, 6);
comp.caster_per_load = zeros(1, 6);
comp.track_per_load = zeros(1, 6);
comp.wheelbase_per_load = zeros(1, 6);
comp.camber_per_load(2) = deltaFy / verticalSep;
comp.toe_per_load(1) = deltaFx / verticalSep;
if geom.is_front
    comp.toe_per_load(6) = 1.0 / max(steeringStiffness, 1e-6);
end
end

function camber = compute_camber_from_pose(geom, pose)
spindleNominal = get_spindle_nominal(geom);
spindleNow = pose.R_upright * spindleNominal;
camber = asin(max(-1.0, min(1.0, spindleNow(3))));
end

function toe = compute_toe_from_pose(geom, pose)
spindleNominal = get_spindle_nominal(geom);
spindleNow = pose.R_upright * spindleNominal;
if geom.is_left
    toe = atan2(spindleNow(1), -spindleNow(2));
else
    toe = atan2(-spindleNow(1), spindleNow(2));
end
end

function spindleNominal = get_spindle_nominal(geom)
if isfield(geom, 'Spindle_Dir')
    spindleNominal = geom.Spindle_Dir / max(norm(geom.Spindle_Dir), 1e-9);
elseif geom.is_left
    spindleNominal = [0; -1; 0];
else
    spindleNominal = [0; 1; 0];
end
end

function coeffs = ensure_quadratic(polyCoeffs)
polyCoeffs = reshape(polyCoeffs, 1, []);
if numel(polyCoeffs) == 3
    coeffs = [polyCoeffs(3), polyCoeffs(2), polyCoeffs(1)];
elseif numel(polyCoeffs) == 2
    coeffs = [polyCoeffs(2), polyCoeffs(1), 0.0];
else
    coeffs = [polyCoeffs(end), 0.0, 0.0];
end
end

function coeff = safe_linear_coeff(x, y)
if numel(x) < 2
    coeff = 0.0;
    return;
end
p = polyfit(x, y, 1);
coeff = p(1);
end

function pRot = rotate_point_around_axis_local(p, axis, axisOrigin, angle)
v = p - axisOrigin;
R = axis_angle_rotation_local(axis, angle);
pRot = axisOrigin + R * v;
end

function R = rotation_between_vectors_local(a, b)
a = a / max(norm(a), 1e-9);
b = b / max(norm(b), 1e-9);
v = cross(a, b);
s = norm(v);
c = dot(a, b);
if s < 1e-12
    if c > 0
        R = eye(3);
    else
        perp = get_perpendicular_local(a);
        R = axis_angle_rotation_local(perp, pi);
    end
else
    vx = [0, -v(3), v(2); v(3), 0, -v(1); -v(2), v(1), 0];
    R = eye(3) + vx + vx * vx * (1 - c) / (s * s);
end
end

function perp = get_perpendicular_local(v)
if abs(v(1)) < 0.9
    perp = cross(v, [1; 0; 0]);
else
    perp = cross(v, [0; 1; 0]);
end
perp = perp / max(norm(perp), 1e-9);
end

function R = axis_angle_rotation_local(axis, angle)
axis = axis / max(norm(axis), 1e-9);
x = axis(1); y = axis(2); z = axis(3);
c = cos(angle);
s = sin(angle);
C = 1 - c;
R = [ ...
    x*x*C + c,     x*y*C - z*s, x*z*C + y*s; ...
    y*x*C + z*s, y*y*C + c,     y*z*C - x*s; ...
    z*x*C - y*s, z*y*C + x*s, z*z*C + c];
end

function v = signed_divide(a, b)
if abs(b) < 1e-12
    if b < 0
        b = -1e-12;
    else
        b = 1e-12;
    end
end
v = a / b;
end

function err = max_fit_error(coeffs, x, y)
if isempty(x)
    err = 0.0;
    return;
end
err = max(abs(polyval(coeffs, x(:)) - y(:)));
end

function n = count_extrema(y)
d = diff(y(:));
d = d(d ~= 0);
if numel(d) < 2
    n = 0;
    return;
end
n = sum(diff(sign(d)) ~= 0);
end
