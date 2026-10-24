classdef test_anti_metrics < matlab.unittest.TestCase
    % Side-view anti geometry from projected pivot axes (issue #37).
    properties (Constant)
        L = 1.55;
        H = 0.28;
        BIAS = 0.60;
    end

    methods (Test)
        function xParallelAxesGiveZeroAnti(tc)
            % Pivot axes parallel to X: the side-view IC is at infinity on a
            % horizontal line, so every anti metric is zero even with W_C_x ~= 0.
            for isFront = [true, false]
                geom = build_geometry(0.0, 0.0, isFront);
                m = evaluate(tc, geom);
                tc.verifyTrue(m.side_view_ic.at_infinity);
                tc.verifyEqual([m.anti_brake_pct, m.anti_drive_pct, m.anti_dive_pct, ...
                    m.anti_rise_pct, m.anti_squat_pct, m.anti_lift_pct], zeros(1, 6), 'AbsTol', 1e-9);
                tc.verifyEqual(m.anti_brake_curve, zeros(size(m.anti_brake_curve)), 'AbsTol', 1e-9);
            end
        end

        function textbookAntiDiveParallelInclinedAxes(tc)
            % Both axes inclined by theta (rear pivots higher): anti-dive =
            % tan(theta) * L/h * bias. Target 25 %.
            tanTheta = 0.25 * tc.H / (tc.L * tc.BIAS);
            m = evaluate(tc, build_geometry(tanTheta, tanTheta, true));
            tc.verifyEqual(m.anti_dive_pct, 0.25, 'RelTol', 0.02);
            tc.verifyEqual(m.anti_brake_pct, 0.25 / tc.BIAS, 'RelTol', 0.02);
            tc.verifyEqual(m.anti_rise_pct, 0.0);
        end

        function textbookAntiDiveFiniteIC(tc)
            % Axes converge to a known IC 1.5 m behind and tan(theta)*1.5 m above
            % the contact patch: anti-dive must again be 25 %.
            tanTheta = 0.25 * tc.H / (tc.L * tc.BIAS);
            geom = build_geometry(0.0, 0.0, true);
            ic = [geom.W_C(1) - 1.5; -1.5 * tanTheta];  % CP is on the ground (z = 0)
            geom = put_axes_through_ic(geom, ic);
            m = evaluate(tc, geom);
            tc.verifyFalse(m.side_view_ic.at_infinity);
            tc.verifyEqual(m.side_view_ic.point_xz, ic', 'AbsTol', 1e-9);
            tc.verifyEqual(m.anti_dive_pct, 0.25, 'RelTol', 0.02);
        end

        function rearAntiSquatUsesWheelCentreAndDriveBias(tc)
            tanTheta = 0.10;
            m = evaluate(tc, build_geometry(-tanTheta, -tanTheta, false));
            tc.verifyEqual(m.anti_drive_pct, tanTheta * tc.L / tc.H, 'RelTol', 1e-6);
            tc.verifyEqual(m.anti_squat_pct, m.anti_drive_pct, 'RelTol', 1e-12);  % RWD
            tc.verifyEqual(m.anti_rise_pct, m.anti_brake_pct * (1 - tc.BIAS), 'RelTol', 1e-12);
            tc.verifyEqual(m.anti_dive_pct, 0.0);
            tc.verifyEqual(numel(m.anti_drive_vs_travel), 3);
            tc.verifyEqual(m.anti_model, 'side_view_pivot_axes_ic');
            tc.verifyFalse(m.anti_validated);
        end
    end
end

function m = evaluate(~, geom)
m = evaluate_double_wishbone_3d(geom, 'UseTieRod', false, ...
    'Wheelbase', 1.55, 'CGHeight', 0.28, 'BrakeBiasFront', 0.60, 'DriveBiasRear', 1.0, ...
    'TravelSamples', linspace(-0.02, 0.02, 5), 'RollSamples', 0.0, 'SteerSamples', 0.0);
end

function geom = build_geometry(upperTan, lowerTan, isFront)
% Realistic left corner at x = +/-0.75 m; *Tan = dz/dx of the pivot axis going
% forward (Z down, so positive = rear pivot higher).
x0 = 0.75;
if ~isFront
    x0 = -0.80;
end
geom = struct();
geom.W_C = [x0; -0.60; -0.228];
geom.R_tire = 0.228;
geom.is_left = true;
geom.is_front = isFront;
geom.U_U = [x0 + 0.005; -0.55; -0.33];
geom.L_U = [x0; -0.58; -0.11];
geom.U_F = [x0 + 0.13; -0.30; -0.30 + 0.13 * upperTan];
geom.U_R = [x0 - 0.13; -0.30; -0.30 - 0.13 * upperTan];
geom.L_F = [x0 + 0.13; -0.25; -0.10 + 0.13 * lowerTan];
geom.L_R = [x0 - 0.13; -0.25; -0.10 - 0.13 * lowerTan];
geom.Spindle_Dir = [0; -1; 0];
geom = finish(geom);
end

function geom = put_axes_through_ic(geom, ic)
for arm = {'U', 'L'}
    a = arm{1};
    mid = (geom.([a '_F']) + geom.([a '_R'])) / 2;
    slope = (mid(3) - ic(2)) / (mid(1) - ic(1));
    geom.([a '_F'])(3) = mid(3) + slope * (geom.([a '_F'])(1) - mid(1));
    geom.([a '_R'])(3) = mid(3) + slope * (geom.([a '_R'])(1) - mid(1));
end
geom = finish(geom);
end

function geom = finish(geom)
geom.upper_arm_axis = (geom.U_R - geom.U_F) / norm(geom.U_R - geom.U_F);
geom.lower_arm_axis = (geom.L_R - geom.L_F) / norm(geom.L_R - geom.L_F);
geom.upper_arm_axis_origin = geom.U_F;
geom.lower_arm_axis_origin = geom.L_F;
geom.kingpin_axis = struct('origin', geom.L_U, ...
    'direction', (geom.U_U - geom.L_U) / norm(geom.U_U - geom.L_U));
end
