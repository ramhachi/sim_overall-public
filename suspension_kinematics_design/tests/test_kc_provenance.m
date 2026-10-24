classdef test_kc_provenance < matlab.unittest.TestCase
    % kc.provenance block and per-corner verdicts survive the artifact save (#39).
    methods (Test)
        function provenanceSummarisesCornerVerdicts(tc)
            left = build_test_geometry();
            withTie = evaluate_double_wishbone_3d(left, 'UseTieRod', true);
            noTie = evaluate_double_wishbone_3d(left, 'UseTieRod', false);
            params = struct();
            params.suspension.kc.corners.FL = build_corner_kc(left, withTie, 0, 0, 'test');
            params.suspension.kc.corners.FR = build_corner_kc(left, noTie, 0, 0, 'test');

            out = [tempname, '.json'];
            cleanup = onCleanup(@() delete_if(out));
            save_suspension_geometry_artifact('OutputPath', out, 'CurrentParams', params);
            data = load_vehicle_artifact(out);
            prov = data.params.suspension.kc.provenance;

            tc.verifyEqual(prov.schema_version, 'kc_provenance/v1');
            tc.verifyEqual(prov.corners.FL.verdict, 'valid');
            tc.verifyEqual(prov.corners.FR.verdict, 'provisional');
            tc.verifyTrue(any(strcmp(prov.corners.FR.reasons, 'tie_rod_not_evaluated')));
            tc.verifyEqual(prov.corners.FR.evaluation_scope.toe, 'not_evaluated');
            tc.verifyEqual(sort(cellstr(prov.missing_corners(:))'), {'RL', 'RR'});
            tc.verifyFalse(prov.production_ready);
            tc.verifyTrue(prov.compliance_provisional);
            % Per-corner provenance is persisted too.
            fl = data.params.suspension.kc.corners.FL;
            tc.verifyEqual(fl.source.script, 'test');
            tc.verifyEqual(fl.evaluation_scope.compliance, 'model_default_stiffness');
        end
    end
end

function delete_if(path)
if isfile(path)
    delete(path);
end
end

function geom = build_test_geometry()
geom = struct();
geom.W_C = [0.75; -0.60; -0.228];
geom.R_tire = 0.228;
geom.is_left = true;
geom.is_front = true;
geom.U_U = [0.755; -0.55; -0.33];
geom.L_U = [0.75; -0.58; -0.11];
geom.U_F = [0.88; -0.30; -0.29];
geom.U_R = [0.62; -0.30; -0.29];
geom.L_F = [0.88; -0.25; -0.10];
geom.L_R = [0.62; -0.25; -0.10];
geom.tie_rod_chassis = [0.85; -0.27; -0.15];
geom.tie_rod_upright = [0.85; -0.57; -0.15];
geom.Spindle_Dir = [0; -1; 0];
geom.upper_arm_axis = (geom.U_R - geom.U_F) / norm(geom.U_R - geom.U_F);
geom.lower_arm_axis = (geom.L_R - geom.L_F) / norm(geom.L_R - geom.L_F);
geom.upper_arm_axis_origin = geom.U_F;
geom.lower_arm_axis_origin = geom.L_F;
geom.kingpin_axis = struct('origin', geom.L_U, ...
    'direction', (geom.U_U - geom.L_U) / norm(geom.U_U - geom.L_U));
end
