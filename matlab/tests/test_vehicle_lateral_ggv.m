classdef test_vehicle_lateral_ggv < matlab.unittest.TestCase
    % The per-vehicle GGV cache must rebuild for chassis changes and be
    % reused for changes that cannot affect lateral capability.
    methods (Test)
        function cacheKeyFollowsLateralParamsOnly(tc)
            root = tempname;
            mkdir(fullfile(root, 'bin'));
            fake = fullfile(root, 'bin', 'ggv_generator');
            % Writes the --out argument ($4) and counts invocations.
            fid = fopen(fake, 'w');
            fprintf(fid, '#!/bin/sh\necho x >> "%s"\necho "{}" > "$4"\n', fullfile(root, 'calls'));
            fclose(fid);
            system(['chmod +x ', fake]);
            config = struct('project_root', root, ...
                'binary_path', fullfile(root, 'bin', 'laptime_sim'), ...
                'ggv_cache_dir', fullfile(root, 'cache'));
            params = struct('mass', struct('h_cg', 0.28), 'powertrain', struct('fd', 3.5), ...
                'tire', struct('model_manifest', 'missing.json'));
            pp = fullfile(root, 'p.json');

            a = vehicle_lateral_ggv(pp, params, config);
            again = vehicle_lateral_ggv(pp, params, config);
            params.powertrain.fd = 4.0;
            powertrain = vehicle_lateral_ggv(pp, params, config);
            params.mass.h_cg = 0.24;
            chassis = vehicle_lateral_ggv(pp, params, config);

            tc.verifyEqual(again, a);
            tc.verifyEqual(powertrain, a);
            tc.verifyNotEqual(chassis, a);
            tc.verifyTrue(isfile(chassis));
            tc.verifyEqual(numel(splitlines(strtrim(fileread(fullfile(root, 'calls'))))), 2);
        end
    end
end
