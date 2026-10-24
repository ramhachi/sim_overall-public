function run_all_tests()
% RUN_ALL_TESTS Run the frame optimization unit tests as one MATLAB batch.
%
% FEM-dependent test_frame_obj is intentionally excluded; run it separately
% with a live FEM server.

    tests_dir = fileparts(mfilename('fullpath'));
    addpath(fullfile(tests_dir, '..', 'matlab'));

    tests = {
        'test_basic_functionality'
        'test_week2_functions'
        'test_week3_functions'
        'test_week4_functions'
        'test_generate_solidworks_macro'
        'test_generate_onshape_3d_sketch'
        'test_resolve_suspension_geometry_source'
        'test_frame_design_v2'
        };

    failures = {};
    for i = 1:numel(tests)
        name = tests{i};
        fprintf('\n########## %s ##########\n', name);
        try
            run(fullfile(tests_dir, [name '.m']));
        catch ME
            failures{end+1} = sprintf('%s: %s', name, ME.message); %#ok<AGROW>
        end
    end

    fprintf('\n=== FRAME TEST RUNNER SUMMARY ===\n');
    if isempty(failures)
        fprintf('ALL %d TEST FILES PASSED\n', numel(tests));
    else
        fprintf('%d TEST FILES FAILED:\n', numel(failures));
        fprintf('  %s\n', failures{:});
        error('run_all_tests:Failed', '%d test files failed', numel(failures));
    end
end
