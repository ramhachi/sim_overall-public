function test_generate_onshape_3d_sketch()
% TEST_GENERATE_ONSHAPE_3D_SKETCH Verify generate_onshape_3d_sketch.m

fprintf('=======================================================\n');
fprintf(' Test: generate_onshape_3d_sketch\n');
fprintf('=======================================================\n');

this_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(this_dir);
matlab_dir = fullfile(root_dir, 'matlab');
addpath(matlab_dir);

test_output_dir = fullfile(tempdir, 'test_onshape_sketch');
if isfolder(test_output_dir)
    rmdir(test_output_dir, 's');
end
mkdir(test_output_dir);
cleanup_output = onCleanup(@() cleanup_temp_output(test_output_dir)); %#ok<NASGU>

[nodes, elements_geom] = build_synthetic_frame();

opts = struct();
opts.output_dir = test_output_dir;
opts.filename = 'test_onshape_sketch.fs';
opts.feature_name = 'FSAE "Quoted" \ Sketch';

fprintf('\n[Test 1] generate_onshape_3d_sketch() runs without error...\n');
output_path = generate_onshape_3d_sketch(nodes, elements_geom, opts);
assert(isfile(output_path), 'FeatureScript file was not created.');
fprintf('  -> PASS (%s)\n', output_path);

fprintf('\n[Test 2] Output file is non-empty...\n');
info = dir(output_path);
assert(info.bytes > 500, 'Generated FeatureScript file is unexpectedly small.');
fprintf('  -> PASS (%d bytes)\n', info.bytes);

fprintf('\n[Test 3] FeatureScript structure validation...\n');
fs_text = fileread(output_path);
assert(contains(fs_text, 'defineFeature(function(context is Context, id is Id, definition is map)'), ...
    'Missing defineFeature wrapper.');
assert(contains(fs_text, '    }, {});'), 'Missing defineFeature closing syntax.');
assert(contains(fs_text, 'opPolyline(context, id + "segment1"'), ...
    'Missing first polyline operation.');
assert(~contains(fs_text, 'extrude'), 'Exporter must not create solids.');
assert(~contains(fs_text, 'boolean'), 'Exporter must not create cope cuts.');
assert(contains(fs_text, 'annotation { "Feature Type Name" : "FSAE \"Quoted\" \\ Sketch" }'), ...
    'Feature name escaping is incorrect.');
fprintf('  -> PASS\n');

fprintf('\n[Test 4] Coordinate transform spot check...\n');
expected_node1 = sprintf('vector(%.9f, %.9f, %.9f) * meter', ...
    -nodes(1, 1), nodes(1, 3), nodes(1, 2));
assert(contains(fs_text, expected_node1), 'Node 1 coordinate transform mismatch.');
fprintf('  -> PASS\n');

fprintf('\n[Test 5] Segment count matches unique connectivity...\n');
unique_segments = unique(sort(elements_geom.connectivity, 2), 'rows', 'stable');
segment_count = count(fs_text, 'opPolyline(context, id + "segment');
assert(segment_count == size(unique_segments, 1), ...
    sprintf('Expected %d segments but found %d.', size(unique_segments, 1), segment_count));
fprintf('  -> PASS (%d segments)\n', segment_count);

fprintf('\nAll tests passed: generate_onshape_3d_sketch\n');
end

function cleanup_temp_output(test_output_dir)
if isfolder(test_output_dir)
    rmdir(test_output_dir, 's');
end
end

function [nodes, elements_geom] = build_synthetic_frame()
nodes = zeros(12, 3);
nodes(1,:) = [0.0, -0.25, 0.05];
nodes(2,:) = [0.0,  0.25, 0.05];
nodes(3,:) = [0.0, -0.25, 0.90];
nodes(4,:) = [0.0,  0.25, 0.90];
nodes(5,:) = [0.8, -0.30, 0.05];
nodes(6,:) = [0.8,  0.30, 0.05];
nodes(7,:) = [0.8, -0.30, 1.10];
nodes(8,:) = [0.8,  0.30, 1.10];
nodes(9,:) = [1.3, -0.25, 0.30];
nodes(10,:) = [1.3, 0.25, 0.30];
nodes(11,:) = [1.6, -0.25, 0.30];
nodes(12,:) = [1.6, 0.25, 0.30];

connectivity = [
    1 2;
    1 3;
    2 4;
    3 4;
    3 7;
    4 8;
    5 6;
    5 7;
    6 8;
    7 8;
    7 9;
    8 10;
    9 11;
    10 12;
    11 12;
    1 5;
    2 6;
    3 5
    ];

elements_geom = struct();
elements_geom.connectivity = connectivity;
end
