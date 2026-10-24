fprintf('=== Suspension artifact resolver fail-fast test ===\n');

test_dir = fileparts(mfilename('fullpath'));
project_root = fullfile(test_dir, '..', '..');
addpath(fullfile(project_root, 'matlab'));
addpath(fullfile(project_root, 'frame_optimize', 'matlab'));

fixture_file = fullfile(test_dir, 'fixtures', 'canonical_suspension_fixture.json');
assert(isfile(fixture_file), 'Missing local suspension fixture: %s', fixture_file);

resolved_path = resolve_suspension_geometry_source(project_root, fixture_file, 'ErrorIfMissing', true);
assert(strcmp(resolved_path, fixture_file), ...
    'Expected preferred fixture to be returned when it already contains required hardpoints.');

empty_root = tempname;
mkdir(empty_root);
cleanup_obj = onCleanup(@() rmdir(empty_root, 's')); %#ok<NASGU>

did_fail_fast = false;
try
    resolve_suspension_geometry_source(empty_root, '', 'ErrorIfMissing', true);
catch ME
    did_fail_fast = true;
    assert(strcmp(ME.identifier, 'resolve_suspension_geometry_source:MissingGeometry'), ...
        'Expected MissingGeometry identifier, got: %s', ME.identifier);
end

assert(did_fail_fast, 'Expected resolver to raise when ErrorIfMissing=true and no artifact exists.');
fprintf('Resolver fail-fast OK\n');
