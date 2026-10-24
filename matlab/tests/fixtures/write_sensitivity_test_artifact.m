function artifact = write_sensitivity_test_artifact(artifact_path, base_params)
if nargin < 2 || isempty(base_params)
    base_params = sensitivity_test_base_params();
end

artifact = struct();
artifact.schema_version = 'vehicle_artifact/v1';
artifact.generated_by = struct( ...
    'tool', 'matlab/tests', ...
    'physics_core', 'sim_overall/cpp', ...
    'source_of_truth', 'matlab/tests/fixtures');
artifact.optimization = struct('stages', {{}});
artifact.performance = struct('events', struct());
artifact.params = base_params;

parent_dir = fileparts(artifact_path);
if ~exist(parent_dir, 'dir')
    mkdir(parent_dir);
end

fid = fopen(artifact_path, 'w');
if fid == -1
    error('write_sensitivity_test_artifact:openFailed', ...
        'Failed to open %s for writing.', artifact_path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s', jsonencode(artifact));
end
