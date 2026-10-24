function report = assert_valid_vehicle_parameter_artifact(artifact_or_path)
% ASSERT_VALID_VEHICLE_PARAMETER_ARTIFACT Require a canonical vehicle artifact.

report = validate_vehicle_parameter_artifact(artifact_or_path);
if report.is_valid
    return;
end

messages = strings(1, numel(report.issues));
for i = 1:numel(report.issues)
    messages(i) = sprintf('%s: %s', report.issues(i).code, report.issues(i).message);
end
error('assert_valid_vehicle_parameter_artifact:invalidArtifact', ...
    'Vehicle parameter artifact is invalid:\n- %s', strjoin(messages, '\n- '));
end
