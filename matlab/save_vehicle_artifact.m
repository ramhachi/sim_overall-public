function save_vehicle_artifact(output_path, artifact)
% save_vehicle_artifact - unified vehicle artifact を JSON に保存する
%
% Writes atomically via a sibling *.writing staging file so that a crash or
% error during serialisation cannot leave the canonical artifact in a partial
% / corrupt state.  The staging file is always cleaned up on failure.

json_text = jsonencode(artifact, 'PrettyPrint', true);

[dir_part, base_part, ext_part] = fileparts(output_path);
if isempty(dir_part)
    dir_part = '.';
end
staging_path = fullfile(dir_part, [base_part, ext_part, '.writing']);

fid = fopen(staging_path, 'w');
if fid == -1
    error('save_vehicle_artifact:openFailed', ...
        'Cannot open staging file for writing: %s', staging_path);
end

try
    fwrite(fid, json_text, 'char');
    fclose(fid);
catch write_err
    fclose(fid);
    if exist(staging_path, 'file')
        delete(staging_path);
    end
    rethrow(write_err);
end

try
    movefile(staging_path, output_path, 'f');
catch move_err
    if exist(staging_path, 'file')
        delete(staging_path);
    end
    error('save_vehicle_artifact:atomicMoveFailed', ...
        'Failed to finalise artifact at ''%s'': %s', output_path, move_err.message);
end

end
