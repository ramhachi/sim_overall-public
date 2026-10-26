function ggv_path = vehicle_lateral_ggv(params_path, params, config)
% VEHICLE_LATERAL_GGV Cached quick lateral-only GGV for the LPV curvature cap.
%
% The LPV speed cap reads the vehicle's own 14DOF ay_max(v) (laptime_sim
% --lpv-ggv), so a design change only shows up in lap time if each candidate
% gets its own GGV. Generation takes ~10-35 s; the cache key covers every
% params group that can change lateral capability (powertrain, brake and DRS
% cannot) plus the deployed tire export and the generator binary, so parallel
% or repeated evaluations of the same chassis reuse one file.

key_params = rmfield_if(params, {'powertrain', 'brake', 'drs'});
binary = fullfile(fileparts(config.binary_path), 'ggv_generator');
binary_info = dir(binary);
if isempty(binary_info)
    error('vehicle_lateral_ggv:missingBinary', 'ggv_generator not found next to %s.', ...
        config.binary_path);
end
manifest = fullfile(config.project_root, params.tire.model_manifest);
tire_id = '';
if isfile(manifest)
    tire_id = fileread(manifest);
end
key = sha256([jsonencode(key_params), tire_id, sprintf('%.6f', binary_info.datenum)]);

[~, ~] = mkdir(config.ggv_cache_dir);  % parallel workers may race here
ggv_path = fullfile(config.ggv_cache_dir, [key(1:16), '.json']);
if isfile(ggv_path)
    return;
end

% Unique staging name: parfor workers may build the same key concurrently;
% the last atomic move wins and both results are identical.
staging = [ggv_path, '.', char(java.util.UUID.randomUUID()), '.writing'];
command = sprintf('cd %s && %s --params %s --out %s --quick --lateral-only', ...
    shell_quote(config.project_root), shell_quote(binary), ...
    shell_quote(params_path), shell_quote(staging));
[status, output] = system(command);
if status ~= 0 || ~isfile(staging)
    error('vehicle_lateral_ggv:generationFailed', 'ggv_generator failed (%d): %s', ...
        status, strtrim(output));
end
movefile(staging, ggv_path, 'f');
end

function s = rmfield_if(s, names)
s = rmfield(s, names(isfield(s, names)));
end

function hex = sha256(text)
md = java.security.MessageDigest.getInstance('SHA-256');
bytes = typecast(md.digest(unicode2native(text, 'UTF-8')), 'uint8');
hex = lower(reshape(dec2hex(bytes, 2).', 1, []));
end

function quoted = shell_quote(value)
value = char(value);
if contains(value, '''')
    error('vehicle_lateral_ggv:unsupportedPath', 'Paths containing a single quote are not supported.');
end
quoted = ['''', value, ''''];
end
