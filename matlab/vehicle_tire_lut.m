function lut_path = vehicle_tire_lut(config)
% VEHICLE_TIRE_LUT Baked 5D table of the PINN tire network for tire.pinn_lut_file.
%
% Network inference is ~97% of laptime_sim CPU; the LUT replaces it with
% multilinear interpolation (~35x faster per lap, lap time within ~0.1 s of
% the network). Baked once per tire export (~70 s, ~180 MB) next to the
% export and re-baked when the export or the baker binary is newer.
% laptime_sim re-checks grid nodes against the network on load, so a stale
% table fails loudly instead of changing the tire.
%
% Call once per campaign before the parallel pool starts evaluating, so
% workers never bake concurrently. Returns '' when config.use_tire_lut is false.

lut_path = '';
if ~config.use_tire_lut
    return;
end
model_dir = fullfile(config.project_root, 'output', 'tire_models', config.tire_model_name);
manifest = fullfile(model_dir, 'manifest.json');
baker = fullfile(fileparts(config.binary_path), 'bake_tire_lut');
if ~isfile(baker)
    error('vehicle_tire_lut:missingBinary', 'bake_tire_lut not found next to %s.', ...
        config.binary_path);
end

lut_path = fullfile(model_dir, 'pinn_raw.lut');
inputs = [dir(manifest); dir(fullfile(model_dir, 'pinn_runtime.json')); dir(baker)];
lut_info = dir(lut_path);
if ~isempty(lut_info) && lut_info.datenum > max([inputs.datenum])
    return;
end

staging = [lut_path, '.', char(java.util.UUID.randomUUID()), '.writing'];
fprintf('Baking tire LUT for %s (~70 s)...\n', config.tire_model_name);
[status, output] = system(sprintf('%s %s %s', ...
    shell_quote(baker), shell_quote(manifest), shell_quote(staging)));
if status ~= 0 || ~isfile(staging)
    error('vehicle_tire_lut:bakeFailed', 'bake_tire_lut failed (%d): %s', status, strtrim(output));
end
fprintf('%s\n', strtrim(output));
movefile(staging, lut_path, 'f');
end

function quoted = shell_quote(value)
value = char(value);
if contains(value, '''')
    error('vehicle_tire_lut:unsupportedPath', 'Paths containing a single quote are not supported.');
end
quoted = ['''', value, ''''];
end
