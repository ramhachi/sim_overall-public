function output_path = regenerate_winged_flexible_attack_preset(output_path)
% REGENERATE_WINGED_FLEXIBLE_ATTACK_PRESET Write the canonical RL/PID preset.

if nargin < 1 || isempty(output_path)
    matlab_root = fileparts(mfilename('fullpath'));
    output_path = fullfile(matlab_root, 'parameter_presets', 'params_winged_flexible.json');
end

artifact = build_winged_flexible_attack_artifact();
fid = fopen(output_path, 'w');
if fid < 0
    error('regenerate_winged_flexible_attack_preset:openFailed', ...
        'Could not open %s for writing.', output_path);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, '%s\n', jsonencode(artifact, PrettyPrint=true));
end
