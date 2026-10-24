function exportNormalizationParams(norm_params, output_path)
% EXPORTNORMALIZATIONPARAMS Export normalization parameters to text format
%
% This function exports normalization parameters (mean and std) to a
% text format that can be easily loaded in C++.
%
% Inputs:
%   norm_params - Struct with input_mean, input_std, output_mean, output_std
%   output_path - Path for output text file
%
% Output file format:
%   INPUT_MEAN:
%   val1 val2 val3 val4 val5
%   INPUT_STD:
%   val1 val2 val3 val4 val5
%   OUTPUT_MEAN:
%   val1 val2 val3 val4
%   OUTPUT_STD:
%   val1 val2 val3 val4
%
% Example:
%   exportNormalizationParams(norm_params, 'norm_params.txt');

fprintf('=== Exporting Normalization Parameters ===\n');

% Open output file
fid = fopen(output_path, 'w');
if fid == -1
    error('Cannot open file for writing: %s', output_path);
end

% Write header
fprintf(fid, '# Normalization Parameters\n');
fprintf(fid, '# Generated: %s\n', datestr(now));
fprintf(fid, '# Format: Each line contains space-separated values\n\n');

% Write input normalization
fprintf(fid, 'INPUT_MEAN:\n');
fprintf(fid, '%.15e %.15e %.15e %.15e %.15e\n', ...
    norm_params.input_mean(1), norm_params.input_mean(2), ...
    norm_params.input_mean(3), norm_params.input_mean(4), ...
    norm_params.input_mean(5));
fprintf(fid, '\n');

fprintf(fid, 'INPUT_STD:\n');
fprintf(fid, '%.15e %.15e %.15e %.15e %.15e\n', ...
    norm_params.input_std(1), norm_params.input_std(2), ...
    norm_params.input_std(3), norm_params.input_std(4), ...
    norm_params.input_std(5));
fprintf(fid, '\n');

% Write output normalization
fprintf(fid, 'OUTPUT_MEAN:\n');
fprintf(fid, '%.15e %.15e %.15e %.15e\n', ...
    norm_params.output_mean(1), norm_params.output_mean(2), ...
    norm_params.output_mean(3), norm_params.output_mean(4));
fprintf(fid, '\n');

fprintf(fid, 'OUTPUT_STD:\n');
fprintf(fid, '%.15e %.15e %.15e %.15e\n', ...
    norm_params.output_std(1), norm_params.output_std(2), ...
    norm_params.output_std(3), norm_params.output_std(4));

fclose(fid);

% Report
fprintf('✓ Normalization parameters exported successfully\n');
fprintf('  File: %s\n', output_path);
fprintf('========================================\n\n');
end
