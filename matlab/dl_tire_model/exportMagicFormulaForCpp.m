%% exportMagicFormulaForCpp.m
% Magic Formula パラメータを C++ 用テキストファイルにエクスポート
%
% 入力: magic_formula_fitting.m で生成した .mat ファイル
% 出力: cpp_export/magic_formula_params.txt
%
% 使用法:
%   exportMagicFormulaForCpp()                        % デフォルト
%   exportMagicFormulaForCpp('output/magic_formula_Hoosier_18p0X6p0-10_R20.mat')

function exportMagicFormulaForCpp(mat_path)
    if nargin < 1
        mat_path = 'output/magic_formula_Hoosier_18p0X6p0-10_R20.mat';
    end

    fprintf('Loading: %s\n', mat_path);
    data = load(mat_path);

    % 出力ディレクトリ
    out_dir = '../../cpp_export';
    if ~exist(out_dir, 'dir')
        mkdir(out_dir);
    end
    out_path = fullfile(out_dir, 'magic_formula_params.txt');

    fid = fopen(out_path, 'w');
    if fid == -1
        error('Cannot open output file: %s', out_path);
    end

    fprintf(fid, '# Magic Formula MF 5.2 Parameters\n');
    fprintf(fid, '# Exported from: %s\n', mat_path);
    fprintf(fid, '# Date: %s\n', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
    fprintf(fid, 'Fz0: %.15f\n', data.mf_params.Fz0);
    fprintf(fid, 'R0: %.15f\n', data.mf_params.R0);

    % Stage 1: Fy0
    writeSection(fid, 'Fy0', data.fy_names, data.fy_params);

    % Stage 2: Fx0
    writeSection(fid, 'Fx0', data.fx_names, data.fx_params);

    % Stage 3: Gxa
    writeSection(fid, 'Gxa', data.gxa_names, data.gxa_params);

    % Stage 3: Gyk
    writeSection(fid, 'Gyk', data.gyk_names, data.gyk_params);

    % Stage 4: Mz
    writeSection(fid, 'Mz', data.mz_names, data.mz_params);

    fclose(fid);
    fprintf('Exported to: %s\n', out_path);
    fprintf('Done.\n');
end

function writeSection(fid, section_name, names, values)
    fprintf(fid, '[%s]\n', section_name);
    for i = 1:length(names)
        fprintf(fid, '%s: %.15f\n', names{i}, values(i));
    end
end
