function result = run_frame_optimization(varargin)
% RUN_FRAME_OPTIMIZATION Non-interactive-by-default wrapper for the canonical frame API.
%
% Interactive usage (keeps the historical prompts):
%   run_frame_optimization
%
% Batch usage:
%   run_frame_optimization('Interactive', false, ...
%       'VehicleParamsFile', '/path/to/vehicle.json', ...
%       'MassTargetKg', 30, 'MaxGenerations', 5, 'UseParallel', false)
%
% All Name-Value options are forwarded to run_canonical_frame_optimization.
% In non-interactive mode no prompts are shown and the viewer is not opened.

    script_dir = fileparts(mfilename('fullpath'));
    matlab_dir = fullfile(script_dir, '..', 'matlab');
    project_root = fullfile(script_dir, '..', '..');
    addpath(matlab_dir);
    addpath(fullfile(project_root, 'matlab'));

    p = inputParser;
    addParameter(p, 'VehicleParamsFile', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'MassTargetKg', [], @(x) isempty(x) || (isscalar(x) && isnumeric(x) && isfinite(x) && x > 0));
    addParameter(p, 'GenerateSolidWorksMacro', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'GenerateOnshape3DSketch', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'Interactive', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'WaitForFigureClose', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
    addParameter(p, 'ShowViewer', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
    addParameter(p, 'SaveFigure', [], @(x) isempty(x) || islogical(x) || isnumeric(x));
    addParameter(p, 'DisplayProgress', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'PopulationSize', 60, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'MaxGenerations', 40, @(x) isnumeric(x) && isscalar(x) && x >= 1);
    addParameter(p, 'UseParallel', true, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'UseFrameDesignV2', false, @(x) islogical(x) || isnumeric(x));
    addParameter(p, 'OptimizerMode', 'mixed_ga', @(x) ischar(x) || isstring(x));
    addParameter(p, 'RandomSeed', [], @(x) isempty(x) || (isnumeric(x) && isscalar(x) && isfinite(x)));
    addParameter(p, 'LoadCaseEvalMode', 'full', @(x) ischar(x) || isstring(x));
    addParameter(p, 'SolverBackend', 'auto', @(x) ischar(x) || isstring(x));
    addParameter(p, 'FemServerLifecycle', 'managed_once', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    opts = p.Results;
    using_defaults = p.UsingDefaults;

    interactive = logical(opts.Interactive);
    if interactive && strcmp(opts.VehicleParamsFile, '') ...
            && evalin('base', 'exist(''vehicle_params_file'', ''var'')')
        opts.VehicleParamsFile = evalin('base', 'vehicle_params_file');
    end

    if interactive
        if isempty(opts.MassTargetKg)
            fprintf('目標フレーム重量を入力してください (Enter でスキップ = ペナルティなし)\n');
            opts.MassTargetKg = promptOptionalPositiveScalar('目標重量 [kg]: ');
        end
        if any(strcmp(using_defaults, 'GenerateSolidWorksMacro'))
            fprintf('\nSolidWorks VBAマクロを生成しますか？\n');
            fprintf('  [1] はい (生成する)\n');
            fprintf('  [2] いいえ (スキップ)\n');
            opts.GenerateSolidWorksMacro = strcmp(strtrim(input('番号を選択 >> ', 's')), '1');
        end
        if any(strcmp(using_defaults, 'GenerateOnshape3DSketch'))
            fprintf('\nOnshape 3Dスケッチ用 FeatureScript を生成しますか？\n');
            fprintf('  [1] はい (生成する)\n');
            fprintf('  [2] いいえ (スキップ)\n');
            opts.GenerateOnshape3DSketch = strcmp(strtrim(input('番号を選択 >> ', 's')), '1');
        end
    end

    wait_for_close = opts.WaitForFigureClose;
    if isempty(wait_for_close)
        wait_for_close = interactive;
    end
    show_viewer = opts.ShowViewer;
    if isempty(show_viewer)
        show_viewer = interactive;
    end
    save_figure = opts.SaveFigure;
    if isempty(save_figure)
        save_figure = interactive;
    end

    result = run_canonical_frame_optimization( ...
        'VehicleParamsFile', char(string(opts.VehicleParamsFile)), ...
        'MassTargetKg', opts.MassTargetKg, ...
        'GenerateSolidWorksMacro', logical(opts.GenerateSolidWorksMacro), ...
        'GenerateOnshape3DSketch', logical(opts.GenerateOnshape3DSketch), ...
        'WaitForFigureClose', logical(wait_for_close), ...
        'ShowViewer', logical(show_viewer), ...
        'SaveFigure', logical(save_figure), ...
        'DisplayProgress', logical(opts.DisplayProgress), ...
        'PopulationSize', opts.PopulationSize, ...
        'MaxGenerations', opts.MaxGenerations, ...
        'UseParallel', logical(opts.UseParallel), ...
        'UseFrameDesignV2', logical(opts.UseFrameDesignV2), ...
        'OptimizerMode', char(string(opts.OptimizerMode)), ...
        'RandomSeed', opts.RandomSeed, ...
        'LoadCaseEvalMode', char(string(opts.LoadCaseEvalMode)), ...
        'SolverBackend', char(string(opts.SolverBackend)), ...
        'FemServerLifecycle', char(string(opts.FemServerLifecycle)));

    assignin('base', 'frame_optimization_output_path', result.output_path);
end

function value = promptOptionalPositiveScalar(prompt_text)
value = [];
raw_value = input(prompt_text, 's');
if isempty(raw_value)
    return;
end

value = str2double(raw_value);
if isnan(value) || value <= 0
    warning('無効な入力。目標重量ペナルティを無効にします。');
    value = [];
end
end
