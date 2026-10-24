function [selectedPath, ok, selectedCandidate] = select_vehicle_json_candidate(varargin)
% SELECT_VEHICLE_JSON_CANDIDATE Interactively choose a vehicle JSON candidate.

p = inputParser;
addParameter(p, 'ProjectRoot', '');
addParameter(p, 'Prompt', '利用する車両諸元JSONを選択してください:');
addParameter(p, 'RequireExportReady', false);
addParameter(p, 'RequireSuspensionHardpoints', false);
addParameter(p, 'RequiredHardpointCorners', {});
parse(p, varargin{:});
opts = p.Results;

selectedPath = '';
ok = false;
selectedCandidate = struct();

projectRoot = opts.ProjectRoot;
if isempty(projectRoot)
    thisDir = fileparts(mfilename('fullpath'));
    projectRoot = fileparts(thisDir);
end

candidates = find_vehicle_json_candidates(projectRoot, ...
    'RequireExportReady', opts.RequireExportReady, ...
    'RequireSuspensionHardpoints', opts.RequireSuspensionHardpoints, ...
    'RequiredHardpointCorners', opts.RequiredHardpointCorners);
if isempty(candidates)
    fprintf('⚠️  利用可能な車両諸元JSONが見つかりません。\n');
    return;
end

fprintf('\n%s\n', opts.Prompt);
for i = 1:numel(candidates)
    optimizedTag = '';
    if candidates(i).is_optimized
        optimizedTag = ' [optimized]';
    end
    fprintf('  [%d] %s%s\n', i, candidates(i).label, optimizedTag);
    fprintf('      %s | %s\n', candidates(i).modified_at, candidates(i).summary);
end
fprintf('  [0] キャンセル\n');

fprintf('番号を選択 >> ');
drawnow;
choice = input('', 's');
if strcmp(choice, '0')
    return;
end

idx = str2double(choice);
if isnan(idx) || idx < 1 || idx > numel(candidates)
    fprintf('⚠️  無効な番号です。\n');
    return;
end

selectedCandidate = candidates(idx);
selectedPath = selectedCandidate.path;
ok = true;
end
