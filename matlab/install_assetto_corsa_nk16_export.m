function result = install_assetto_corsa_nk16_export(exportDir, targetCarDir)
% INSTALL_ASSETTO_CORSA_NK16_EXPORT Copy a generated NK16 overlay into an extracted car folder.

if nargin < 1 || isempty(exportDir)
    error('Export directory is required.');
end
if nargin < 2 || isempty(targetCarDir)
    error('Target NK16_ICV directory is required.');
end

exportDir = char(exportDir);
targetCarDir = char(targetCarDir);
sourceDataDir = fullfile(exportDir, 'data');
sourceUiDir = fullfile(exportDir, 'ui');
targetDataDir = fullfile(targetCarDir, 'data');
targetUiDir = fullfile(targetCarDir, 'ui');

if ~isfolder(exportDir)
    error('Export directory not found: %s', exportDir);
end
if ~isfolder(sourceDataDir) || ~isfolder(sourceUiDir)
    error('Export directory does not contain data/ and ui/: %s', exportDir);
end
if ~isfolder(targetCarDir)
    error('Target NK16_ICV directory not found: %s', targetCarDir);
end
if ~looksLikeAssettoCorsaCarDir(targetCarDir)
    error(['Target directory does not look like an Assetto Corsa car folder: %s\n' ...
        'Expected at least one of: data/, data.acd, ui/, ui/ui_car.json.'], targetCarDir);
end
if isfile(fullfile(targetCarDir, 'data.acd')) && ~isfolder(targetDataDir)
    error(['Target car still uses data.acd and has no extracted data/ folder: %s\n' ...
        'Extract the NK16 physics data first, then rerun the installer.'], targetCarDir);
end

if ~isfolder(targetDataDir)
    mkdir(targetDataDir);
end
if ~isfolder(targetUiDir)
    mkdir(targetUiDir);
end

copyOverlayDirectory(sourceDataDir, targetDataDir);
copyOverlayDirectory(sourceUiDir, targetUiDir);

metadataPath = fullfile(exportDir, 'sim_overall_export_metadata.json');
if isfile(metadataPath)
    copyfile(metadataPath, fullfile(targetCarDir, 'sim_overall_export_metadata.json'), 'f');
end

result = struct();
result.export_dir = exportDir;
result.target_car_dir = targetCarDir;
result.target_data_dir = targetDataDir;
result.target_ui_dir = targetUiDir;
end

function tf = looksLikeAssettoCorsaCarDir(targetCarDir)
tf = isfolder(fullfile(targetCarDir, 'data')) ...
    || isfile(fullfile(targetCarDir, 'data.acd')) ...
    || isfolder(fullfile(targetCarDir, 'ui')) ...
    || isfile(fullfile(targetCarDir, 'ui', 'ui_car.json'));
end

function copyOverlayDirectory(sourceDir, targetDir)
entries = dir(sourceDir);
for i = 1:numel(entries)
    name = entries(i).name;
    if strcmp(name, '.') || strcmp(name, '..')
        continue;
    end

    sourcePath = fullfile(sourceDir, name);
    targetPath = fullfile(targetDir, name);
    if entries(i).isdir
        if ~isfolder(targetPath)
            mkdir(targetPath);
        end
        copyOverlayDirectory(sourcePath, targetPath);
    else
        copyfile(sourcePath, targetPath, 'f');
    end
end
end
