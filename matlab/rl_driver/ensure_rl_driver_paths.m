function ensure_rl_driver_paths()
% ENSURE_RL_DRIVER_PATHS Add rl_driver and matlab roots to the MATLAB path.

thisDir = fileparts(mfilename('fullpath'));
addpath_if_missing(thisDir);
addpath_if_missing(fileparts(thisDir));
end

function addpath_if_missing(dirPath)
dirPath = char(dirPath);
if isempty(dirPath) || ~isfolder(dirPath)
    return;
end

currentPath = strsplit(path, pathsep);
if ~any(strcmp(currentPath, dirPath))
    addpath(dirPath);
end
end
