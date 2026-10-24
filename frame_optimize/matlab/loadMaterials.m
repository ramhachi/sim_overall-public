function materials = loadMaterials(json_path)
% LOADMATERIALS Load material properties from JSON file
%
% Syntax:
%   materials = loadMaterials(json_path)
%
% Input:
%   json_path - Path to materials.json file (string or char)
%
% Output:
%   materials - Struct with material_id as fieldnames
%
% Example:
%   mats = loadMaterials('data/materials.json');
%   E = mats.AISI4130.E_GPa;

    narginchk(1, 1);
    validateattributes(json_path, {'char', 'string'}, {'scalartext'});

    % Read JSON file
    if ~isfile(json_path)
        error('loadMaterials:FileNotFound', ...
              'Materials file not found: %s', json_path);
    end

    try
        raw = jsondecode(fileread(json_path));
    catch ME
        error('loadMaterials:InvalidJSON', ...
              'Failed to parse JSON: %s', ME.message);
    end

    % Validate structure
    if ~isfield(raw, 'materials')
        error('loadMaterials:InvalidStructure', ...
              'JSON must contain "materials" field');
    end

    % Convert to struct with material_id as fieldnames
    materials = struct();

    % Handle both struct array and cell array (MATLAB version compatibility)
    mat_array = raw.materials;
    if iscell(mat_array)
        n_mats = length(mat_array);
    else
        n_mats = length(mat_array);
    end

    for i = 1:n_mats
        % Extract material (handle both cell and struct array)
        if iscell(mat_array)
            mat = mat_array{i};
        else
            mat = mat_array(i);
        end

        % Validate required fields
        required = {'id', 'E_GPa', 'G_GPa', 'rho_kg_m3', ...
                   'Sy_welded_MPa', 'Su_welded_MPa'};
        for j = 1:length(required)
            if ~isfield(mat, required{j})
                error('loadMaterials:MissingField', ...
                      'Material %d missing field: %s', i, required{j});
            end
        end

        % Add to struct (use id as fieldname)
        mat_id = matlab.lang.makeValidName(mat.id);
        materials.(mat_id) = mat;
    end

    fprintf('Loaded %d materials from %s\n', length(fieldnames(materials)), json_path);
end
