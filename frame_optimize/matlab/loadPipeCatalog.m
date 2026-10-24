function catalog = loadPipeCatalog(json_path, materials)
% LOADPIPECATALOG Load pipe catalog from JSON and compute section properties
%
% Syntax:
%   catalog = loadPipeCatalog(json_path, materials)
%
% Inputs:
%   json_path - Path to pipe_catalog.json file
%   materials - Materials struct from loadMaterials()
%
% Output:
%   catalog - Array of structs, each with pipe properties + computed A, Iy, J, size_class
%
% Example:
%   mats = loadMaterials('data/materials.json');
%   cat = loadPipeCatalog('data/pipe_catalog.json', mats);
%   fprintf('Pipe 1: %s, Size %s, I=%.1f mm4\n', cat(1).id, cat(1).size_class, cat(1).Iy_mm4);

    narginchk(2, 2);
    validateattributes(json_path, {'char', 'string'}, {'scalartext'});
    validateattributes(materials, {'struct'}, {});

    % Read JSON file
    if ~isfile(json_path)
        error('loadPipeCatalog:FileNotFound', ...
              'Catalog file not found: %s', json_path);
    end

    try
        raw = jsondecode(fileread(json_path));
    catch ME
        error('loadPipeCatalog:InvalidJSON', ...
              'Failed to parse JSON: %s', ME.message);
    end

    % Validate structure
    if ~isfield(raw, 'pipes')
        error('loadPipeCatalog:InvalidStructure', ...
              'JSON must contain "pipes" field');
    end

    % Process each pipe
    % Handle both struct array and cell array (MATLAB version compatibility)
    pipe_array = raw.pipes;
    if iscell(pipe_array)
        n_pipes = length(pipe_array);
    else
        n_pipes = length(pipe_array);
    end

    catalog = repmat(struct(), n_pipes, 1);

    for i = 1:n_pipes
        % Extract pipe (handle both cell and struct array)
        if iscell(pipe_array)
            pipe = pipe_array{i};
        else
            pipe = pipe_array(i);
        end

        % Validate required fields
        required = {'id', 'shape', 'material_id', 'D_outer_mm', 't_mm'};
        for j = 1:length(required)
            if ~isfield(pipe, required{j})
                error('loadPipeCatalog:MissingField', ...
                      'Pipe %d missing field: %s', i, required{j});
            end
        end

        % Copy basic fields
        catalog(i).id = pipe.id;
        catalog(i).shape = pipe.shape;
        catalog(i).material_id = pipe.material_id;
        catalog(i).D_outer_mm = pipe.D_outer_mm;
        catalog(i).t_mm = pipe.t_mm;
        catalog(i).available = pipe.available;

        % Optional fields
        if isfield(pipe, 'cost_per_meter_jpy')
            catalog(i).cost_per_meter_jpy = pipe.cost_per_meter_jpy;
        end
        if isfield(pipe, 'lead_time_days')
            catalog(i).lead_time_days = pipe.lead_time_days;
        end

        % Compute section properties
        props = computeTubeSectionProperties(pipe.D_outer_mm, pipe.t_mm, pipe.shape);
        catalog(i).A_mm2 = props.A_mm2;
        catalog(i).Iy_mm4 = props.Iy_mm4;
        catalog(i).Iz_mm4 = props.Iz_mm4;
        catalog(i).J_mm4 = props.J_mm4;

        % Determine size class
        catalog(i).size_class = determineSizeClass(props.Iy_mm4, props.A_mm2, ...
                                                   pipe.D_outer_mm, pipe.t_mm);

        % Check material restrictions
        mat_id = matlab.lang.makeValidName(pipe.material_id);
        if ~isfield(materials, mat_id)
            warning('loadPipeCatalog:UnknownMaterial', ...
                    'Pipe %d references unknown material: %s', i, pipe.material_id);
        end
    end

    % Sort by (material, shape, D, t) so a +/-1 catalog mutation changes one
    % attribute instead of jumping to an unrelated tube (#53).
    if n_pipes > 1
        sort_table = table( ...
            {catalog.material_id}', {catalog.shape}', [catalog.D_outer_mm]', [catalog.t_mm]', ...
            'VariableNames', {'material_id', 'shape', 'D_outer_mm', 't_mm'});
        [~, order] = sortrows(sort_table);
        catalog = catalog(order);
    end

    fprintf('Loaded %d pipes from %s\n', n_pipes, json_path);
end
