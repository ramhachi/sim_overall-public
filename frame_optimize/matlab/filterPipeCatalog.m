function filtered_indices = filterPipeCatalog(catalog, options)
% FILTERPIPECATALOG Filter pipe catalog by constraints
%
% Syntax:
%   indices = filterPipeCatalog(catalog, options)
%
% Inputs:
%   catalog - Pipe catalog array from loadPipeCatalog()
%   options - Struct with optional filter fields:
%             min_size_class - Minimum size class ('A', 'B', 'C', 'D', 'None')
%             material_id    - Material ID (e.g., 'AISI4130')
%             shape          - Shape ('round' or 'square')
%             available_only - true/false (default: true)
%             max_cost       - Maximum cost per meter [JPY]
%
% Output:
%   filtered_indices - Array of indices into catalog that meet all criteria
%
% Example:
%   % Find all available Size B+ steel round pipes
%   opts = struct('min_size_class', 'B', ...
%                 'material_id', 'AISI4130', ...
%                 'shape', 'round', ...
%                 'available_only', true);
%   idx = filterPipeCatalog(catalog, opts);
%   valid_pipes = catalog(idx);

    narginchk(2, 2);
    validateattributes(catalog, {'struct'}, {'vector'});
    validateattributes(options, {'struct'}, {'scalar'});

    n_pipes = length(catalog);
    valid = true(n_pipes, 1);

    % Filter by availability
    if isfield(options, 'available_only') && options.available_only
        for i = 1:n_pipes
            if ~catalog(i).available
                valid(i) = false;
            end
        end
    end

    % Filter by material
    if isfield(options, 'material_id')
        mat_id = options.material_id;
        for i = 1:n_pipes
            if ~strcmp(catalog(i).material_id, mat_id)
                valid(i) = false;
            end
        end
    end

    % Filter by shape
    if isfield(options, 'shape')
        shape = lower(options.shape);
        for i = 1:n_pipes
            if ~strcmp(lower(catalog(i).shape), shape)
                valid(i) = false;
            end
        end
    end

    % Filter by size class
    if isfield(options, 'min_size_class')
        min_class = upper(options.min_size_class);

        % A/B/C form a strength hierarchy. Size D is a separate bent-upper-SI
        % specification and is intentionally excluded from A/B/C filters.
        size_map = containers.Map({'None', 'C', 'B', 'A'}, {0, 1, 2, 3});
        valid_classes = [size_map.keys(), {'D'}];

        if ~any(strcmp(valid_classes, min_class))
            error('filterPipeCatalog:InvalidSizeClass', ...
                  'Invalid size class: %s (must be A/B/C/D/None)', min_class);
        end

        for i = 1:n_pipes
            pipe_class = upper(catalog(i).size_class);
            if strcmp(pipe_class, 'D')
                meets = strcmp(min_class, 'D') || strcmp(min_class, 'None');
            elseif isKey(size_map, pipe_class)
                if strcmp(min_class, 'D')
                    meets = false;
                else
                    meets = strcmp(min_class, 'None') || ...
                        (size_map(pipe_class) >= size_map(min_class));
                end
            else
                meets = false;
            end
            if ~meets
                valid(i) = false;
            end
        end
    end

    % Filter by max cost
    if isfield(options, 'max_cost')
        max_cost = options.max_cost;
        for i = 1:n_pipes
            if isfield(catalog(i), 'cost_per_meter_jpy')
                if catalog(i).cost_per_meter_jpy > max_cost
                    valid(i) = false;
                end
            end
        end
    end

    % Return indices
    filtered_indices = find(valid);

    if isempty(filtered_indices)
        warning('filterPipeCatalog:NoMatch', ...
                'No pipes match the specified criteria');
    end
end
