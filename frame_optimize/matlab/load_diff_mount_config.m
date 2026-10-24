function diff_config = load_diff_mount_config(params, geom)
% LOAD_DIFF_MOUNT_CONFIG Extract differential mount data from artifact/config.
%
% Accepted locations:
%   params.frame.diff_mount_points / params.frame.diff_envelope
%   params.powertrain.diff_mount_points / params.powertrain.diff_envelope
%   geom.diff_mount_points / geom.diff_envelope

    if nargin < 1 || isempty(params)
        params = struct();
    end
    if nargin < 2 || isempty(geom)
        geom = struct();
    end

    diff_config = struct();
    diff_config.diff_mount_points = [];
    diff_config.diff_envelope = struct();
    diff_config.source = '';

    candidates = {};
    if isfield(params, 'frame')
        candidates{end + 1} = struct('source', 'params.frame', 'data', params.frame); %#ok<AGROW>
    end
    if isfield(params, 'powertrain')
        candidates{end + 1} = struct('source', 'params.powertrain', 'data', params.powertrain); %#ok<AGROW>
    end
    candidates{end + 1} = struct('source', 'geom', 'data', geom);

    for i = 1:numel(candidates)
        data = candidates{i}.data;
        points = readField(data, {'diff_mount_points', 'differential_mount_points'});
        envelope = readField(data, {'diff_envelope', 'differential_envelope'});
        if ~isempty(points)
            diff_config.diff_mount_points = normalizePoints(points);
            diff_config.diff_envelope = normalizeEnvelope(envelope);
            diff_config.source = candidates{i}.source;
            return;
        end
    end
end

function value = readField(s, names)
    value = [];
    if ~isstruct(s)
        return;
    end
    for i = 1:numel(names)
        if isfield(s, names{i}) && ~isempty(s.(names{i}))
            value = s.(names{i});
            return;
        end
    end
end

function points = normalizePoints(value)
    if isnumeric(value)
        points = value;
    elseif iscell(value)
        points = cell2mat(value);
    else
        error('load_diff_mount_config:invalidPoints', ...
            'diff_mount_points must be an Nx3 numeric array.');
    end
    if size(points, 2) ~= 3 && size(points, 1) == 3
        points = points';
    end
    if size(points, 2) ~= 3 || size(points, 1) < 2
        error('load_diff_mount_config:invalidPoints', ...
            'diff_mount_points must contain at least two [x,y,z] points.');
    end
end

function envelope = normalizeEnvelope(value)
    envelope = struct();
    if isempty(value)
        return;
    end
    if ~isstruct(value)
        error('load_diff_mount_config:invalidEnvelope', ...
            'diff_envelope must be a struct with min/max or min_bounds/max_bounds.');
    end
    if isfield(value, 'min')
        envelope.min_bounds = reshape(value.min, 1, []);
    elseif isfield(value, 'min_bounds')
        envelope.min_bounds = reshape(value.min_bounds, 1, []);
    end
    if isfield(value, 'max')
        envelope.max_bounds = reshape(value.max, 1, []);
    elseif isfield(value, 'max_bounds')
        envelope.max_bounds = reshape(value.max_bounds, 1, []);
    end
    if isfield(value, 'name')
        envelope.name = value.name;
    else
        envelope.name = 'Differential Envelope';
    end
    if isfield(envelope, 'min_bounds') && isfield(envelope, 'max_bounds') ...
            && (numel(envelope.min_bounds) ~= 3 || numel(envelope.max_bounds) ~= 3)
        error('load_diff_mount_config:invalidEnvelope', ...
            'diff_envelope min/max bounds must be 1x3 vectors.');
    end
end
