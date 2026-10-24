function cd_a = compute_aero_cd_a(cl_a, varargin)
% compute_aero_cd_a - optimizer 間で共通の簡易 drag polar

p = inputParser;
addRequired(p, 'cl_a', @isnumeric);
addParameter(p, 'cd0', 0.30, @isnumeric);
addParameter(p, 'k_induced', 0.10, @isnumeric);
addParameter(p, 'power', 2.0, @isnumeric);
parse(p, cl_a, varargin{:});

cl_mag = max(cl_a, 0.0);
cd_a = p.Results.cd0 + p.Results.k_induced * (cl_mag .^ p.Results.power);

end
