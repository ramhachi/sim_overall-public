function [total_mass_penalty, m_unsprung, sprung_mass_penalty, unsprung_delta] = compute_track_width_mass_distribution(base_unsprung, track_f, track_r, varargin)
% compute_track_width_mass_distribution - estimate width-driven sprung/unsprung mass growth

p = inputParser;
addRequired(p, 'base_unsprung', @(x) isnumeric(x) && numel(x) == 4);
addRequired(p, 'track_f', @(x) isnumeric(x) && isscalar(x));
addRequired(p, 'track_r', @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'base_track', 1.20, @(x) isnumeric(x) && isscalar(x));
addParameter(p, 'sprung_penalty_per_meter', 6.0, @(x) isnumeric(x) && isscalar(x) && x >= 0);
addParameter(p, 'unsprung_penalty_per_meter', 1.5, @(x) isnumeric(x) && isscalar(x) && x >= 0);
parse(p, base_unsprung, track_f, track_r, varargin{:});

base_unsprung = reshape(p.Results.base_unsprung, 1, []);
base_track = p.Results.base_track;

front_delta = max(0.0, track_f - base_track);
rear_delta = max(0.0, track_r - base_track);

unsprung_delta = [front_delta, front_delta, rear_delta, rear_delta] * p.Results.unsprung_penalty_per_meter;
sprung_mass_penalty = (front_delta + rear_delta) * p.Results.sprung_penalty_per_meter;

m_unsprung = base_unsprung + unsprung_delta;
total_mass_penalty = sprung_mass_penalty + sum(unsprung_delta);

end
