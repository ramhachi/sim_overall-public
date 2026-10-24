function assertAgentObservationDimension(agent, expectedDim, context)
% ASSERTAGENTOBSERVATIONDIMENSION Guard against stale lower-dimension agents.

if nargin < 2 || isempty(expectedDim)
    expectedDim = size(get_rl_observation_bounds(), 1);
end
if nargin < 3 || isempty(context)
    context = 'Agent';
end

obsInfo = getObservationInfo(agent);
obsDim = obsInfo.Dimension(1);
if obsDim ~= expectedDim
    error(['%s observation dimension (%d) does not match expected dimension (%d). ' ...
        'Legacy agents trained on the older observation contract must be retrained from scratch.'], ...
        context, obsDim, expectedDim);
end
end
