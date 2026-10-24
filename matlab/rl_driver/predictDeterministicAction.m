function action = predictDeterministicAction(actorModel, obs)
% PREDICTDETERMINISTICACTION Evaluate the actor mean path without sampling.

result = predict(actorModel, dlarray(obs, 'CB'));
if iscell(result)
    action = toNumeric(result{1});
else
    action = toNumeric(result);
end
action = action(:);
end

function value = toNumeric(x)
if isa(x, 'dlarray')
    value = double(extractdata(x));
else
    value = double(x);
end
end
