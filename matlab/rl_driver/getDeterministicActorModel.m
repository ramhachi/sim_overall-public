function actorModel = getDeterministicActorModel(agent)
% GETDETERMINISTICACTORMODEL Return the actor mean-path model used for deployment.

assertAgentObservationDimension(agent);
actor = getActor(agent);
try
    actorModel = getModel(actor);
catch
    actorModel = actor.Model;
end
end
