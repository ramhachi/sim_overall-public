function params = extract_vehicle_params_from_artifact(data)
% extract_vehicle_params_from_artifact - unified artifact or plain params から車両諸元を取り出す

if ~isstruct(data)
    error('Vehicle artifact must decode to a struct.');
end

if isfield(data, 'params') && isstruct(data.params)
    params = data.params;
else
    params = data;
end

end
