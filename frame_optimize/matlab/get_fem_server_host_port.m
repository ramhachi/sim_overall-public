function port = get_fem_server_host_port()
%GET_FEM_SERVER_HOST_PORT Return the host port mapped to the FEM TCP server.

port = parse_port_env('FSAE_HOST_FEM_PORT', 50051);
end

function port = parse_port_env(env_name, default_port)
port_text = strtrim(getenv(env_name));
if isempty(port_text)
    port = default_port;
    return;
end

parsed = str2double(port_text);
if ~isfinite(parsed) || parsed <= 0 || mod(parsed, 1) ~= 0
    port = default_port;
    return;
end

port = parsed;
end
