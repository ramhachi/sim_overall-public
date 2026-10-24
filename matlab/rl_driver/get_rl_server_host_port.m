function port = get_rl_server_host_port()
%GET_RL_SERVER_HOST_PORT Return the host port mapped to the RL TCP server.

port = parse_port_env('FSAE_HOST_RL_PORT', 22345);
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
