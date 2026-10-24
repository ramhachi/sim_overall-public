function profile = resolve_easystart_docker_build_profile()
%RESOLVE_EASYSTART_DOCKER_BUILD_PROFILE Select the Docker/CMake profile for this host.

raw_arch = detect_raw_architecture();
normalized_arch = normalize_architecture(raw_arch);

switch normalized_arch
    case 'arm64'
        docker_platform = 'linux/arm64';
        cpu_profile = 'arm-native';
    case 'amd64'
        docker_platform = 'linux/amd64';
        cpu_profile = 'x86-native';
    otherwise
        docker_platform = '';
        cpu_profile = 'auto';
end

platform_tag = docker_platform;
if isempty(platform_tag)
    platform_tag = 'host-default';
end
platform_tag = strrep(platform_tag, '/', '-');

profile = struct( ...
    'host_arch', normalized_arch, ...
    'raw_host_arch', raw_arch, ...
    'docker_platform', docker_platform, ...
    'cpu_optimization_profile', cpu_profile, ...
    'generator', 'Ninja', ...
    'build_signature', sprintf('%s__%s__release__ninja', platform_tag, cpu_profile), ...
    'build_targets', {{ ...
        'laptime_sim', ...
        'test_14dof_new', ...
        'test_velocity_profile', ...
        'test_aero_integration', ...
        'test_aero_map_4d', ...
        'test_lsd', ...
        'fsae_rl_server', ...
        'rl_actor_check' ...
    }} ...
);
end

function raw_arch = detect_raw_architecture()
raw_arch = '';

if ispc
    raw_arch = strtrim(getenv('PROCESSOR_ARCHITECTURE'));
    if isempty(raw_arch)
        raw_arch = strtrim(getenv('PROCESSOR_ARCHITEW6432'));
    end
else
    [status, output] = system('uname -m');
    if status == 0
        raw_arch = strtrim(output);
    end
end

if isempty(raw_arch)
    raw_arch = computer('arch');
end
end

function normalized_arch = normalize_architecture(raw_arch)
arch = lower(strtrim(raw_arch));

switch arch
    case {'arm64', 'aarch64', 'maca64'}
        normalized_arch = 'arm64';
    case {'x86_64', 'amd64', 'maci64', 'glnxa64', 'win64'}
        normalized_arch = 'amd64';
    otherwise
        normalized_arch = arch;
end
end
