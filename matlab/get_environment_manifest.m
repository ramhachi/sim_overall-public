function env = get_environment_manifest()
% GET_ENVIRONMENT_MANIFEST Record host/runtime inputs that affect numerics.

env = struct();
env.matlab_version = version();
env.matlab_release = version('-release');
env.computer = computer();
env.architecture = computer('arch');
env.num_cores = feature('numcores');
env.os = system_dependent('getos');
env.blas = safe_version_query('-blas');
env.lapack = safe_version_query('-lapack');

[status, cpu] = system('sysctl -n machdep.cpu.brand_string 2>/dev/null');
if status == 0 && ~isempty(strtrim(cpu))
    env.cpu = strtrim(cpu);
else
    env.cpu = getenv('PROCESSOR_IDENTIFIER');
end

try
    compilers = mex.getCompilerConfigurations('C++', 'Selected');
    if isempty(compilers)
        env.compiler = 'unknown';
    else
        env.compiler = sprintf('%s %s', compilers(1).Name, compilers(1).Version);
    end
catch
    env.compiler = 'unknown';
end

[status, eigen] = system('pkg-config --modversion eigen3 2>/dev/null');
if status == 0 && ~isempty(strtrim(eigen))
    env.eigen = strtrim(eigen);
else
    env.eigen = 'unknown';
end
end

function value = safe_version_query(option)
try
    value = version(option);
catch
    value = 'unknown';
end
end
