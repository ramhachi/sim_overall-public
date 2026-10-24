function version = get_code_version(project_root)
% GET_CODE_VERSION Return the current git commit for provenance metadata.

if nargin < 1 || isempty(project_root)
    project_root = fileparts(mfilename('fullpath'));
end

version = 'unknown';
[status, out] = system(sprintf('git -C "%s" rev-parse HEAD 2>/dev/null', project_root));
if status == 0
    candidate = strtrim(out);
    if ~isempty(candidate)
        version = candidate;
    end
end
end
