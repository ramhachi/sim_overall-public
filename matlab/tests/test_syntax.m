try
    addpath(fullfile(fileparts(mfilename('fullpath')), '..', '..'));
    pcode EasyStart
    disp('Syntax OK');
catch e
    disp('Syntax Error:');
    disp(e.message);
end
