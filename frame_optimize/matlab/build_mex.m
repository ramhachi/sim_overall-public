function build_mex()
% BUILD_MEX Compile MEX FEM solver with Eigen3 support
%
% This script compiles the C++ MEX file beam_fem_solver.cpp
% Requirements:
%   - Eigen3 library installed
%   - C++14 compatible compiler
%   - Classic C MEX ABI support
%
% Platform-specific Eigen paths:
%   macOS (Homebrew):   /opt/homebrew/include/eigen3 or /usr/local/include/eigen3
%   Linux (apt):        /usr/include/eigen3
%   Windows (manual):   C:/eigen3 (set manually)

    fprintf('Building MEX FEM solver...\n');

    % Get current directory
    current_dir = fileparts(mfilename('fullpath'));
    mex_dir = fullfile(current_dir, '..', 'mex');
    mex_file = fullfile(mex_dir, 'beam_fem_solver.cpp');

    if ~isfile(mex_file)
        error('build_mex:FileNotFound', 'MEX source file not found: %s', mex_file);
    end

    % Detect Eigen3 path
    eigen_path = detect_eigen_path();
    if isempty(eigen_path)
        error('build_mex:EigenNotFound', ...
              ['Eigen3 library not found. Please install:\n', ...
               '  macOS:   brew install eigen\n', ...
               '  Linux:   sudo apt-get install libeigen3-dev\n', ...
               '  Windows: Download from https://eigen.tuxfamily.org/']);
    end

    fprintf('Found Eigen3 at: %s\n', eigen_path);

    % Build MEX command
    mex_args = {'-v', ...  % Verbose
                '-R2017b', ...  % Classic C MEX ABI for mexFunction()
                ['-I', eigen_path], ...  % Eigen include path
                mex_file};

    % Platform-specific optimization flags
    if isunix
        mex_args = [mex_args, ...
                    'CXXFLAGS="$CXXFLAGS -O3 -march=native -std=c++14"'];
        if ismac
            % This solver exposes the classic C MEX entry point
            % mexFunction() from a C++ translation unit. MATLAB R2025b on
            % macOS otherwise adds C++ MEX adapter exports, which require
            % mexCreateMexFunction/mexFunctionAdapter symbols and break
            % linking for C-API MEX files built as C++.
            mex_args = [mex_args, ...
                        'LINKEXPORTCPP='];
        end
    elseif ispc
        mex_args = [mex_args, ...
                    'COMPFLAGS="$COMPFLAGS /O2 /std:c++14"'];
    end

    % Change to output directory
    orig_dir = pwd;
    cd(mex_dir);

    try
        % Compile
        fprintf('Compiling with command:\n  mex %s\n', strjoin(mex_args, ' '));
        mex(mex_args{:});

        fprintf('\n✓ MEX compilation successful!\n');
        fprintf('  Location: %s\n', mex_dir);

        % Test if MEX file was created
        mex_output = fullfile(mex_dir, ['beam_fem_solver.', mexext]);
        if isfile(mex_output)
            fprintf('  MEX file: %s\n', mex_output);
        else
            warning('build_mex:OutputNotFound', 'Expected MEX file not found: %s', mex_output);
        end

    catch ME
        cd(orig_dir);
        fprintf('\n✗ MEX compilation failed!\n');
        fprintf('Error: %s\n', ME.message);
        rethrow(ME);
    end

    cd(orig_dir);
end

function eigen_path = detect_eigen_path()
% Detect Eigen3 installation path

    eigen_path = '';

    % Candidate paths
    candidates = {
        '/opt/homebrew/include/eigen3',  % macOS ARM (M1/M2)
        '/usr/local/include/eigen3',     % macOS Intel
        '/usr/include/eigen3',           % Linux
        'C:/eigen3',                     % Windows (manual install)
        'C:/Program Files/eigen3'        % Windows (alternative)
    };

    % Check each candidate
    for i = 1:length(candidates)
        test_file = fullfile(candidates{i}, 'Eigen', 'Dense');
        if isfile(test_file) || isfolder(test_file)
            eigen_path = candidates{i};
            return;
        end
    end

    % Try pkg-config (Linux/macOS)
    if isunix
        [status, result] = system('pkg-config --cflags eigen3');
        if status == 0
            % Extract path from "-I/path/to/eigen3"
            tokens = regexp(strtrim(result), '-I(\S+)', 'tokens');
            if ~isempty(tokens)
                eigen_path = tokens{1}{1};
                return;
            end
        end
    end
end
