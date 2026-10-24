% Check Validation Readiness
% This script verifies that all prerequisites are met before running
% the final system validation

function ready = check_validation_readiness()
    fprintf('\n');
    fprintf('=================================================================\n');
    fprintf('Checking Validation Readiness\n');
    fprintf('=================================================================\n\n');
    
    ready = true;
    issues = {};
    
    %% Check 1: MATLAB Version
    fprintf('1. Checking MATLAB version...\n');
    matlab_version = version('-release');
    matlab_year = str2double(matlab_version(1:4));
    
    if matlab_year >= 2023
        fprintf('   ✓ MATLAB %s (>= R2023a required)\n\n', matlab_version);
    else
        fprintf('   ✗ MATLAB %s (R2023a or later required)\n\n', matlab_version);
        ready = false;
        issues{end+1} = 'MATLAB version too old';
    end
    
    %% Check 2: Required Toolboxes
    fprintf('2. Checking required toolboxes...\n');
    
    toolboxes = {
        'Deep Learning Toolbox', 'neural_network_toolbox'
        'Statistics and Machine Learning Toolbox', 'statistics_toolbox'
        'Parallel Computing Toolbox', 'distrib_computing_toolbox'
    };
    
    for i = 1:size(toolboxes, 1)
        toolbox_name = toolboxes{i, 1};
        toolbox_id = toolboxes{i, 2};
        
        if license('test', toolbox_id)
            fprintf('   ✓ %s\n', toolbox_name);
        else
            if strcmp(toolbox_name, 'Parallel Computing Toolbox')
                fprintf('   ⚠ %s (optional, for GPU acceleration)\n', toolbox_name);
            else
                fprintf('   ✗ %s (required)\n', toolbox_name);
                ready = false;
                issues{end+1} = sprintf('Missing %s', toolbox_name);
            end
        end
    end
    fprintf('\n');
    
    %% Check 3: GPU Availability
    fprintf('3. Checking GPU availability...\n');
    
    if gpuDeviceCount > 0
        try
            gpu = gpuDevice();
            fprintf('   ✓ GPU detected: %s\n', gpu.Name);
            fprintf('     Memory: %.2f GB available\n', gpu.AvailableMemory / 1e9);
            fprintf('     Compute Capability: %.1f\n', gpu.ComputeCapability);
        catch
            fprintf('   ⚠ GPU detected but cannot be accessed\n');
        end
    else
        fprintf('   ⚠ No GPU detected (training will use CPU)\n');
    end
    fprintf('\n');
    
    %% Check 4: Data Files
    fprintf('4. Checking data files...\n');
    
    data_files = {
        '../../data/tiredata/DriveBrake_sampled_quarter.csv'
    };
    
    for i = 1:length(data_files)
        if isfile(data_files{i})
            fprintf('   ✓ %s\n', data_files{i});
        else
            fprintf('   ✗ %s (not found)\n', data_files{i});
            ready = false;
            issues{end+1} = sprintf('Missing data file: %s', data_files{i});
        end
    end
    fprintf('\n');
    
    %% Check 5: Required Functions
    fprintf('5. Checking required functions...\n');
    
    required_functions = {
        'loadTireData.m'
        'extractFeatures.m'
        'extractTargets.m'
        'removeMissingData.m'
        'normalizeData.m'
        'denormalizeData.m'
        'splitData.m'
        'createTireModel.m'
        'trainTireModel.m'
        'evaluateModel.m'
        'computeRMSE.m'
        'computeR2.m'
        'computeTIC.m'
        'preprocessDataWrapper.m'
        'final_system_validation.m'
    };
    
    missing_functions = {};
    for i = 1:length(required_functions)
        if isfile(required_functions{i})
            % Function exists
        else
            missing_functions{end+1} = required_functions{i};
        end
    end
    
    if isempty(missing_functions)
        fprintf('   ✓ All required functions present (%d files)\n', length(required_functions));
    else
        fprintf('   ✗ Missing %d functions:\n', length(missing_functions));
        for i = 1:length(missing_functions)
            fprintf('     - %s\n', missing_functions{i});
        end
        ready = false;
        issues{end+1} = sprintf('Missing %d required functions', length(missing_functions));
    end
    fprintf('\n');
    
    %% Check 6: Output Directory
    fprintf('6. Checking output directory...\n');
    
    if ~exist('output', 'dir')
        fprintf('   ⚠ Output directory does not exist (will be created)\n');
        try
            mkdir('output');
            fprintf('   ✓ Created output directory\n');
        catch
            fprintf('   ✗ Cannot create output directory\n');
            ready = false;
            issues{end+1} = 'Cannot create output directory';
        end
    else
        fprintf('   ✓ Output directory exists\n');
    end
    fprintf('\n');
    
    %% Check 7: Disk Space
    fprintf('7. Checking disk space...\n');
    
    % Get current directory info
    if ispc
        [~, result] = system('dir');
    else
        [~, result] = system('df -h .');
    end
    
    fprintf('   ℹ Check manually if sufficient space (need ~1GB for models)\n\n');
    
    %% Summary
    fprintf('=================================================================\n');
    fprintf('Readiness Summary\n');
    fprintf('=================================================================\n\n');
    
    if ready
        fprintf('✓ SYSTEM READY FOR VALIDATION\n\n');
        fprintf('You can now run: run_final_validation\n\n');
    else
        fprintf('✗ SYSTEM NOT READY - Issues Found:\n\n');
        for i = 1:length(issues)
            fprintf('  %d. %s\n', i, issues{i});
        end
        fprintf('\nPlease resolve these issues before running validation.\n\n');
    end
    
    fprintf('=================================================================\n\n');
end
