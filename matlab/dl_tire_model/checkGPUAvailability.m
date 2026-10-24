function [gpu_available, gpu_info] = checkGPUAvailability()
% CHECKGPUAVAILABILITY Check GPU availability and display information
%
% This function checks if a GPU is available for training and provides
% information about the detected GPU. It also handles fallback to CPU
% if GPU is not available.
%
% Outputs:
%   gpu_available - Boolean indicating if GPU is available
%   gpu_info      - Structure with GPU information:
%                   .Name         - GPU device name
%                   .Index        - GPU device index
%                   .ComputeCapability - GPU compute capability
%                   .TotalMemory  - Total GPU memory in GB
%                   .AvailableMemory - Available GPU memory in GB
%
% Example:
%   [gpu_available, gpu_info] = checkGPUAvailability();
%   if gpu_available
%       fprintf('Using GPU: %s\n', gpu_info.Name);
%   end
%
% Requirements: 9.4, 9.5, 9.7

    fprintf('=== GPU Detection ===\n');
    
    % Check if Parallel Computing Toolbox is available
    if ~license('test', 'Distrib_Computing_Toolbox')
        fprintf('WARNING: Parallel Computing Toolbox not available.\n');
        fprintf('GPU acceleration requires Parallel Computing Toolbox.\n');
        fprintf('Training will use CPU.\n\n');
        gpu_available = false;
        gpu_info = struct();
        return;
    end
    
    % Check GPU device count
    try
        gpu_count = gpuDeviceCount;
    catch ME
        fprintf('WARNING: Error checking GPU availability: %s\n', ME.message);
        fprintf('Training will use CPU.\n\n');
        gpu_available = false;
        gpu_info = struct();
        return;
    end
    
    if gpu_count == 0
        fprintf('No GPU detected.\n');
        fprintf('Training will use CPU (this may be slower).\n\n');
        gpu_available = false;
        gpu_info = struct();
        return;
    end
    
    % GPU is available - get information
    try
        gpu_device = gpuDevice;
        gpu_available = true;
        
        % Store GPU information
        gpu_info = struct();
        gpu_info.Name = gpu_device.Name;
        gpu_info.Index = gpu_device.Index;
        gpu_info.ComputeCapability = sprintf('%d.%d', ...
            gpu_device.ComputeCapability(1), gpu_device.ComputeCapability(2));
        gpu_info.TotalMemory = gpu_device.TotalMemory / 1e9;  % Convert to GB
        gpu_info.AvailableMemory = gpu_device.AvailableMemory / 1e9;  % Convert to GB
        
        % Display GPU information
        fprintf('GPU detected: %s\n', gpu_info.Name);
        fprintf('  Index: %d\n', gpu_info.Index);
        fprintf('  Compute Capability: %s\n', gpu_info.ComputeCapability);
        fprintf('  Total Memory: %.2f GB\n', gpu_info.TotalMemory);
        fprintf('  Available Memory: %.2f GB\n', gpu_info.AvailableMemory);
        fprintf('GPU acceleration will be used for training.\n\n');
        
    catch ME
        fprintf('WARNING: GPU detected but failed to initialize: %s\n', ME.message);
        fprintf('Training will use CPU.\n\n');
        gpu_available = false;
        gpu_info = struct();
    end
end
