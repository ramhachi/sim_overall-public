function main_a_arm_optimizer(json_file, run_options)
% MAIN_A_ARM_OPTIMIZER - Runtime-driven front double-wishbone inverse-analysis.
% See run_a_arm_stage for options.
if nargin < 1, json_file = ''; end
if nargin < 2, run_options = struct(); end
run_a_arm_stage('front', json_file, run_options);
end
