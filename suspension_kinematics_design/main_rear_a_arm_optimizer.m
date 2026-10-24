function main_rear_a_arm_optimizer(json_file, run_options)
% MAIN_REAR_A_ARM_OPTIMIZER - Runtime-driven rear double-wishbone inverse-analysis.
% See run_a_arm_stage for options.
if nargin < 1, json_file = ''; end
if nargin < 2, run_options = struct(); end
run_a_arm_stage('rear', json_file, run_options);
end
