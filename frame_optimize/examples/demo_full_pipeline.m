%% DEMO_FULL_PIPELINE - Redirect legacy demo path to the canonical SSOT flow
%
% This script used to build a fixed node/element problem and pass it into
% runFrameOptimization(...). That legacy contract is no longer valid because
% the canonical optimizer is parametric and ignores those fixed-geometry inputs.
%
% Keep this path as a redirect so existing users do not silently optimize the
% wrong problem. The recommended entrypoint remains:
%   frame_optimize/examples/run_frame_optimization.m

clear; close all; clc;

script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

fprintf('demo_full_pipeline is deprecated and now redirects to the canonical SSOT frame optimization.\n\n');

run_frame_optimization();
