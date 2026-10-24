%% TEST_JOINT_CORRECTIONS - per-element joint factor and rigid end offsets (#48)

fprintf('=== Joint corrections (CIDECT jf + rigid end offsets) ===\n');
test_dir = fileparts(mfilename('fullpath'));
addpath(fullfile(test_dir, '..', 'matlab'));

% T-joint: chord 31.8x1.65 along x through node 2, brace 19.05x1.24 along y at 90 deg,
% and a 45-deg brace 25.4x1.65.
nodes = [0 0 0; 0.5 0 0; 1.0 0 0; 0.5 0.4 0; 0.9 0.4 0];
row = @(i, j, D, t) [i, j, D, t, 205, 80, 7850, 1, 1];
elements = [row(1, 2, 31.8, 1.65); row(2, 3, 31.8, 1.65); row(2, 4, 19.05, 1.24); row(2, 5, 25.4, 1.65)];
[jf, off] = compute_joint_corrections(nodes, elements, false(4, 1));

% Chord members are continuous at node 2 and free at nodes 1/3.
assert(all(off(1:2, :) == 0, 'all') && all(jf(1:2) == 1), 'Chord must be continuous');

% 90-deg brace: beta = 0.599, gamma = 9.64 -> end factor 0.6868; free end = 1.
beta = 19.05/31.8; gamma = 31.8/(2*1.65);
jf_end = 1 - 0.15*(gamma/10)*(1 - sqrt(beta));
assert(abs(jf(3) - (jf_end + 1)/2) < 1e-12, '90-deg brace jf');
assert(abs(off(3, 1) - 31.8/2000) < 1e-12 && off(3, 2) == 0, '90-deg brace offset = chord radius');

% 45-deg brace: offset = r_chord / sin(45)
assert(abs(off(4, 1) - 31.8/2000/sind(45)) < 1e-12, '45-deg brace offset');
assert(jf(4) < 1 && jf(4) >= 0.5, '45-deg brace jf range');

% Bonded CFRP members get no weld knockdown but keep the geometric offsets.
[jf_b, off_b] = compute_joint_corrections(nodes, elements, [false; false; true; true]);
assert(all(jf_b(3:4) == 1) && isequal(off_b, off), 'CFRP: no weld factor');

fprintf('Joint corrections OK\n');
