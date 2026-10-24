%% TEST_BEAM_FEM_CLOSED_FORM - MEX FEM vs closed-form beam solutions (#46)
% Builds the MEX (frame_optimize/mex) if it is not on the path yet.

fprintf('=== Beam FEM closed-form benchmarks (MEX) ===\n');
test_dir = fileparts(mfilename('fullpath'));
frame_root = fullfile(test_dir, '..');
addpath(fullfile(frame_root, 'matlab'));
addpath(fullfile(frame_root, 'mex'), '-begin');
if ~endsWith(which('beam_fem_solver'), mexext)
    build_mex();
    rehash;
end

D = 25.4; t = 1.65; E = 205; G = 80;
p = computeTubeSectionProperties(D, t, 'round');
A = p.A_mm2 * 1e-6; I = p.Iy_mm4 * 1e-12; J = p.J_mm4 * 1e-12; kA = p.Asy_mm2 * 1e-6;
clamp = [ones(6, 1), (1:6)', zeros(6, 1)];
row = @(i, j) [i, j, D, t, E, G, 7850, 1, 1.0, p.Asy_mm2, p.Asz_mm2];
P = 1000;

% 1. Short skew cantilever (4 elements), tip load normal to the axis:
%    delta = P L^3 / (3 E I) + P L / (kappa G A)
a = [1 2 2] / 3; f = [2 1 -2] / 3; L = 0.15;
nodes = (0:4)' * (L / 4) * a;
elements = cell2mat(arrayfun(@(k) row(k, k + 1), (1:4)', 'UniformOutput', false));
loads = [5 1 P*f(1); 5 2 P*f(2); 5 3 P*f(3)];
r = beam_fem_solver(nodes, elements, clamp, loads);
expected = P*L^3/(3*E*1e9*I) + P*L/(G*1e9*kA);
got = r.U(5, 1:3) * f';
assert(abs(got - expected) / expected < 1e-6, 'Timoshenko cantilever: %g vs %g', got, expected);

% Same mesh without shear areas (legacy 9 columns) must be the stiffer EB answer.
warning('off', 'beamFEM:Warning');
r_eb = beam_fem_solver(nodes, elements(:, 1:9), clamp, loads);
warning('on', 'beamFEM:Warning');
eb = P*L^3/(3*E*1e9*I);
assert(abs(r_eb.U(5, 1:3) * f' - eb) / eb < 1e-6, 'Legacy 9-column rows must give Euler-Bernoulli');

% 2. Round-tube torsion: theta = T L / (G J), K_torsion = T / theta[deg]
L = 0.5; T = 100;
nodes = [0 0 0; L/2 0 0; L 0 0];
r = beam_fem_solver(nodes, [row(1, 2); row(2, 3)], clamp, [3 4 T]);
theta = T*L/(G*1e9*J);
assert(abs(r.U(3, 4) - theta) / theta < 1e-9, 'Round tube twist');
assert(abs(r.K_torsion - T/rad2deg(theta)) / (T/rad2deg(theta)) < 1e-9, 'Round tube K_torsion');

% 3. Stress recovery (#50): root bending sigma = P L c / I, Euler ratio in compression
L = 0.5;
nodes = [0 0 0; 0 L 0];
r = beam_fem_solver(nodes, row(1, 2), clamp, [2 3 P]);
assert(abs(r.stress_max - P*L*1e3*(D/2)/p.Iy_mm4) / r.stress_max < 1e-9, 'Root bending stress');
assert(isequal(size(r.element_forces), [1 12]), 'element_forces must be Mx12');
r = beam_fem_solver(nodes, row(1, 2), clamp, [2 2 -P]);
assert(abs(r.buckling_ratio - P/(pi^2*E*1e9*I/L^2)) / r.buckling_ratio < 1e-9, 'Euler buckling ratio');

% 4. Rigid link (#52): load on a slave offset e from the tip twists it by P e L / (G J)
e = 0.1;
nodes = [0 0 0; L 0 0; L e 0];
r = beam_fem_solver(nodes, row(1, 2), clamp, [3 3 P], struct('rigid_links', [2 3]));
assert(abs(r.U(2, 4) - P*e*L/(G*1e9*J)) / (P*e*L/(G*1e9*J)) < 1e-9, 'Rigid link twist');

fprintf('Beam FEM closed-form benchmarks OK\n');
