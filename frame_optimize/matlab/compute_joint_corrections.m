function [jf, offsets] = compute_joint_corrections(nodes, elements, bonded)
% COMPUTE_JOINT_CORRECTIONS Per-element weld-joint knockdown and rigid end offsets (#48).
%
%   [jf, offsets] = compute_joint_corrections(nodes, elements, bonded)
%
%   nodes    - Nx3 [m]
%   elements - Mx>=8 FEM rows [ni nj D t E G rho shape ...] (D, t in mm)
%   bonded   - Mx1 logical, true for bonded (CFRP) members: no weld knockdown
%
%   jf      - Mx1 bending/torsion factor, mean of the two end factors
%   offsets - Mx2 rigid end lengths [m] (chord radius / sin(theta), <= 0.4 L)
%
% At each end the chord is the largest-OD other member at that node. A member
% within 30 deg of its chord is treated as the continuous chord (jf = 1, no
% offset). Otherwise the end is a branch: CIDECT-style factor ported from
% fsae_frame_optimizer.py:462-492
%   jf = clip(1 - 0.15*(gamma/10)*(1 - sqrt(beta))/sin(theta), 0.5, 1)
%   beta = D_brace/D_chord, gamma = D_chord/(2 t_chord)
% and the brace end inside the chord is rigid (L_eff = L - offsets).
% ponytail: scalar jf per member; per-end 6x6 joint springs from offline shell
% sub-models are the stage-2 upgrade (docs/README "Joint model").

    if nargin < 3 || isempty(bonded)
        bonded = false(size(elements, 1), 1);
    end
    m = size(elements, 1);
    ni = elements(:, 1);
    nj = elements(:, 2);
    vec = nodes(nj, :) - nodes(ni, :);
    L = vecnorm(vec, 2, 2);
    dir = vec ./ L;
    D = elements(:, 3);
    t = elements(:, 4);

    jf_end = ones(m, 2);
    offsets = zeros(m, 2);
    for e = 1:m
        ends = [ni(e), nj(e)];
        for k = 1:2
            others = find((ni == ends(k) | nj == ends(k)) & (1:m)' ~= e);
            if isempty(others)
                continue;
            end
            [~, c] = max(D(others));
            c = others(c);
            sin_theta = norm(cross(dir(e, :), dir(c, :)));
            if sin_theta < sind(30)
                continue;  % continuous with the chord
            end
            beta = D(e) / D(c);
            gamma = D(c) / (2 * t(c));
            jf_end(e, k) = min(1, max(0.5, 1 - 0.15 * (gamma / 10) * (1 - sqrt(min(beta, 1))) / sin_theta));
            offsets(e, k) = min(0.4 * L(e), D(c) / 2000 / sin_theta);
        end
    end
    jf_end(bonded, :) = 1;
    jf = mean(jf_end, 2);
end
