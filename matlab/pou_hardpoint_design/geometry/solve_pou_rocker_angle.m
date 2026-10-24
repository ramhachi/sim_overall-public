function [theta, Lp, ok] = solve_pou_rocker_angle(bc, M, Lp0, theta_guess)
% SOLVE_POU_ROCKER_ANGLE Rotate the rigid rocker so the pushrod length stays Lp0.
%
% bc has pushrod_pin, frame_pivot, rotation_axis. M is the chassis-frame
% pushrod point on the upright. Returns the rocker rotation and residual
% pushrod length.

    if nargin < 4 || isempty(theta_guess)
        theta_guess = 0.0;
    end
    f = @(th) norm(rotate_about_axis(bc.pushrod_pin, bc.frame_pivot, ...
        bc.rotation_axis, th) - M) - Lp0;
    theta = theta_guess;
    ok = false;
    for span = [0.8, 1.5, 2.5]
        lo = theta_guess - span;
        hi = theta_guess + span;
        flo = f(lo);
        fhi = f(hi);
        if isfinite(flo) && isfinite(fhi) && flo * fhi <= 0
            try
                theta = fzero(f, [lo, hi]);
                ok = true;
            catch
                ok = false;
            end
            break;
        end
    end
    if ~ok
        opts = optimoptions('fsolve', 'Display', 'off', ...
            'FunctionTolerance', 1e-12, 'StepTolerance', 1e-12);
        [theta, fval, exitflag] = fsolve(f, theta_guess, opts);
        ok = exitflag > 0 && all(isfinite(theta)) && abs(fval) < 1e-6;
    end
    Lp = norm(rotate_about_axis(bc.pushrod_pin, bc.frame_pivot, ...
        bc.rotation_axis, theta) - M);
end

function p = rotate_about_axis(p0, origin, axis, angle)
    p = origin + axis_angle_rotation(axis, angle) * (p0 - origin);
end
