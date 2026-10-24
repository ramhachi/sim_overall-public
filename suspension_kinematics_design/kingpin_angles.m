function [kpi_deg, caster_deg] = kingpin_angles(kp_dir)
% KINGPIN_ANGLES Separate KPI (Y-Z plane) from caster (X-Z plane), in degrees.
kpi_deg = asind(abs(kp_dir(2)));
caster_deg = asind(abs(kp_dir(1)));
end
