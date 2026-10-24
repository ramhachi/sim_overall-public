function geomMirror = mirror_wishbone_geometry(geom)
% MIRROR_WISHBONE_GEOMETRY Mirror a double-wishbone corner across the Y=0 plane.

geomMirror = geom;
fields = {'U_F', 'U_R', 'U_U', 'L_F', 'L_R', 'L_U', 'W_C', ...
    'upper_arm_axis_origin', 'lower_arm_axis_origin'};
for i = 1:numel(fields)
    fieldName = fields{i};
    geomMirror.(fieldName)(2) = -geomMirror.(fieldName)(2);
end
if isfield(geomMirror, 'tie_rod_chassis')
    geomMirror.tie_rod_chassis(2) = -geomMirror.tie_rod_chassis(2);
end
if isfield(geomMirror, 'tie_rod_upright')
    geomMirror.tie_rod_upright(2) = -geomMirror.tie_rod_upright(2);
end
if isfield(geomMirror, 'Spindle_Dir')
    geomMirror.Spindle_Dir(2) = -geomMirror.Spindle_Dir(2);
end
geomMirror.is_left = ~geom.is_left;
geomMirror.upper_arm_axis = geomMirror.U_R - geomMirror.U_F;
geomMirror.upper_arm_axis = geomMirror.upper_arm_axis / max(norm(geomMirror.upper_arm_axis), 1e-9);
geomMirror.lower_arm_axis = geomMirror.L_R - geomMirror.L_F;
geomMirror.lower_arm_axis = geomMirror.lower_arm_axis / max(norm(geomMirror.lower_arm_axis), 1e-9);
geomMirror.kingpin_axis.origin = geomMirror.L_U;
geomMirror.kingpin_axis.direction = (geomMirror.U_U - geomMirror.L_U) / ...
    max(norm(geomMirror.U_U - geomMirror.L_U), 1e-9);
end
