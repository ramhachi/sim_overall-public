function fields = roll_center_frame(z_down_m)
% ROLL_CENTER_FRAME Return explicit Z-down coordinate and ground-up height.
fields = struct();
fields.roll_center_z_down_m = z_down_m;
fields.roll_center_height_m = -z_down_m;
fields.roll_center_z_m = -z_down_m; % legacy alias for height
end
