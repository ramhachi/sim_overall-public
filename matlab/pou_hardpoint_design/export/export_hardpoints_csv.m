function csv_path = export_hardpoints_csv(results, geom, output_dir, options)
% EXPORT_HARDPOINTS_CSV - Export POU hardpoint coordinates for SolidWorks import
%
% INPUTS:
%   results    - Results struct from main_pou_hardpoint_optimizer
%   geom       - Suspension geometry struct from define_suspension_geometry
%   output_dir - Output directory path (default: current directory)
%   options    - (optional) Struct with fields:
%     .wheel_name  - Wheel label string (default: 'FL')
%     .format      - 'solidworks' or 'general' (default: 'solidworks')
%
% OUTPUT:
%   csv_path - Path to generated CSV file
%
% CSV FORMAT (SolidWorks point import compatible):
%   Point Name, X [mm], Y [mm], Z [mm]
%   Each hardpoint as a named 3D coordinate

    if nargin < 3 || isempty(output_dir)
        output_dir = '.';
    end
    if nargin < 4
        options = struct();
    end
    if ~isfield(options, 'wheel_name')
        wheel_names = {'FL', 'FR', 'RL', 'RR'};
        options.wheel_name = wheel_names{geom.wheel_index + 1};
    end
    if ~isfield(options, 'format')
        options.format = 'solidworks';
    end

    % Create output directory if needed
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    % Build filename
    csv_path = fullfile(output_dir, ...
        sprintf('pou_hardpoints_%s.csv', options.wheel_name));

    % Collect all hardpoint coordinates in chassis frame [m]
    points = struct();

    % Optimized POU hardpoints
    points.M_pivot = results.M_pivot_chassis(:)';
    points.bellcrank_pivot = results.bellcrank_pivot(:)';

    % Fixed A-arm hardpoints (from geometry definition - full DW)
    points.upper_arm_front_chassis = geom.upper_arm_front_chassis(:)';
    points.upper_arm_rear_chassis = geom.upper_arm_rear_chassis(:)';
    points.upper_arm_upright = geom.upper_arm_upright(:)';
    points.lower_arm_front_chassis = geom.lower_arm_front_chassis(:)';
    points.lower_arm_rear_chassis = geom.lower_arm_rear_chassis(:)';
    points.lower_arm_upright = geom.lower_arm_upright(:)';
    points.tie_rod_chassis = geom.tie_rod_chassis(:)';
    points.tie_rod_upright = geom.tie_rod_upright(:)';

    % Kingpin axis (origin + tip for visualization)
    points.kingpin_origin = geom.kingpin_axis.origin(:)';
    kingpin_tip = geom.kingpin_axis.origin + 0.15 * geom.kingpin_axis.direction;
    points.kingpin_tip = kingpin_tip(:)';

    % Caliper center
    points.caliper_center = geom.caliper.center(:)';

    % Write CSV
    fid = fopen(csv_path, 'w');
    if fid == -1
        error('Cannot open file for writing: %s', csv_path);
    end

    wn = options.wheel_name;

    if strcmp(options.format, 'solidworks')
        % SolidWorks reference point import format
        % Header line
        fprintf(fid, 'Point Name,X [mm],Y [mm],Z [mm]\n');

        % Write each point (convert m -> mm)
        write_point(fid, sprintf('%s_M_pivot', wn), points.M_pivot * 1000);
        write_point(fid, sprintf('%s_bellcrank_pivot', wn), points.bellcrank_pivot * 1000);
        write_point(fid, sprintf('%s_upper_arm_front_chassis', wn), points.upper_arm_front_chassis * 1000);
        write_point(fid, sprintf('%s_upper_arm_rear_chassis', wn), points.upper_arm_rear_chassis * 1000);
        write_point(fid, sprintf('%s_upper_arm_upright', wn), points.upper_arm_upright * 1000);
        write_point(fid, sprintf('%s_lower_arm_front_chassis', wn), points.lower_arm_front_chassis * 1000);
        write_point(fid, sprintf('%s_lower_arm_rear_chassis', wn), points.lower_arm_rear_chassis * 1000);
        write_point(fid, sprintf('%s_lower_arm_upright', wn), points.lower_arm_upright * 1000);
        write_point(fid, sprintf('%s_tie_rod_chassis', wn), points.tie_rod_chassis * 1000);
        write_point(fid, sprintf('%s_tie_rod_upright', wn), points.tie_rod_upright * 1000);
        write_point(fid, sprintf('%s_kingpin_origin', wn), points.kingpin_origin * 1000);
        write_point(fid, sprintf('%s_kingpin_tip', wn), points.kingpin_tip * 1000);
        write_point(fid, sprintf('%s_caliper_center', wn), points.caliper_center * 1000);
    else
        % General CSV with metadata
        fprintf(fid, 'Wheel,Point,X_m,Y_m,Z_m,X_mm,Y_mm,Z_mm\n');

        names = fieldnames(points);
        for i = 1:length(names)
            p = points.(names{i});
            fprintf(fid, '%s,%s,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f\n', ...
                wn, names{i}, p(1), p(2), p(3), ...
                p(1)*1000, p(2)*1000, p(3)*1000);
        end
    end

    fclose(fid);
    fprintf('CSV exported: %s\n', csv_path);
end

function write_point(fid, name, coords_mm)
    fprintf(fid, '%s,%.3f,%.3f,%.3f\n', name, coords_mm(1), coords_mm(2), coords_mm(3));
end
