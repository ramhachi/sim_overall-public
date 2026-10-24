function result = visualizeRLAutocrossTrace(config)
% VISUALIZERLAUTOCROSSTRACE Create trajectory plots and video from an RL trace CSV.
%
% result = visualizeRLAutocrossTrace(struct( ...
%     'trace_csv_path', 'output/runtime/.../Agent40_autocross.csv', ...
%     'track_csv_path', 'data/ASE_autocross_sim.csv', ...
%     'output_dir', 'output/runtime/rl_visualization/agent40'))
%
% The trace is expected to come from laptime_sim --output. The generated MP4
% uses a time-compressed replay so a full autocross lap remains inspectable.

if nargin < 1 || isempty(config)
    config = struct();
end
if ~isstruct(config)
    error('visualizeRLAutocrossTrace:InvalidConfig', 'config must be a struct.');
end

config = normalize_config(config);
ensure_dir(config.output_dir);

trace = readtable(config.trace_csv_path);
track = readtable(config.track_csv_path);
validate_trace(trace, config.trace_csv_path);
validate_track(track, config.track_csv_path);

track_geometry = compute_track_geometry(track);
metrics = compute_trace_metrics(trace, config);

[~, trace_name] = fileparts(config.trace_csv_path);
png_path = fullfile(config.output_dir, [trace_name, '_trajectory.png']);
video_path = fullfile(config.output_dir, [trace_name, '_replay.mp4']);
summary_path = fullfile(config.output_dir, [trace_name, '_summary.txt']);

make_summary_plot(trace, track_geometry, metrics, config, png_path);
if config.make_video
    make_replay_video(trace, track_geometry, metrics, config, video_path);
else
    video_path = '';
end
write_summary(summary_path, metrics, config, png_path, video_path);

result = struct();
result.png_path = png_path;
result.video_path = video_path;
result.summary_path = summary_path;
result.lap_time = metrics.lap_time;
result.distance_traveled = metrics.distance_traveled;
result.max_speed = metrics.max_speed;
result.max_lateral_error = metrics.max_lateral_error;
result.max_track_limit_excess = metrics.max_track_limit_excess;
end

function config = normalize_config(config)
if ~isfield(config, 'project_root') || isempty(config.project_root)
    this_dir = fileparts(mfilename('fullpath'));
    config.project_root = fileparts(fileparts(this_dir));
else
    config.project_root = char(string(config.project_root));
end
config.project_root = char(java.io.File(config.project_root).getCanonicalPath());

if ~isfield(config, 'trace_csv_path') || isempty(config.trace_csv_path)
    error('visualizeRLAutocrossTrace:MissingTrace', 'trace_csv_path is required.');
end
config.trace_csv_path = resolve_project_path(config.project_root, config.trace_csv_path);

if ~isfield(config, 'track_csv_path') || isempty(config.track_csv_path)
    config.track_csv_path = fullfile(config.project_root, 'data', 'ASE_autocross_sim.csv');
else
    config.track_csv_path = resolve_project_path(config.project_root, config.track_csv_path);
end

if ~isfield(config, 'output_dir') || isempty(config.output_dir)
    config.output_dir = fullfile(config.project_root, 'output', 'runtime', ...
        'rl_visualization', char(datetime('now', 'Format', 'yyyyMMdd_HHmmss')));
else
    config.output_dir = resolve_project_path(config.project_root, config.output_dir);
end

if ~isfield(config, 'title') || isempty(config.title)
    config.title = 'RL autocross trajectory';
else
    config.title = char(string(config.title));
end
if ~isfield(config, 'episode') || isempty(config.episode)
    config.episode = NaN;
end
if ~isfield(config, 'lap_time') || isempty(config.lap_time)
    config.lap_time = NaN;
end
if ~isfield(config, 'make_video') || isempty(config.make_video)
    config.make_video = true;
else
    config.make_video = logical(config.make_video);
end
if ~isfield(config, 'video_fps') || isempty(config.video_fps)
    config.video_fps = 30;
end
if ~isfield(config, 'video_speedup') || isempty(config.video_speedup)
    config.video_speedup = 8;
end
if ~isfield(config, 'video_quality') || isempty(config.video_quality)
    config.video_quality = 90;
end
if ~isfield(config, 'max_video_frames') || isempty(config.max_video_frames)
    config.max_video_frames = 900;
end
if ~isfield(config, 'plot_width') || isempty(config.plot_width)
    config.plot_width = 1600;
end
if ~isfield(config, 'plot_height') || isempty(config.plot_height)
    config.plot_height = 950;
end
if ~isfield(config, 'steering_ratio') || isempty(config.steering_ratio)
    config.steering_ratio = 4.5;
end
end

function validate_trace(trace, path_in)
required = {'time', 'x', 'y', 'psi', 'speed', 'steering', 'throttle', ...
    'brake', 'lateral_error'};
for i = 1:numel(required)
    if ~ismember(required{i}, trace.Properties.VariableNames)
        error('visualizeRLAutocrossTrace:MissingTraceColumn', ...
            'Trace %s is missing column "%s".', path_in, required{i});
    end
end
end

function validate_track(track, path_in)
required = {'x', 'y', 'width'};
for i = 1:numel(required)
    if ~ismember(required{i}, track.Properties.VariableNames)
        error('visualizeRLAutocrossTrace:MissingTrackColumn', ...
            'Track %s is missing column "%s".', path_in, required{i});
    end
end
end

function geometry = compute_track_geometry(track)
x = double(track.x(:));
y = double(track.y(:));
width = double(track.width(:));
if isempty(width)
    width = 4.5 * ones(size(x));
end
if isscalar(width)
    width = repmat(width, size(x));
end

dx = gradient(x);
dy = gradient(y);
norm_xy = hypot(dx, dy);
norm_xy(norm_xy < eps) = 1.0;
nx = -dy ./ norm_xy;
ny = dx ./ norm_xy;
half_width = 0.5 * width;

geometry.x = x;
geometry.y = y;
geometry.left_x = x + nx .* half_width;
geometry.left_y = y + ny .* half_width;
geometry.right_x = x - nx .* half_width;
geometry.right_y = y - ny .* half_width;
geometry.width = width;
end

function metrics = compute_trace_metrics(trace, config)
time = double(trace.time(:));
x = double(trace.x(:));
y = double(trace.y(:));
speed = double(trace.speed(:));
lat_error = double(trace.lateral_error(:));
metrics.lap_time = time(end) - time(1);
metrics.distance_traveled = sum(hypot(diff(x), diff(y)));
metrics.max_speed = max(speed);
metrics.avg_speed = mean(speed);
metrics.max_lateral_error = max(abs(lat_error));
if ismember('track_margin_effective_margin', trace.Properties.VariableNames)
    margin = double(trace.track_margin_effective_margin(:));
    metrics.max_track_limit_excess = max(max(-margin, 0));
else
    metrics.max_track_limit_excess = NaN;
end
if ismember('delta_sw', trace.Properties.VariableNames)
    delta_sw = double(trace.delta_sw(:));
    metrics.max_abs_sw_deg = max(abs(rad2deg(delta_sw)));
else
    delta_sw = [];
    metrics.max_abs_sw_deg = NaN;
end
if ismember('actual_delta_tire', trace.Properties.VariableNames)
    actual_delta_tire = double(trace.actual_delta_tire(:));
    if all(abs(actual_delta_tire) < 1e-9) && ~isempty(delta_sw) && config.steering_ratio > 0
        actual_delta_tire = delta_sw ./ config.steering_ratio;
    end
    metrics.max_abs_tire_steer_deg = max(abs(rad2deg(actual_delta_tire)));
elseif ~isempty(delta_sw) && config.steering_ratio > 0
    metrics.max_abs_tire_steer_deg = max(abs(rad2deg(delta_sw ./ config.steering_ratio)));
else
    metrics.max_abs_tire_steer_deg = NaN;
end
end

function make_summary_plot(trace, geometry, metrics, config, png_path)
fig = figure('Name', 'RL Autocross Trajectory', ...
    'Position', [50 50 config.plot_width config.plot_height], ...
    'Color', 'w', 'Visible', figure_visibility());
layout = tiledlayout(fig, 3, 4, 'TileSpacing', 'compact', 'Padding', 'compact');

ax_path = nexttile(layout, [3 3]);
hold(ax_path, 'on');
grid(ax_path, 'on');
axis(ax_path, 'equal');
plot(ax_path, geometry.left_x, geometry.left_y, '-', 'Color', [0.15 0.15 0.15], ...
    'LineWidth', 1.1, 'DisplayName', 'Track boundary');
plot(ax_path, geometry.right_x, geometry.right_y, '-', 'Color', [0.15 0.15 0.15], ...
    'LineWidth', 1.1, 'HandleVisibility', 'off');
plot(ax_path, geometry.x, geometry.y, '--', 'Color', [0.55 0.55 0.55], ...
    'LineWidth', 0.8, 'DisplayName', 'Centerline');
scatter(ax_path, trace.x, trace.y, 9, trace.speed, 'filled', 'DisplayName', 'RL path');
plot(ax_path, trace.x(1), trace.y(1), 'go', 'MarkerFaceColor', 'g', ...
    'MarkerSize', 7, 'DisplayName', 'Start');
plot(ax_path, trace.x(end), trace.y(end), 'rs', 'MarkerFaceColor', 'r', ...
    'MarkerSize', 7, 'DisplayName', 'Finish');
if height(trace) >= 60
    arrow_idx = unique(round(linspace(1, height(trace), 18)));
    quiver(ax_path, trace.x(arrow_idx), trace.y(arrow_idx), ...
        2.5 * cos(trace.psi(arrow_idx)), 2.5 * sin(trace.psi(arrow_idx)), 0, ...
        'Color', [0 0.25 0.8], 'LineWidth', 0.7, 'MaxHeadSize', 0.6, ...
        'DisplayName', 'Heading');
end
title(ax_path, path_title(config, metrics));
xlabel(ax_path, 'X [m]');
ylabel(ax_path, 'Y [m]');
cb = colorbar(ax_path);
ylabel(cb, 'Speed [m/s]');
legend(ax_path, 'Location', 'bestoutside');
style_axis(ax_path);

time = trace.time - trace.time(1);

ax_speed = nexttile(layout);
hold(ax_speed, 'on');
grid(ax_speed, 'on');
plot(ax_speed, time, trace.speed, 'b-', 'LineWidth', 1.1);
ylabel(ax_speed, 'Speed [m/s]');
title(ax_speed, sprintf('Speed: max %.2f m/s, avg %.2f m/s', ...
    metrics.max_speed, metrics.avg_speed));

ax_control = nexttile(layout);
hold(ax_control, 'on');
grid(ax_control, 'on');
plot(ax_control, time, trace.steering, 'Color', [0.1 0.1 0.8], ...
    'LineWidth', 0.9, 'DisplayName', 'Steer torque');
plot(ax_control, time, trace.throttle, 'Color', [0.0 0.55 0.0], ...
    'LineWidth', 0.9, 'DisplayName', 'Throttle');
plot(ax_control, time, trace.brake, 'Color', [0.85 0.0 0.0], ...
    'LineWidth', 0.9, 'DisplayName', 'Brake');
xlabel(ax_control, 'Time [s]');
ylabel(ax_control, 'Command');
title(ax_control, 'RL commands');
legend(ax_control, 'Location', 'best');
style_axis(ax_control);

ax_error = nexttile(layout);
hold(ax_error, 'on');
grid(ax_error, 'on');
plot(ax_error, time, trace.lateral_error, 'm-', 'LineWidth', 1.0, ...
    'DisplayName', 'Lateral error');
if ismember('track_margin_effective_margin', trace.Properties.VariableNames)
    plot(ax_error, time, trace.track_margin_effective_margin, 'k-', ...
        'LineWidth', 0.8, 'DisplayName', 'Track margin');
end
yline(ax_error, 0, 'k--', 'HandleVisibility', 'off');
xlabel(ax_error, 'Time [s]');
ylabel(ax_error, 'm');
title(ax_error, sprintf('Max |lat err| %.2f m', metrics.max_lateral_error));
legend(ax_error, 'Location', 'best');
style_axis(ax_speed);
style_axis(ax_error);
style_figure(fig);

exportgraphics(fig, png_path, 'Resolution', 180);
close(fig);
end

function make_replay_video(trace, geometry, metrics, config, video_path)
duration = max(double(trace.time(end) - trace.time(1)), eps);
n_frames = ceil(duration * config.video_fps / config.video_speedup);
n_frames = min(max(n_frames, 2), config.max_video_frames);
frame_times = linspace(double(trace.time(1)), double(trace.time(end)), n_frames);

fig = figure('Name', 'RL Autocross Replay', ...
    'Position', [80 80 1280 720], 'Color', 'w', ...
    'Visible', figure_visibility());
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
axis(ax, 'equal');
plot(ax, geometry.left_x, geometry.left_y, '-', 'Color', [0.12 0.12 0.12], 'LineWidth', 1.1);
plot(ax, geometry.right_x, geometry.right_y, '-', 'Color', [0.12 0.12 0.12], 'LineWidth', 1.1);
plot(ax, geometry.x, geometry.y, '--', 'Color', [0.7 0.7 0.7], 'LineWidth', 0.7);
plot(ax, trace.x(1), trace.y(1), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 7);
plot(ax, trace.x(end), trace.y(end), 'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 7);
path_line = plot(ax, NaN, NaN, '-', 'Color', [0 0.25 0.95], 'LineWidth', 2.0);
car_marker = plot(ax, NaN, NaN, 'o', 'MarkerFaceColor', [0.05 0.35 1.0], ...
    'MarkerEdgeColor', 'w', 'MarkerSize', 9, 'LineWidth', 1.0);
heading_arrow = quiver(ax, NaN, NaN, NaN, NaN, 0, 'Color', [0.85 0.05 0.0], ...
    'LineWidth', 1.6, 'MaxHeadSize', 0.9);
hud = text(ax, 0.02, 0.98, '', 'Units', 'normalized', ...
    'VerticalAlignment', 'top', 'FontName', 'Menlo', 'FontSize', 11, ...
    'BackgroundColor', [1 1 1], 'Margin', 6);
title(ax, path_title(config, metrics));
xlabel(ax, 'X [m]');
ylabel(ax, 'Y [m]');
pad_axis(ax, [geometry.x; trace.x], [geometry.y; trace.y]);
style_axis(ax);
style_figure(fig);

writer = VideoWriter(video_path, 'MPEG-4');
writer.FrameRate = config.video_fps;
writer.Quality = config.video_quality;
open(writer);
cleanup = onCleanup(@() close(writer));

trace_time = double(trace.time(:));
for i = 1:n_frames
    [~, idx] = min(abs(trace_time - frame_times(i)));
    set(path_line, 'XData', trace.x(1:idx), 'YData', trace.y(1:idx));
    set(car_marker, 'XData', trace.x(idx), 'YData', trace.y(idx));
    set(heading_arrow, 'XData', trace.x(idx), 'YData', trace.y(idx), ...
        'UData', 4.0 * cos(trace.psi(idx)), 'VData', 4.0 * sin(trace.psi(idx)));
    set(hud, 'String', sprintf(['t = %6.2f s / %.2f s\n' ...
        'speed = %5.2f m/s\nsteer torque = %6.2f Nm\n' ...
        'throttle = %.2f, brake = %.2f\nlat error = %+5.2f m'], ...
        trace_time(idx) - trace_time(1), metrics.lap_time, trace.speed(idx), ...
        trace.steering(idx), trace.throttle(idx), trace.brake(idx), ...
        trace.lateral_error(idx)));
    drawnow;
    writeVideo(writer, getframe(fig));
end
close(fig);
end

function write_summary(path_in, metrics, config, png_path, video_path)
fid = fopen(path_in, 'w');
if fid == -1
    error('visualizeRLAutocrossTrace:WriteFailed', 'Failed to open %s.', path_in);
end
cleanup = onCleanup(@() fclose(fid));
fprintf(fid, 'RL autocross trajectory visualization\n');
fprintf(fid, 'trace_csv_path: %s\n', config.trace_csv_path);
fprintf(fid, 'track_csv_path: %s\n', config.track_csv_path);
fprintf(fid, 'episode: %.0f\n', config.episode);
fprintf(fid, 'lap_time: %.3f s\n', metrics.lap_time);
fprintf(fid, 'distance_traveled: %.3f m\n', metrics.distance_traveled);
fprintf(fid, 'max_speed: %.3f m/s\n', metrics.max_speed);
fprintf(fid, 'avg_speed: %.3f m/s\n', metrics.avg_speed);
fprintf(fid, 'max_lateral_error: %.3f m\n', metrics.max_lateral_error);
fprintf(fid, 'max_track_limit_excess: %.3f m\n', metrics.max_track_limit_excess);
fprintf(fid, 'max_abs_tire_steer: %.3f deg\n', metrics.max_abs_tire_steer_deg);
fprintf(fid, 'max_abs_steering_wheel: %.3f deg\n', metrics.max_abs_sw_deg);
fprintf(fid, 'png_path: %s\n', png_path);
fprintf(fid, 'video_path: %s\n', video_path);
end

function text_out = path_title(config, metrics)
if isfinite(config.episode)
    agent_text = sprintf('Agent%.0f', config.episode);
else
    agent_text = 'RL agent';
end
lap_time = metrics.lap_time;
if isfinite(config.lap_time)
    lap_time = config.lap_time;
end
text_out = sprintf('%s - %s, lap %.3f s', config.title, agent_text, lap_time);
end

function pad_axis(ax, x, y)
margin = 0.06;
x_min = min(x);
x_max = max(x);
y_min = min(y);
y_max = max(y);
dx = max(x_max - x_min, 1);
dy = max(y_max - y_min, 1);
xlim(ax, [x_min - margin * dx, x_max + margin * dx]);
ylim(ax, [y_min - margin * dy, y_max + margin * dy]);
end

function style_axis(ax)
set(ax, 'Color', [1 1 1], 'XColor', [0 0 0], 'YColor', [0 0 0], ...
    'GridColor', [0.75 0.75 0.75], 'MinorGridColor', [0.86 0.86 0.86]);
end

function style_figure(fig)
set(fig, 'Color', [1 1 1]);
set(findall(fig, 'Type', 'Text'), 'Color', [0 0 0]);
legends = findall(fig, 'Type', 'Legend');
for i = 1:numel(legends)
    set(legends(i), 'Color', [1 1 1], 'TextColor', [0 0 0], ...
        'EdgeColor', [0.35 0.35 0.35]);
end
colorbars = findall(fig, 'Type', 'ColorBar');
for i = 1:numel(colorbars)
    set(colorbars(i), 'Color', [0 0 0]);
end
end

function visibility = figure_visibility()
if usejava('desktop')
    visibility = 'on';
else
    visibility = 'off';
end
end

function ensure_dir(path_in)
if ~exist(path_in, 'dir')
    mkdir(path_in);
end
end

function path_out = resolve_project_path(project_root, path_in)
path_in = char(string(path_in));
if startsWith(path_in, filesep) || ~isempty(regexp(path_in, '^[A-Za-z]:[\\/]', 'once'))
    path_out = path_in;
else
    path_out = fullfile(project_root, path_in);
end
end
