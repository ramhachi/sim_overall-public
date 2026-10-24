function targets = build_inverse_targets(params, axleKey)
% BUILD_INVERSE_TARGETS Build runtime-driven inverse-analysis targets from artifact params.

isFront = strcmpi(axleKey, 'front');
if isFront
    wheelIndex = 1;
    halfTrack = max(params.mass.track_f, 0.0) / 2;
    defaultRC = -0.035;
    defaultKPI = 4.0;
    defaultCaster = 3.0;
    staticToeDefault = params.suspension.static_toe(1);
    camberSteerDefault = [0.0, params.suspension.pou_camber_c1(1), params.suspension.pou_camber_c2(1)];
else
    wheelIndex = 3;
    halfTrack = max(params.mass.track_r, 0.0) / 2;
    defaultRC = -0.025;
    defaultKPI = 1.5;
    defaultCaster = 0.0;
    staticToeDefault = params.suspension.static_toe(3);
    camberSteerDefault = [0.0, 0.0, 0.0];
end

targets = struct();
targets.static_camber = params.suspension.static_camber(wheelIndex);
targets.static_toe = staticToeDefault;
targets.roll_center_z = defaultRC;
targets.kpi_deg = defaultKPI;
targets.caster_deg = defaultCaster;
targets.scrub_radius_m = 0.0;
targets.mechanical_trail_m = 0.0;
targets.camber_vs_travel = [params.suspension.static_camber(wheelIndex), params.suspension.camber_gain(wheelIndex), 0.0];
targets.toe_vs_travel = [staticToeDefault, 0.0, 0.0];
targets.camber_vs_steer = camberSteerDefault;
targets.toe_vs_steer = [0.0, 0.0, 0.0];
targets.roll_camber_gain = 0.0;
targets.roll_steer_gain = 0.0;
targets.roll_center_height_per_travel = 0.0;
targets.roll_center_height_per_roll = 0.0;
targets.jacking_force_per_fy = 0.0;
targets.anti_brake_pct = 0.0;
targets.anti_drive_pct = 0.0;
targets.anti_lift_pct = 0.0;
targets.anti_rise_pct = 0.0;
targets.toe_per_load = zeros(1, 6);
targets.camber_per_load = zeros(1, 6);
targets.caster_per_load = zeros(1, 6);
targets.track_per_load = zeros(1, 6);
targets.wheelbase_per_load = zeros(1, 6);

sources = struct();
sources.static_camber = 'vehicle_artifact.params.suspension.static_camber';
sources.static_toe = 'vehicle_artifact.params.suspension.static_toe';
sources.roll_center_z = 'fallback_default_requires_design_target';
sources.kpi_deg = 'fallback_default_requires_design_target';
sources.caster_deg = 'fallback_default_requires_design_target';
sources.scrub_radius_m = 'fallback_zero_requires_design_target';
sources.mechanical_trail_m = 'fallback_zero_requires_design_target';
sources.camber_vs_travel = 'vehicle_artifact.params.suspension.camber_gain';
sources.toe_vs_travel = 'fallback_static_toe_zero_gradient';
sources.camber_vs_steer = 'vehicle_artifact.params.suspension.pou_camber_coefficients_reference_only';
sources.toe_vs_steer = 'fallback_zero_requires_design_target';
sources.roll_camber_gain = 'fallback_zero_requires_design_target';
sources.roll_steer_gain = 'fallback_zero_requires_design_target';
sources.roll_center_height_per_travel = 'fallback_zero_requires_design_target';
sources.roll_center_height_per_roll = 'fallback_zero_requires_design_target';
sources.jacking_force_per_fy = 'fallback_zero_proxy_requires_validation';
sources.anti_brake_pct = 'fallback_zero_proxy_requires_validation';
sources.anti_drive_pct = 'fallback_zero_proxy_requires_validation';
sources.anti_lift_pct = 'fallback_zero_proxy_requires_validation';
sources.anti_rise_pct = 'fallback_zero_proxy_requires_validation';
sources.compliance = 'fallback_zero_proxy_requires_validation';
sources.toe_per_load = sources.compliance;
sources.camber_per_load = sources.compliance;
sources.caster_per_load = sources.compliance;
sources.track_per_load = sources.compliance;
sources.wheelbase_per_load = sources.compliance;

% Explicit design targets are the only sanctioned override source. The
% previously written kc block is not read back as a target: doing so made
% the optimizer converge on its own last output.
if isfield(params.suspension, 'design_targets') && isfield(params.suspension.design_targets, axleKey)
    designTargets = params.suspension.design_targets.(axleKey);
    fields = fieldnames(designTargets);
    for i = 1:numel(fields)
        name = fields{i};
        if isfield(targets, name)
            targets.(name) = designTargets.(name);
            sources.(name) = ['vehicle_artifact.params.suspension.design_targets.' axleKey '.' name];
        end
    end
end

% Derived targets keep the objective self-consistent: roll camber gain is
% determined by camber gain and half track, jacking by roll-center height.
if strncmp(sources.roll_camber_gain, 'fallback', 8)
    targets.roll_camber_gain = -targets.camber_vs_travel(2) * max(halfTrack, 1e-6);
    sources.roll_camber_gain = 'derived_from_camber_gain_and_half_track';
end
if strncmp(sources.jacking_force_per_fy, 'fallback', 8)
    targets.jacking_force_per_fy = targets.roll_center_z / max(halfTrack, 1e-6);
    sources.jacking_force_per_fy = 'derived_from_roll_center_proxy';
end

tolerances = struct();
tolerances.roll_center_z = 0.010;
tolerances.kpi_deg = 0.5;
tolerances.caster_deg = 0.5;
tolerances.scrub_radius_m = 0.005;
tolerances.mechanical_trail_m = 0.005;
tolerances.camber_gain_rad_m = 0.10;
tolerances.roll_camber_gain = 0.05;
tolerances.roll_center_height_per_travel = 0.02;
tolerances.roll_center_height_per_roll = 0.02;
tolerances.jacking_force_per_fy = 0.05;
tolerances.anti_pct = 0.05;
tolerances.compliance = 1e-6;

weights = struct();
weights.roll_center_z = 1.0;
weights.kpi = 1.0;
weights.caster = 1.0;
weights.scrub = 1.0;
weights.mechanical_trail = 1.0;
weights.camber_gain = 1.0;
weights.roll_camber_gain = 1.0;
weights.roll_center_migration = 1.0;
weights.jacking = 1.0;
weights.anti = 1.0;
weights.compliance = 1.0;
if isfield(params.suspension, 'objective_weights') && isstruct(params.suspension.objective_weights)
    override_weights = fieldnames(params.suspension.objective_weights);
    for i = 1:numel(override_weights)
        name = override_weights{i};
        if isfield(weights, name)
            weights.(name) = params.suspension.objective_weights.(name);
        end
    end
end
% Fallback/proxy targets are provisional. They must not silently steer the
% geometry with the same authority as measured or explicit design targets.
fallback_fields = {};
source_for_weight = struct( ...
    'roll_center_z', 'roll_center_z', ...
    'kpi', 'kpi_deg', ...
    'caster', 'caster_deg', ...
    'scrub', 'scrub_radius_m', ...
    'mechanical_trail', 'mechanical_trail_m', ...
    'camber_gain', 'camber_vs_travel', ...
    'roll_camber_gain', 'roll_camber_gain', ...
    'roll_center_migration', 'roll_center_height_per_travel', ...
    'jacking', 'jacking_force_per_fy', ...
    'anti', 'anti_brake_pct', ...
    'compliance', 'compliance');
weight_fields = fieldnames(weights);
for i = 1:numel(weight_fields)
    name = weight_fields{i};
    source_name = name;
    if isfield(source_for_weight, name)
        source_name = source_for_weight.(name);
    end
    if isfield(sources, source_name) && (strncmp(sources.(source_name), 'fallback', 8) || contains(sources.(source_name), 'proxy'))
        weights.(name) = 0.0;
        fallback_fields{end + 1} = name; %#ok<AGROW>
    end
end
% Jacking is derived from roll-center height; disable it when RC is fallback.
if isfield(sources, 'roll_center_z') && strncmp(sources.roll_center_z, 'fallback', 8)
    weights.jacking = 0.0;
    fallback_fields{end + 1} = 'jacking'; %#ok<AGROW>
end
source_quality = struct();
target_fields = fieldnames(targets);
for i = 1:numel(target_fields)
    name = target_fields{i};
    source_name = name;
    if isfield(source_for_weight, name)
        source_name = source_for_weight.(name);
    end
    if isfield(sources, source_name)
        [kind, confidence] = classify_source(sources.(source_name));
        source_quality.(name) = struct( ...
            'source', sources.(source_name), ...
            'kind', kind, ...
            'confidence', confidence, ...
            'provisional', ~ismember(kind, {'measured', 'design_target'}), ...
            'required', false);
    end
end
for i = 1:numel(weight_fields)
    name = weight_fields{i};
    target_name = name;
    if isfield(source_for_weight, name)
        target_name = source_for_weight.(name);
    end
    if isfield(source_quality, target_name)
        source_quality.(target_name).required = weights.(name) > 0.0;
    end
end
if isfield(source_quality, 'roll_camber_gain') && isfield(source_quality, 'camber_vs_travel')
    source_quality.roll_camber_gain.provisional = source_quality.camber_vs_travel.provisional;
end
if isfield(source_quality, 'jacking_force_per_fy') && isfield(source_quality, 'roll_center_z')
    source_quality.jacking_force_per_fy.provisional = source_quality.roll_center_z.provisional;
end

targets.tolerances = tolerances;
targets.weights = weights;
targets.weight_policy = 'tolerance_normalized';
targets.sources = sources;
targets.source_quality = source_quality;
targets.fallback_fields = fallback_fields;
targets.provisional = ~isempty(fallback_fields);
targets.quality = target_quality_summary(source_quality, fallback_fields);
targets.model_version = 'suspension_inverse_targets/v1';
targets.model_scope = struct( ...
    'inverse_target_contract', 'Targets are runtime-driven. Any source containing fallback_* is a provisional design assumption, not measured K&C data. Optimizer-written kc values are never read back as targets.', ...
    'compliance_model', 'A-arm compliance terms are proxy metrics unless a corner.compliance source is present.');
end

function [kind, confidence] = classify_source(source)
source = char(source);
if strncmp(source, 'fallback', 8)
    kind = 'fallback';
    confidence = 0.0;
elseif contains(source, 'proxy') || contains(source, 'reference_only')
    kind = 'proxy';
    confidence = 0.2;
elseif strncmp(source, 'derived', 7)
    kind = 'derived';
    confidence = 0.6;
elseif contains(source, 'design_targets')
    kind = 'design_target';
    confidence = 0.8;
elseif contains(source, 'vehicle_artifact')
    kind = 'design_target';
    confidence = 0.8;
else
    kind = 'fallback';
    confidence = 0.0;
end
end

function quality = target_quality_summary(source_quality, fallback_fields)
counts = struct('measured', 0, 'design_target', 0, 'derived', 0, 'fallback', 0, 'proxy', 0);
proxy_fields = {};
fields = fieldnames(source_quality);
for i = 1:numel(fields)
    name = fields{i};
    kind = source_quality.(name).kind;
    if isfield(counts, kind)
        counts.(kind) = counts.(kind) + 1;
    end
    if strcmp(kind, 'proxy')
        proxy_fields{end + 1} = name; %#ok<AGROW>
    end
end
if counts.fallback > 0 || counts.proxy > 0
    confidence = 'low';
elseif counts.derived > 0
    confidence = 'medium';
else
    confidence = 'high';
end
if strcmp(confidence, 'high')
    recommendation = 'Trusted target set; still verify with closed-loop/design review.';
else
    recommendation = 'Do not use for final design claims until fallback/proxy targets are replaced.';
end
quality = struct( ...
    'source_counts', counts, ...
    'fallback_fields', {fallback_fields}, ...
    'proxy_fields', {proxy_fields}, ...
    'confidence', confidence, ...
    'do_not_use_for_final_design', ~strcmp(confidence, 'high'), ...
    'recommendation', recommendation);
end
