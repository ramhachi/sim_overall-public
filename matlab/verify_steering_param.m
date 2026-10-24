% Verify steering parameter passing
% 1. Create a dummy design vector with a specific steering stiffness
x = zeros(30, 1);
x(1) = 250; % Mass
x(2) = 0.5; % Front Dist
x(30) = 5555; % Steering Stiffness (Unique value for verification)

% 2. Apply design
base_params = struct();
base_params.mass = struct('m_unsprung', [12, 12, 14, 14], 'wheelbase', 1.55);
base_params.aero = struct();
base_params.powertrain = struct();
base_params.suspension = struct();
% The wrapper adds .steering if missing, but we should verify that behavior too

params = apply_full_design(base_params, x);

% 3. Verify struct field
if isfield(params, 'steering') && isfield(params.steering, 'stiffness')
    val = params.steering.stiffness;
    fprintf('Struct check: params.steering.stiffness = %.2f\n', val);
    if val == 5555
        fprintf('  -> OK: Value matches input.\n');
    else
        fprintf('  -> FAIL: Value mismatch.\n');
    end
else
    fprintf('Struct check: FAIL - params.steering.stiffness missing.\n');
end

% 4. Verify JSON encoding
json_str = jsonencode(params);
fprintf('JSON output snippet for steering:\n');
disp(json_str);

% Check if the JSON contains strict match for stiffness
if contains(json_str, '"stiffness":5555')
    fprintf('JSON check: OK - "stiffness":5555 found in JSON.\n');
else
    fprintf('JSON check: FAIL - "stiffness":5555 not found in JSON.\n');
end
