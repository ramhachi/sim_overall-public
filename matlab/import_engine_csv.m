% import_engine_csv.m
% ユーザー提供のCSVからエンジントルクマップを作成し、構造体として保存する

clear; clc;

% CSV読み込み (ヘッダーなし、Space区切りと仮定)
filename = 'test_04.CSV';
opts = detectImportOptions(filename);
opts.DataLines = [2, Inf]; % 1行目はヘッダーっぽいゴミラベルがあるかも?
opts.VariableNamesLine = 0; 
% 実際にはヘッダーがない、もしくはテキストヘッダーがある模様。
% readmatrixで数値のみ読み込むのが安全。

raw_data = readmatrix(filename);

% データ抽出
% Column 1: RPM
% Column 8: Torque [Kgm] -> [Nm] Wait, unit check.
% User CSV header says "RPM", "Kgm". 
% 47.46 Kgm? 
% 1 Kgm = 9.8 Nm. 
% 47.46 Kgm * 9.8 = 465 Nm! That is WAY too high for 600cc.
% CBR600RR peak is ~60 Nm.
%
% Let's look at the numbers again.
% 11000 rpm -> 63.58.
% If Unit is Nm, 63.58 Nm is perfect for FSAE.
% If Unit is Kgm, 63.58 Kgm = 623 Nm. Impossible.
%
% BUT, the header in `head` output shows:
% "RPM", "Kgm"
%
% Maybe it's NOT Kgm? Or maybe it's scaled?
% Let's look at the "Power" column (Col 8 matches Torque in previous layout?).
% Col 8: 63.58.
% Col 10: 0.5598.
%
% Let's calculate Power check.
% Power(kW) = Torque(Nm) * RPM * 2pi/60 / 1000
% Power(PS) = Power(kW) * 1.36
%
% At 11000 rpm, Torque 63.58.
% If Nm: P = 63.58 * 11000 * 2pi/60 / 1000 = 73 kW = 99 PS.
% This matches CBR600RR (stock is ~100-110PS).
%
% So the unit is **Nm**, despite the header saying "Kgm" (or header is misaligned).
% Wait, header line 2 says `"RPM","Kgm","%","KPa",...`
% Maybe "Torque" is Col 2? "4.139"?
% 4.139 Kgm * 9.8 = 40.5 Nm.
% At 11000 rpm, 40Nm is low (50-60 is typical).
%
% Let's check Power column. The header suggests "Power" and "PS"?
% Line 1: "Power", "La1:", ...
% Line 2: "PS", "AFR", ...
%
% Data at 11000:
% Col 8: 63.58. This could be Power (PS)??
% If 63.58 PS @ 11000rpm. 
% Torque = Power(W) / omega = (63.58/1.36 * 1000) / (11000 * 2pi/60)
% = 46750 / 1151 = 40.6 Nm.
%
% This matches Col 2 value "4.139" Kgm (4.139 * 9.80665 = 40.59 Nm).
%
% **HYPOTHESIS**:
% Col 2 = Torque (Kgm)
% Col 8 = Power (PS)
%
% Let's verify with 8300 rpm data.
% Col 2: 4.095 Kgm -> 40.15 Nm.
% Col 8: 47.46 PS.
% Power from Torque: 40.15 * 8300 * 2pi/60 / 1000 * 1.36 
% = 34.9 kW * 1.36 = 47.46 PS !!
%
% **MATCHED!**
% Col 2 is Torque in Kgm.
% Col 8 is Power in PS.
%
% So Torque(Nm) = Col 2 * 9.80665.
% Peak Torque seems to be ~4.3 Kgm (~42 Nm).
% Wait, 42 Nm is LOW for a 600cc. Restricted?
% Yes, FSAE restrictor (20mm) limits power/torque significantly. 
% Stock CBR is 60Nm. Restricted is usually 40-55Nm. 
% So 42Nm is a plausible, albeit slightly low, restricted engine.
% Or maybe it's a 450cc?
%
% Anyway, rely on Col 2 * 9.80665.
%
% Data starts at 7000 rpm.
% 0-7000 needs filling.
% At 7000, T = 1.485 Kgm = 14.5 Nm. (Very low)
% Linear interpolation from 0 to 14.5?
%
% Wait, 7000 is 14Nm, 8300 is 40Nm.
% This steep rise suggests 7000 is off-cam or throttle opening.
% I will linearly interpolate from Idle (1500, ~15Nm) to (7000, 14.5Nm) to be safe?
% Actually, if 7000 is 14Nm, and 8300 is 40Nm, the "useful" range starts > 7500.
% I'll assume linear ramp from 3000rpm (10Nm) to 7000 (14.5Nm).
%
% Final Data Structure:
% RPM: [1000, 3000, 7000, ...]
% T:   [10,   12,   14.5, ...]
% 
% Let's implement.

% Load Data
data = raw_data;
rpm_raw = data(:, 1);
trq_kgm_raw = data(:, 2);
trq_nm_raw = trq_kgm_raw * 9.80665;

% Filter valid RPM (Remove zeros or weird drops)
valid_idx = rpm_raw > 100 & trq_nm_raw > 0;
rpm_valid = rpm_raw(valid_idx);
trq_valid = trq_nm_raw(valid_idx);

% Sort
[rpm_valid, sort_idx] = sort(rpm_valid);
trq_valid = trq_valid(sort_idx);

% Create Full Curve
rpm_target = [1000; 3000; 5000; rpm_valid];
% Low RPM Estimation:
trq_target = [15.0; 20.0; 25.0; trq_valid]; % Slightly higher low-end for drivability

% Extrapolate High RPM (Redline Drop-off)
% Last point is ~12000 rpm. We need up to 14000.
% Assume torque drops to ~30Nm at 14000 to limit power.
if max(rpm_target) < 13500
    rpm_target = [rpm_target; 14000];
    trq_target = [trq_target; 30.0];
end

% Remove duplicate RPMs
[rpm_unique, unique_idx] = unique(rpm_target);
trq_unique = trq_target(unique_idx);

% Print for ConceptTradeOffs.m
fprintf('\n%% --- Paste this into ConceptTradeOffs.m ---\n');
fprintf('rpm_data = [');
fprintf('%.0f, ', rpm_unique(1:end-1));
fprintf('%.0f];\n', rpm_unique(end));

fprintf('torque_data = [');
fprintf('%.1f, ', trq_unique(1:end-1));
fprintf('%.1f];\n', trq_unique(end));
fprintf('%% ------------------------------------------\n');

save('custom_engine_curve.mat', 'engine_data');

% Plot for verification (visible in workspace if run locally, but here just save params)
disp('Peak Torque:');
disp(max(trq_unique));

% Save to .mat for easy loading
engine_data = struct();
engine_data.rpm = rpm_unique;
engine_data.torque = trq_unique;
engine_data.source = 'test_04.CSV';

save('custom_engine_curve.mat', 'engine_data');

% Update ConceptTradeOffs.m automatically?
% Better to just update the config file to load this .mat or hardcode the values if small.
% I will generate a JSON string to copy-paste or write to a file.

% Output for user check
fprintf('Imported %d points.\n', length(rpm_unique));
fprintf('Range: %.0f - %.0f rpm\n', min(rpm_unique), max(rpm_unique));
