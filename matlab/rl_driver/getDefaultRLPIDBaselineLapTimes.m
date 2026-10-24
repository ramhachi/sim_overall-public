function baselines = getDefaultRLPIDBaselineLapTimes()
% GETDEFAULTRLPIDBASELINELAPTIMES Default PID lap-time baselines for RL gating.
%
% Values are sourced from checked-in simulator reports for the default
% vehicle/configuration:
% - circle:    report_circle_20260216_202544.txt   -> 15.684 s
% - skidpad:   report_skidpad_20260302_010340.txt  -> 13.014 s
% - autocross: report_ASE_autocross_20260221_161143.txt -> 60.010 s

baselines = struct( ...
    'circle', 15.684, ...
    'skidpad', 13.014, ...
    'autocross', 60.010);
end
