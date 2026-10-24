%% check_fz_dist.m - Fzの実際の値域を確認
data_dir = '../../data/tiredata/csv';
cr_paths = {fullfile(data_dir, 'Cr_Hoosier_18p0X6p0-10_R20_Run027.csv')};
cr = loadTireData(cr_paths);
fprintf('=== Cornering Fz 統計 ===\n');
fprintf('min=%.1f, max=%.1f, mean=%.1f, median=%.1f\n', min(cr.Fz), max(cr.Fz), mean(cr.Fz), median(cr.Fz));
fprintf('prctile: 5%%=%.1f, 25%%=%.1f, 50%%=%.1f, 75%%=%.1f, 95%%=%.1f\n', ...
    prctile(cr.Fz,[5 25 50 75 95]));
fprintf('\nFz histogram:\n');
edges = -2000:200:2000;
[N,~] = histcounts(cr.Fz, edges);
for k=1:length(N)
    if N(k)>0, fprintf('  [%5.0f, %5.0f): %d\n', edges(k), edges(k+1), N(k)); end
end
fprintf('\nSA: min=%.2f, max=%.2f\n', min(cr.SA), max(cr.SA));
fprintf('\nFirst 3 rows:\n');
disp(cr(1:3,:));
