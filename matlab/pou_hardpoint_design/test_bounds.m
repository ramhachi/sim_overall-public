% TEST_BOUNDS - 設計変数の境界チェック
%
% 各ホイールの境界が正しく設定されているか確認

clear; clc;

fprintf('=== 設計変数境界チェック ===\n\n');

% パスを追加
addpath('geometry');

wheel_names = {'FL', 'FR', 'RL', 'RR'};

for wheel_idx = 0:3
    fprintf('--- %s ホイール ---\n', wheel_names{wheel_idx + 1});

    % ジオメトリ定義を取得
    geom = define_suspension_geometry(wheel_idx);

    % 境界を構築
    lb = [geom.M_pivot_bounds.lower; geom.bellcrank_bounds.lower];
    ub = [geom.M_pivot_bounds.upper; geom.bellcrank_bounds.upper];

    % 初期値
    x0 = [geom.M_pivot_initial; geom.bellcrank_initial];

    % 変数名
    var_names = {'Xm (M-pivot X)', 'Ym (M-pivot Y)', 'Zm (M-pivot Z)', ...
                 'Xb (Bellcrank X)', 'Yb (Bellcrank Y)', 'Zb (Bellcrank Z)'};

    % チェック
    all_ok = true;
    for i = 1:6
        fprintf('  %s:\n', var_names{i});
        fprintf('    下限: %8.4f, 上限: %8.4f, 初期値: %8.4f\n', lb(i), ub(i), x0(i));

        if lb(i) >= ub(i)
            fprintf('    ✗ エラー: 下限 >= 上限\n');
            all_ok = false;
        elseif x0(i) < lb(i) || x0(i) > ub(i)
            fprintf('    ⚠ 警告: 初期値が境界外\n');
            all_ok = false;
        else
            fprintf('    ✓ OK\n');
        end
    end

    if all_ok
        fprintf('  総合: ✓ すべて正常\n\n');
    else
        fprintf('  総合: ✗ 問題あり\n\n');
    end
end

fprintf('=== チェック完了 ===\n');
