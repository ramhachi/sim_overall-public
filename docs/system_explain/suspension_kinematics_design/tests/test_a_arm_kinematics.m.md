# tests/test_a_arm_kinematics.m

## 概要
- `test_a_arm_kinematics` は、`a_arm_kinematics_solver` の基本幾何計算を確認するユニットテスト関数です。
- 平行アーム時のロールセンターと KPI、非平行アーム時の瞬間中心位置を簡易に検証します。
- `matlab.unittest` は使わず、`assert` による軽量な自前テストとして書かれています。

## 関数シグネチャ
```matlab
function test_a_arm_kinematics
```

## 入力
- 引数はありません。
- テスト用ハードポイント構造体を関数内部で直接組み立てます。

## 出力
- 戻り値はありません。
- 進捗は `disp` で表示され、失敗時は `assert` 例外が送出されます。

## 実行フロー

### 1. 開始表示
- `disp('Running A-Arm Kinematics Tests...');` を表示します。

### 2. テストケース 1: 平行・水平アーム
- `hp_parallel` 構造体を作成し、上下アームを完全に水平・平行に配置します。
- `is_left = true`、`R_tire = 0.25`、`W_C = [0; 0.6; 0.25]` を設定します。
- `a_arm_kinematics_solver(hp_parallel)` の結果に対し、以下を検証します。
  - `roll_center_z` が 0 に近い
  - `IC_y` が実質無限遠
  - `kpi_deg` が 0 に近い
- 平行リンク特例分岐が破綻していないかを見るテストです。

### 3. テストケース 2: 非平行アーム
- `hp_angle = hp_parallel` をベースに、上側シャシー点を高く、下側シャシー点を低く変更します。
- これにより車体内側で交わるフロントビュー幾何を作ります。
- `a_arm_kinematics_solver(hp_angle)` 実行後、`IC_y` が `0.5 < IC_y < 1.0` に入ることを確認します。
- ロールセンター高さは表示のみで、厳密な `assert` 判定はしていません。

### 4. 完了表示
- すべての `assert` を通過すると `All tests passed successfully!` を表示して終了します。

## 関数本体の責務
- ソルバの平行アーム分岐と通常分岐を最低限カバーすること
- 幾何ロジックの大きな破綻を早期検出すること

## 依存関係
- `a_arm_kinematics_solver`
- `disp`, `assert`, `num2str`

## 前提・注意点
- 呼び出し前に `a_arm_kinematics_solver` が MATLAB パス上にある必要があります。
- `W_C` の Z が正値で、本体コードの Z-down 慣例とは異なるテスト形状です。
- キャンバー、スクラブ半径、左右輪反転などは検証していません。
