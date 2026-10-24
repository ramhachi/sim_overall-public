# tests/test_rear_a_arm_optimizer.m

## 概要
- `test_rear_a_arm_optimizer` は、`main_rear_a_arm_optimizer` の統合テスト関数です。
- リア最適化が完走すること、`suspension_base_geometry.json` に `RL` と `RR` が書き込まれること、主要値が許容範囲に入ることを確認します。
- 既存 JSON を壊しにくくするため、バックアップと失敗時復元の処理を含みます。

## 関数シグネチャ
```matlab
function test_rear_a_arm_optimizer
```

## 入力
- 引数はありません。
- 対象ファイルパスは関数内で自動解決されます。

## 出力
- 戻り値はありません。
- `fprintf` で各テストの進捗を表示します。
- 失敗時は `assert` または再送出された例外で停止します。

## 実行フロー

### 1. バナー表示
- テスト名付きの区切り線を表示し、実行開始を明示します。

### 2. パス設定
- `this_dir = fileparts(mfilename('fullpath'));` で `tests` ディレクトリを取得します。
- `root_dir = fileparts(this_dir);` で `suspension_kinematics_design` 本体ディレクトリを取得します。
- `addpath(root_dir);` で `main_rear_a_arm_optimizer` と `a_arm_kinematics_solver` を参照可能にします。
- `json_file` は `../matlab/optimized_autocross.json`、`out_json` は `suspension_base_geometry.json` へ解決されます。

### 3. 前提ファイル確認
- `optimized_autocross.json` が無い場合は `warning` を出し、そのまま `return` します。
- そのため、このケースは失敗ではなくスキップ扱いです。

### 4. バックアップ作成
- 既存の `suspension_base_geometry.json` があれば、同じ場所に `.bak` 拡張子付きでコピーします。
- 後続の失敗時復元と成功後後始末でこのバックアップを使います。

### 5. `try` ブロック内テスト

#### Test 1: 最適化が正常完了するか
- `orig_dir = pwd` で元ディレクトリを保存します。
- `cd(root_dir)` 後に `main_rear_a_arm_optimizer(json_file)` を実行します。
- 実行後は `cd(orig_dir)` で元へ戻します。
- ここでは「例外が出ないこと」だけを確認します。

#### Test 2: `RL` フィールド確認
- `assert(isfile(out_json), ...)` で JSON 出力の存在を確認します。
- `jsondecode(fileread(out_json))` で読み込み、`isfield(data, 'RL')` を検証します。

#### Test 3: `RR` フィールド確認
- 同じ `data` に対して `isfield(data, 'RR')` を検証します。

#### Test 4: `RL` ロールセンター範囲確認
- `target_RC = -0.050`、`tolerance = 0.015` を設定します。
- `data.RL.roll_center_z` が目標値から ±15 mm に入ることを確認します。
- ただし本体コードの固定目標は `-0.025` なので、ここは実装目標そのものではなく経験的な妥当範囲チェックです。

#### Test 5: `RL` KPI 範囲確認
- `target_KPI = 2.0`、`kpi_tol = 2.0` を設定します。
- `data.RL.kpi_deg` が許容範囲に入るか確認します。
- 本体コードの固定目標 `1.5 deg` とは完全一致していません。

#### Test 6: `FL` 保持確認
- `isfield(data, 'FL')` なら PASS を表示します。
- 無ければ INFO を表示するだけで、失敗にはしません。

### 6. 例外処理
- `catch ME` ではまず `cd(orig_dir)` で元ディレクトリへ戻します。
- バックアップがあれば `copyfile(backup_file, out_json)` で復元します。
- その後エラーメッセージを表示し、`rethrow(ME)` で元の例外を再送出します。
- `orig_dir` は Test 1 直前で代入されるため、それ以前に例外が出るケースでは未定義になり得ます。

### 7. 後片付け
- 正常終了時は `.bak` があれば削除します。
- 失敗時は `catch` で再送出するため、通常この削除処理には到達しません。

## 関数本体の責務
- リア最適化の実行可否確認
- JSON 追記結果の存在確認
- 主要な幾何値の範囲検証
- 既存成果物の保全

## 依存関係
- `main_rear_a_arm_optimizer`
- `addpath`, `fileparts`, `fullfile`, `pwd`, `cd`
- `copyfile`, `delete`, `isfile`, `fileread`, `jsondecode`, `isfield`
- `fprintf`, `warning`, `assert`, `sprintf`, `rethrow`

## 前提・注意点
- 実行ディレクトリを一時的に変更するため、復帰処理が重要です。
- テスト期待値と本体コードの固定目標値には差があります。
- 最適化ツールボックスや入力 JSON が無い環境ではスキップまたは失敗します。
