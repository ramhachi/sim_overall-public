# `matlab/pou_hardpoint_design/verify_installation.m`

## 1. 目的

POU ハードポイントシステムの導入確認スクリプトです。パス、必須ファイル、Optimization Toolbox、主要関数呼び出し、入力 JSON の有無を順番に点検します。

## 2. 入出力と依存関係

- 形式: スクリプト
- 入力:
  - カレントディレクトリ
  - 必須ファイル群
  - `../optimized_autocross.json`
- 出力: 各チェック結果のコンソール表示
- 依存関係:
  - `define_suspension_geometry`
  - `axis_angle_rotation`
  - `solve_upright_position_3d`
  - `forward_kinematics`
  - `objective_polynomial_fit`
  - `constraint_rod_misalignment`
  - `validate_packaging`
  - `load_vehicle_artifact`
  - `license('test', 'Optimization_Toolbox')`

## 3. 全体処理フロー

1. 自身の親ディレクトリを `addpath` します。
2. 現在位置に `pou_hardpoint_design` が含まれているか確認します。
3. 必須 `.m` ファイルの存在を列挙チェックします。
4. Optimization Toolbox のライセンスを確認します。
5. 代表関数を固定入力で順に呼び、例外が出ないか試します。
6. `../optimized_autocross.json` を見つけた場合だけ `load_vehicle_artifact` で読込み、POU パラメータ有無を見ます。
7. 最後に次の実行ステップと関連ドキュメント名を表示します。

## 4. ブロック詳細

- **Check 1: Path Setup**  
  実行場所が推奨ディレクトリと違っても警告するだけで、処理自体は継続します。

- **Check 2: Function Files Exist**  
  列挙された `required_files` のいずれかが欠けると「Installation incomplete」として即 `return` します。

- **Check 3: MATLAB Toolboxes**  
  `fmincon` に必須な Optimization Toolbox の存在確認です。無い場合はここで終了します。

- **Check 4: Function Calls**  
  主要関数へ簡単な入力を与えて smoke test を行います。数値の妥当性までは深く検証せず、呼び出し成功を主に見ています。

- **Check 5: Input File**  
  JSON ファイルがあれば読みますが、無い場合も「最適化時にパスを与える必要がある」と案内して終了します。

- **末尾の案内文**  
  `QUICK_START.md` や `README.md` などのファイル名を出しますが、このスクリプト自体はそれらの存在確認をしません。

- **ローカル関数**  
  このファイルにローカル関数はありません。
