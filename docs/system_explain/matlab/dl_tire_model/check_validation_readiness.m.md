# `matlab/dl_tire_model/check_validation_readiness.m`

## 1. 概要

`check_validation_readiness` は、最終検証を走らせる前に、MATLAB バージョン・ツールボックス・GPU・データファイル・必須 M ファイル・出力ディレクトリを確認し、準備完了かどうかを論理値で返す関数です。

## 2. ファイルの役割

- 検証前の前提条件を 7 項目で点検します。
- 問題がある項目を `issues` に蓄積します。
- すべて通れば `true`、必須条件が不足していれば `false` を返します。
- `output` ディレクトリがなければ作成を試みます。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
ready = check_validation_readiness()
```

### 3.2 戻り値

- `ready`
  - 全必須条件が満たされている場合は `true`
  - 必須条件のどれかが欠ける場合は `false`

### 3.3 内部で確認する主対象

- MATLAB リリース (`version('-release')`)
- ライセンス
  - Deep Learning Toolbox
  - Statistics and Machine Learning Toolbox
  - Parallel Computing Toolbox（任意扱い）
- データファイル
  - `../../data/tiredata/DriveBrake_sampled_quarter.csv`
- 必須ファイル 15 件
- `output` ディレクトリ

## 4. 依存関係

- MATLAB 標準関数: `version`, `license`, `gpuDeviceCount`, `gpuDevice`, `isfile`, `exist`, `mkdir`, `system`, `fprintf`, `str2double`, `sprintf`
- このファイル自身は他のプロジェクト関数を呼ばず、ファイル存在確認のみ行います。

## 5. 処理の流れ

1. `ready = true` と `issues = {}` で開始します。
2. MATLAB リリース文字列の先頭 4 桁を数値化し、2023 以上か確認します。
3. 3 種類のツールボックスを `license('test', toolbox_id)` で確認します。Parallel Computing Toolbox だけは警告扱いで `ready` を落としません。
4. `gpuDeviceCount` と `gpuDevice` で GPU 情報を表示します。GPU 不在は警告のみです。
5. データファイル `../../data/tiredata/DriveBrake_sampled_quarter.csv` の存在を確認します。
6. 必須 M ファイル一覧について `isfile` で存在確認し、欠けていれば `ready = false` にします。
7. `output` ディレクトリがなければ `mkdir('output')` を試み、失敗時は `ready = false` にします。
8. 最後に結果サマリーと問題一覧を表示します。

## 6. 関数・ローカル関数詳細

### 6.1 `check_validation_readiness`

- `toolboxes` は表示名とライセンス ID の 2 列セル配列です。
- 必須ファイル一覧には `loadTireData.m` から `final_system_validation.m` まで 15 ファイルが固定で書かれています。
- ディスク容量確認では `system('df -h .')` または `system('dir')` を実行しますが、結果文字列 `result` は後続判定には使っていません。

## 7. 注意点

- GPU がなくても `ready` は `false` になりません。GPU は任意条件です。
- `output` ディレクトリは確認だけでなく作成まで行うため、副作用があります。
- ファイル存在確認はカレントディレクトリ基準の相対パスです。別ディレクトリから呼ぶと結果が変わる可能性があります。
