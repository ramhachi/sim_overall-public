# `matlab/dl_tire_model/loadTireData.m`

## 1. 概要

`loadTireData` は、TTC 形式の CSV 群を読み込み、必要列だけを抜き出して 1 つの table に連結する関数です。DriveBrake と Cornering の 2 形式を先頭行で判別します。

## 2. ファイルの役割

- 入力 CSV 群の存在確認
- TTC の 2 形式に応じた `readtable` 設定
- 列名の大小文字差異吸収
- 必須列の検証と必要列抽出
- 複数ファイルの縦連結

## 3. インターフェース

### 関数

`data = loadTireData(data_paths)`

### 引数

- `data_paths`
  - CSV パスのセル配列。

### 戻り値

- `data`
  - 結合済み table。
  - コメント上の必須列は `SR, SA, Fz, IA, TSTC, Fx, Fy, Mx, Mz`。

## 4. 依存関係

- MATLAB 標準関数
  - `isfile`, `fopen`, `fgetl`, `fclose`, `contains`, `detectImportOptions`, `readtable`, `setdiff`, `strjoin`, `height`

## 5. 処理の流れ

1. 必須列一覧 `required_cols` を定義する。
2. 空 table を初期化する。
3. `data_paths` を順に処理する。
4. 各ファイルについて
   - 存在確認を行う
   - 先頭行を読み、`"TIRF Data File"` を含むかで形式判定する
   - DriveBrake なら列名 2 行目、データ開始 3 行目に設定する
   - それ以外は Cornering とみなし、列名 1 行目、データ開始 3 行目に設定する
   - `readtable` で読み込む
   - `FX/FY/FZ/MX/MZ` を `Fx/Fy/Fz/Mx/Mz` へリネームする
   - 必須列が揃っているか確認する
   - 必須列のみ抽出する
   - 既存 table へ縦連結する
5. 各ファイルの読込件数と総件数を表示する。

## 6. 関数・ローカル関数詳細

### `loadTireData`

- DriveBrake 判定条件は、先頭行に `"TIRF Data File"` が含まれるかどうかです。
- `opts.VariableNamingRule = 'preserve'` により原列名を保持した上で、後段で必要なものだけ手動対応しています。
- 列名マッピングは `FX/FY/FZ/MX/MZ` だけで、他の綴り違いは補正しません。

### ローカル関数

- 定義されていません。

## 7. 注意点

- 判別ロジックは 2 形式のみを想定しています。
- 必須列が不足すると `loadTireData:MissingColumns` で停止します。
- 返り値は必要列だけを持つ table で、元 CSV の他列は保持しません。
