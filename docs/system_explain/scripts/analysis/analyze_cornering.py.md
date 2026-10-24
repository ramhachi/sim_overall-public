# `scripts/analysis/analyze_cornering.py`

## 1. 役割

コーナリング TTC 生データに対し、**前処理フィルタがどれだけサンプルを落とすか** を段階的に確認する診断スクリプトです。

## 2. 入出力

- 実行例: `python3 scripts/analysis/analyze_cornering.py`
- 入力: `data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`
- 出力: 標準出力の件数・範囲・フィルタ後件数

## 3. トップレベルフロー

1. Cornering CSV を `skiprows=[1]` で読みます。
2. `FZ` を正値へ変換します。
3. 総行数と `FZ/SR/V/TSTC` の範囲を表示します。
4. 次の順でフィルタし、各段階の残件数を表示します。
   - `V >= 10`
   - `FZ >= 500`
   - `|SR| <= 0.01`
   - `35 <= TSTC <= 85`
5. 最後に `FZ` フィルタ後の `SR` 分布統計を表示します。

## 4. 注意点

- 関数化されておらず、import すると即実行されます。
- 図は作らず、純粋にデータ量と分布確認に特化しています。
