# `scripts/refit_improved.sh`

## 1. 役割

改善したフィッティング条件で `tire_fitting/main.py` を再実行する **手動再学習ランチャー** です。  
境界条件とフィルタ変更点を先に表示し、その条件で Hoosier データを再フィットします。

## 2. 入出力

- 実行例: `bash scripts/refit_improved.sh`
- 入力:
  - `data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv`
  - `data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv`
- 出力:
  - `output/tire_params_v2/`
  - 次工程として使う検証コマンドの案内

## 3. トップレベルフロー

1. 冒頭で改善点を人間向けに表示します。
2. `python3 tire_fitting/main.py` を固定引数付きで実行します。
3. 完了後、`validate_predictions.py` を使った次の検証コマンドを表示します。

## 4. 実際に固定されている条件

- 出力先: `output/tire_params_v2`
- タイヤ名: `Hoosier_18x6_R20_v2`
- 最小速度: `25.0`
- 温度範囲: `30.0` ～ `90.0`
- `--remove-outliers` 有効

## 5. 副作用

- フィット済みパラメータファイルを新規出力します。
- 元の既定出力 `output/tire_params` ではなく `_v2` 系へ分離保存します。
