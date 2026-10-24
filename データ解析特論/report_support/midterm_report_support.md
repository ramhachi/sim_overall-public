# 中間レポート補助資料

## 要件チェック
- 現在の deck はサンプル PDF に近い構成で、タイトル、データ説明、目的変数と説明変数、外れ値解析、共線性チェック、結果、考察はすでに入っています。
- 現在の deck で最も不足が明確なのは `t-value` の明示です。
- 標準化回帰係数は比較図で部分的には触れていますが、サンプルにより近づけるなら数表か小さな棒グラフを併記すると安全です。

## 手動で足すとよいもの
- 6枚目: Fx 主解析の係数表に `t-value` 列を追加する。
- 7枚目: サンプルの体裁により寄せるなら、標準化回帰係数の数表か、生成した棒グラフを小さく貼る。
- 2枚目: 仮説を明示するなら `仮説: Fx は主に SR と Fz に支配され、IA と TSTC が補助的に効く。` をそのまま使える。
- 2枚目の文言注意: `Fx = 縦力`, `Fy = 横力` を全スライドで統一する。

## Fx 主解析
- n = 39790
- R^2 = 0.8830, Adjusted R^2 = 0.8830
- 標準化回帰係数が大きい順: SR=0.949, TSTC=0.051, Fz=0.021
- |t| が大きい順: SR=537.5, TSTC=28.3, Fz=12.3

## Fy 比較解析
- n = 15939
- R^2 = 0.8937, Adjusted R^2 = 0.8937
- 標準化回帰係数が大きい順: SA=-0.927, TSTC=0.055, Fz=0.048
- |t| が大きい順: SA=-349.4, TSTC=20.9, Fz=17.8

## 生成したファイル
- `midterm_regression_extended_summary.json`: t-value と標準化回帰係数を含む完全版サマリ
- `fx_main_regression_table.csv` / `fy_comparison_regression_table.csv`: そのまま貼り替えやすい数表
- `fx_main_regression_table.png` / `fy_comparison_regression_table.png`: 貼り込み用の表画像
- `fx_main_standardized_coefficients.png` / `fy_comparison_standardized_coefficients.png`: 標準化回帰係数の棒グラフ

## サンプル PDF から見える傾向
- サンプル deck は 7-9 枚程度で、考察を 1-2 枚に分ける形が多い。
- 外れ値解析と共線性チェックは、それぞれ独立スライドか明確に分離された節として見せている。
- 結果スライドでは、採用変数の回帰係数と補助統計量をコンパクトな表で示す例が多い。
