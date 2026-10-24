# `matlab/pou_hardpoint_design/visualization/plot_comparison.m`

## 1. 目的

最適化結果の比較可視化を 1 枚の figure にまとめる関数です。ジャッキアップ・キャンバーの目標/幾何比較、誤差プロット、ヒストグラム、要約表を 2×3 レイアウトで表示します。

## 2. 入出力と依存関係

- 入力:
  - `results`: `main_pou_hardpoint_optimizer` の出力
- 出力:
  - `fig`: Figure ハンドル
- 依存関係:
  - `figure`, `subplot`, `plot`, `fill`, `histogram`, `text`, `sgtitle` などの描画 API

## 3. 全体処理フロー

1. `1200x800` の figure を作ります。
2. 1 枚目にジャッキアップの目標曲線と最適化曲線を描きます。
3. 2 枚目にキャンバーの目標曲線と最適化曲線を描きます。
4. 4 枚目と 5 枚目で、それぞれジャッキアップ誤差とキャンバー誤差の塗りつぶし付き時系列を描きます。
5. 3 枚目にジャッキアップ誤差ヒストグラムを描きます。
6. 6 枚目に数値要約テキストを固定幅フォントで置きます。
7. `sgtitle` で全体タイトルを付けます。

## 4. 関数詳細

### `plot_comparison(results)`

- **使用する `results` フィールド**  
  `delta_range_deg`, `jack_target_mm`, `jack_curve_mm`, `camber_target_deg`, `camber_curve_deg`, `polynomial_error_mm`, `max_camber_error_deg`, `M_pivot_chassis`, `bellcrank_pivot`, `installation_ratio`, `optimization_exitflag`, `optimization_time_s` などを参照します。

- **誤差プロットの軸設定**  
  最大誤差が 1 未満の場合でも見やすさのため最低 ±1 を確保し、その 1.2 倍まで表示します。

- **要約欄**  
  `results.packaging_valid` を `YES/NO` に変換して文字列化します。

- **コメントとの差分**  
  ヘッダコメントには「4. 3D hardpoint visualization」とありますが、実装されている 6 枚目は 3D 図ではなく要約テキストです。

- **ローカル関数**  
  このファイルにローカル関数はありません。
