# `matlab/pou_hardpoint_design/example_usage_jp.m`

## 1. 目的

`example_usage.m` の日本語版デモです。前半は 1 輪の最適化例、後半はユーザー入力付きで全 4 輪を順番に処理する追加例を含みます。

## 2. 入出力と依存関係

- 形式: スクリプト
- 主入力:
  - `../optimized_autocross.json`
  - `input('', 's')` で受ける全輪最適化実行可否
- 主出力:
  - 変数 `results`
  - 条件付きで `all_results`
  - コンソール表示
  - `plot_comparison(results)` の図
- 依存関係:
  - `geometry` / `optimization` / `visualization` の各パス追加
  - `main_pou_hardpoint_optimizer`
  - `plot_comparison`

## 3. 全体処理フロー

1. クリア後に各サブフォルダを `addpath` します。
2. `optimized_autocross.json` の存在を確認します。
3. FL を対象にした既定オプションを作成します。
4. `main_pou_hardpoint_optimizer` を実行して 1 輪分の結果を表示します。
5. `plot_comparison` を試行します。
6. Phase 3 未実装として SolidWorks エクスポート予定を表示します。
7. 追加例として全輪ループ用コード断片を文字列で表示します。
8. `実行しますか？ (y/n)` に対する入力が `y` のときだけ、`wheel_index=0:3` で 4 輪最適化を実行し、サマリー表を出力します。

## 4. ブロック詳細

- **単輪デモ部**  
  構成は英語版と同じで、評価指標・閾値コメントだけ日本語化されています。

- **性能表示部**  
  `polynomial_error_mm < 2.0`、`max_rod_angle_deg < 15.0`、`installation_ratio` が `0.10～0.30` に入るかで表示文を切り替えます。ここで閾値判定しても、値自体は変更しません。

- **パッケージング表示部**  
  `results.packaging_valid` が偽なら `results.violations` を順に出力します。

- **全輪最適化案内部**  
  `fprintf` でサンプルコードを表示するだけの部分と、実際に `input` を取り `for i = 0:3` を回す実行部分が両方あります。

- **全輪実行部**  
  `wheel_names = {'FL', 'FR', 'RL', 'RR'}` を使い、各ホイールで `main_pou_hardpoint_optimizer(params_file, struct('wheel_index', i, 'verbose', false))` を呼びます。結果は `all_results{i+1}` に格納されます。

- **全輪サマリー部**  
  各ホイールの `polynomial_error_mm`、`max_rod_angle_deg`、`installation_ratio`、`packaging_valid` を表形式で出力します。

- **ローカル関数**  
  このファイルにローカル関数はありません。
