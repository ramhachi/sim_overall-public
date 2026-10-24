# `matlab/verify_skidpad_time.m`

## 1. ファイルの目的

固定の設計変数ベクトルを `apply_full_design()` で物理パラメータへ変換し、スキッドパッドのラップタイムを再確認する検証スクリプトです。

## 2. 主な入出力または依存関係

- 入力: スクリプト内固定の `design_vector`
- 出力: 標準出力へのラップタイム・平均速度・誤差表示
- 依存関係:
  - `VehicleSimRunner`
  - `apply_full_design`

## 3. 処理の流れ / アルゴリズム

1. プロジェクトルートから `VehicleSimRunner` を作成します。
2. 必要最小限の `base_params` を手動で用意します。
3. 固定 `design_vector` を `apply_full_design()` へ渡して車両パラメータへ展開します。
4. skidpad を 20 秒上限で実行し、完走ならラップタイムや平均速度を表示します。
5. 失敗なら結果未返却か未完走かを区別して表示します。

## 4. ファイル内の定義

### スクリプト本体

- `optimized_full_vehicle.json` 相当の設計変数をコメント付きで直書きしています。
- シミュレーション成功時には `4*pi*9.125` を周回距離として平均速度を計算します。

## 5. 注意点・前提条件

- `base_params` は簡易版で、完全テンプレートではありません。
- `design_vector` の並びは `apply_full_design()` の旧インデックス契約に依存します。
