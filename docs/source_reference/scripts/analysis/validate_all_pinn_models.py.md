# scripts/analysis/validate_all_pinn_models.py

## 役割

deployable な PINN タイヤモデルをまとめて読み込み、実測との一致度と **Fy leakage** を確認する検証スクリプトです。

## 主な処理

- `manifest.json` から学習済みモデルを特定する
- 正規化パラメータ付きで PINN を復元する
- `PhysicsEnforcedTireModel` を通して実行時補正を適用する
- 実測 vs 予測プロット、DriveBrake 条件での Fy 漏れ、ゼロスリップ近傍の挙動を出力する

## 重要性

PINN を主系統として運用するうえで、学習済みモデルを一括で点検する安全弁です。  
「精度は高そうだが純縦条件で不自然な横力が出ていないか」を見ます。
