# Suspension Compliance / Geometry Study

このフォルダは、`suspension_kinematics_design/` を **アーム剛性/コンプライアンス対応** と **別サスペンション形式対応** に拡張するためのリサーチ成果物をまとめたものです。

## ファイル構成

- `01_current_system_audit.md`
  - 現行システムの監査。どこが A-arm 前提で、どこに KC / compliance の受け皿があるかを整理。
- `02_gpt54_report.md`
  - GPT-5.4 系統の独立リサーチ結果。
- `03_claude_second_opinion.md`
  - Claude 系統の独立リサーチ結果。
- `04_team_debate.md`
  - 各系統の一致点・相違点・最終統合判断。
- `05_final_recommendation.md`
  - 実装優先順位、計算根拠、拡張ロードマップ、非推奨順序。

## 先に結論

1. **今すぐ足すべきなのは、汎用マルチリンク最適化器ではなく、現行 wishbone 系に対する compliance-aware オプションです。**
2. **この repo では runtime 側がすでに KC / compliance surrogate を受けられるため、最短経路は新スキーマの発明ではなく既存 `VehicleParams` への正規 export です。** `cpp/include/fsae/vehicle/vehicle_params.hpp:117-190,438-499`
3. **最初の “新 family” は rear toe-link / A-arm+toe-link 系、その次に front virtual kingpin / double-joint upright 系を入れるのが妥当** です。
4. **pushrod / pullrod は “新しいキネマティクス形式” ではなく、荷重経路と剛性を変える actuation/load-path option として扱うのが妥当** です。`suspension_kinematics_design/FSAE_Suspension_Design_JP.md:10-37`

## モデル系統について

ユーザー要望は **GPT-5.4 + Claude Opus 4.6** でしたが、この task runtime では Claude Opus 4.6 / 4.5 が起動できませんでした。したがって Claude 系の第二意見は **Claude Sonnet 4.6** で代替し、その制約も文書中に明示します。
