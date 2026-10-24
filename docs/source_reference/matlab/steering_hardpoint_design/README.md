# `matlab/steering_hardpoint_design/`

ステアリングハードポイント設計用ディレクトリ。  
front suspension hardpoints を含む vehicle artifact を受け取り、タイロッド配置を最適化して同じ artifact へ書き戻します。

## 収録ファイル

- `main_steering_optimizer.m.md` — Layer 1.5 ステアリング最適化本体

## このディレクトリの役割

サスペンション基礎設計と車両運動モデルの中間に位置し、  
artifact-first の front steering geometry を確定させる役割を持ちます。
