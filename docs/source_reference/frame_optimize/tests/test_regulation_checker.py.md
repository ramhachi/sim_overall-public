# frame_optimize/tests/test_regulation_checker.py

## 役割

Python 側の `fsae_frame_optimizer.py` と `fsae_regulation_checker.py` を対象に、規則チェッカーの判定ロジックを pytest で検証するテストです。

## 何を検証するか

- 管材外径・肉厚の Size 判定
- メインフープやフロントフープの角度
- ホイールベース、トレッド比、サイドインパクト高さ
- インパクトアッテネータ寸法とエネルギー
- 材料制約
- `check_all()` とレポート生成

## このサブシステムとの関係

MATLAB の現行最適化ループに Python 規則チェッカーが直接入っているわけではありません。ですが、**規則ロジックをどこまで詳細化したいか** を知る参考として有用です。`checkFrameCompliance.m` より細かい観点を含むため、将来の統合候補を読む入口にもなります。
