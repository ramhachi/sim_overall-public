# `matlab/rl_driver/trainAceDriverPhase4Extension.m`

## 概要

完走済み ace final agent を起点に、新しい run ディレクトリで **phase4 の追加学習** だけを回す継続学習エントリです。

## 主な入力

- `ResumeAgentPath`
- `SaveBaseDir`
- `CompletedEpisodesOffset`
- `Phase4Episodes`
- `Phase4Steps`
- `RandInterval`
- `RandStrength`

## システム上の役割

「ace final からもう少し磨く」ための専用 continuation entrypoint です。
