# `matlab/rl_driver/resumeAceDriverPhase4ExtensionAfterRestart.m`

## 概要

phase4 extension 用の再起動 bootstrap です。deployment candidate を守りつつ、必要なら fallback を publish してから `trainAceDriverPhase4Extension()` を起動します。

## システム上の役割

再起動後の手動復旧手順を 1 コマンドへまとめる運用 helper です。
