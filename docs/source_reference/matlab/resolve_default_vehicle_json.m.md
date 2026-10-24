# `matlab/resolve_default_vehicle_json.m`

## 概要

候補選択 UI を使わず、優先度順で最上位の車両 artifact を 1 件返す helper です。`optimized_autocross.json` のような固定ファイル名前提を置き換えるために追加されました。

## 主な入力

- `projectRoot`
- `RequireExportReady`
- `RequireSuspensionHardpoints`
- `RequiredHardpointCorners`

## 主な出力

- `jsonPath`

## システム上の役割

「既定で何を使うか」を shared selector へ統一する薄い層です。
