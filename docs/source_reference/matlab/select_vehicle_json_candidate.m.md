# `matlab/select_vehicle_json_candidate.m`

## 概要

`find_vehicle_json_candidates()` が返した候補一覧を MATLAB コンソールへ表示し、番号入力で 1 件選ばせる対話ラッパです。

## 主な入力

- `ProjectRoot`
- `Prompt`
- `RequireExportReady`
- `RequireSuspensionHardpoints`
- `RequiredHardpointCorners`

## 主な出力

- `selectedPath`
- `ok`
- `selectedCandidate`

## 重要な挙動

- optimized 候補には一覧上でタグを付けます。
- `selectedCandidate` を返すので、呼び出し側は path だけでなく label / summary をログに再利用できます。
- hardpoint 条件を引き回せるため、用途ごとに必要な artifact だけを UI に出せます。

## システム上の役割

EasyStart の共通 JSON 選択 UI です。候補ロジックをランチャー本体へ埋め込まず、選択規則の変更を 1 箇所へ閉じ込めます。
