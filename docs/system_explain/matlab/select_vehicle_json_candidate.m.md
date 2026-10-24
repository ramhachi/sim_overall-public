# `matlab/select_vehicle_json_candidate.m`

## 1. 役割

`find_vehicle_json_candidates()` を UI 化し、コンソールで番号選択させる関数です。EasyStart の複数メニューから共通利用されます。

## 2. 受け取るオプション

- `ProjectRoot`
- `Prompt`
- `RequireExportReady`
- `RequireSuspensionHardpoints`
- `RequiredHardpointCorners`

## 3. 処理の流れ

1. `find_vehicle_json_candidates()` を同じ条件で呼ぶ。
2. 候補が無ければ warning 的メッセージで終了。
3. 各候補について path / optimized tag / modified_at / summary を表示。
4. `input('', 's')` で番号を受け取る。
5. 妥当な番号なら `selectedPath`, `ok`, `selectedCandidate` を返す。

## 4. 今回の変更で重要な点

- export-ready 条件だけでなく **hardpoint 条件も UI に引き回せる** ようになりました。
- そのため frame は `FL+RL`、steering は `FL` を必須とする選別が可能です。

## 5. システム上の位置づけ

shared selector の対話版です。EasyStart 本体に候補列挙ロジックを重複させないための部品です。
