# `matlab/resolve_default_vehicle_json.m`

## 役割

`find_vehicle_json_candidates()` を内部で呼び、最上位候補 1 件だけを返す既定 artifact 解決 helper です。固定ファイル名依存をなくし、selector の優先度規則をそのまま既定選択にも適用するために追加されました。

## 受けるオプション

- `RequireExportReady`
- `RequireSuspensionHardpoints`
- `RequiredHardpointCorners`

## 処理の流れ

1. `projectRoot` が空なら `matlab/` 親をプロジェクトルートとして解決。
2. `find_vehicle_json_candidates(...)` を同じ条件で呼ぶ。
3. 候補があれば先頭 `candidates(1).path` を返し、無ければ空文字を返す。

## システム上の位置づけ

front / rear suspension、steering、visualization、frame resolver の「未指定時既定入力」を shared selector へ揃える薄い共通層です。
