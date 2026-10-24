# `matlab/find_vehicle_json_candidates.m`

## 1. 役割

sim_overall 内で利用可能な車両 JSON / artifact を探索し、選択 UI や既定解決がそのまま使える候補 struct 配列へ正規化して返します。

## 2. 探索範囲

- `matlab/`
- `matlab/parameter_presets/`
- `output/runtime/`

`dir(fullfile(rootDir, '**', '*.json'))` により再帰探索します。

## 3. 候補 struct

- `path`
- `label`
- `name`
- `group`
- `datenum`
- `modified_at`
- `is_optimized`
- `has_suspension_hardpoints`
- `summary`
- `params`

## 4. 重要なフィルタ

### `RequireExportReady`
完全車両 struct だけを通します。

### `RequireSuspensionHardpoints`
hardpoints を持つ artifact だけを通します。

### `RequiredHardpointCorners`
`{'FL'}` や `{'FL','RL'}` のように corner を限定できます。これにより steering と frame が必要な artifact を誤らず選べます。

### transient runtime params 除外
`output/runtime/params_run.json` と `params_drs_run.json` は一時派生物なので候補から外します。

## 5. 判定の流れ

1. `load_vehicle_artifact()` で読む。
2. `artifact.params` を raw 判定用に使う。
3. baseline template と merge した `params` を表示 / 実利用用に作る。
4. optimized 判定、hardpoint 判定、summary 生成を行う。
5. `is_optimized` → group priority → 更新日時で並べ替える。

## 6. システム上の位置づけ

EasyStart、frame、suspension、steering、AC export の vehicle artifact 選択規則を一元化する SSOT selector です。
