# `matlab/pou_hardpoint_design/export/update_vehicle_params_json.m`

## 1. 役割

この関数は、POU 逆算で得た物理ハードポイントを **既存 vehicle artifact JSON に追記し直す** 更新器です。  
単に別 JSON を新規作成するのではなく、既存の車両正本へ戻し込む点が重要です。

## 2. 更新対象

更新先は `params_json_path` で指定された既存車両 JSON です。  
対象 JSON は `load_vehicle_artifact()` で読み込まれ、`data.params` を正本として編集されます。

## 3. 追加される内容

`target.pou_hardpoints.<wheel>` に次の物理座標が入ります。

- bellcrank / pushrod 関連点
- upper / lower arm chassis-side / upright-side
- tie rod
- kingpin axis
- installation ratio

つまり、もともとの多項式係数だけでなく、**実体座標の POU ハードポイント情報** が単一 JSON に追加されます。

## 4. artifact 更新方法

関数末尾では

- `stage_entry` を構築
- `build_vehicle_artifact(target, 'base_artifact', data, 'stage_entry', stage_entry)`
- `save_vehicle_artifact(params_json_path, data)`

を行います。

この流れによって

- `params` は更新後状態へ差し替え
- `optimization.stages` には `update_pou_hardpoints` ステージが追記
- 元ファイルパスに上書き保存

されます。

## 5. 単一 JSON 管理という観点での意味

この関数が存在するため、POU の最適化結果は別ファイルに散らばらず、  
**最終的な車両 artifact JSON に統合されます**。

今回のコード追跡でも、サスペンション / POU 系はこの経路により単一 JSON 化されていることを確認できます。
