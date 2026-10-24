# `matlab/export_assetto_corsa_nk16.m`

## 1. 役割

この関数は、sim_overall の車両 JSON を **Assetto Corsa の NK16_ICV 向け `data/` + `ui/` オーバーレイ** に変換します。  
上流 GitHub で `data.acd` しか見えないため、元 NK16 の unpack 済み物理ファイルをそのまま複製するのではなく、  
sim_overall 側の車両パラメータから AC 用の ini / lut を再構築する方針を取っています。

## 2. 入力と出口

### 入力

- `vehicleJsonPath`
- `OutputCarDir`
- `TemplateCarDir`
- `ScreenName`

### 出力

標準では `output/assetto_corsa/NK16_ICV/` に次を生成します。

- `data/car.ini`
- `data/suspensions.ini`
- `data/drivetrain.ini`
- `data/engine.ini`
- `data/power.lut`
- `data/brakes.ini`
- `data/tyres.ini`
- タイヤ LUT
- 空力 LUT
- `ui/ui_car.json`
- `sim_overall_export_metadata.json`

## 3. export 元 JSON の選び方

`vehicleJsonPath` が省略された場合、`getDefaultVehicleJson()` が呼ばれます。  
ここでは `find_vehicle_json_candidates(..., 'RequireExportReady', true)` を使い、  
**完全車両 JSON に限定した候補群** の中から最適化済み候補を抽出し、その中で最も新しいものを既定選択します。

この設計により、旧式の部分 / 集約 JSON を既定 export 元にしてしまう事故を防いでいます。

## 4. export-ready 判定

`isExportReadyArtifact()` は、読み込んだ `artifact.params` に

- `mass`
- `suspension`
- `tire`
- `powertrain`
- `aero`
- `brake`

が揃っているかだけを見ます。

以前はファイル名や `design_vector` の有無で通していましたが、それでは baseline merge によって  
「足りない項目が自動補完され、もっともらしい AC データが出てしまう」問題がありました。  
今回の修正で、**完全車両でない JSON は export 前に明示的に reject** されます。

## 5. パラメータ処理

読み込み後は

1. `load_vehicle_artifact()`
2. `merge_nested_structs(create_vehicle_baseline_template(), artifact.params)`
3. `deriveVehicleMetrics(params)`

の順で、AC 出力に必要な派生量を作ります。

`deriveVehicleMetrics()` が実質的な変換中心で、次を算出します。

- 質量・前後重量配分
- CG 高さ
- トレッド
- タイヤ半径・幅
- バネ下質量
- モーションレシオと wheel rate
- ダンパ相当値
- アライメント
- 摩擦係数
- 慣性半径
- 最大出力 / 最大トルク / レッドライン
- 最高速推定
- ブレーキ前後配分
- 空力バランス
- ステア比 / FF 係数

## 6. AC 物理ファイル生成の思想

この関数は NK16 の元 ini をコピーするのではなく、sim_overall の車両モデルから AC の近い表現へ写像します。  
したがって生成値は **厳密な upstream NK16 内部値そのものではなく、sim_overall パラメータに基づく再構成値** です。

代表的な対応は次の通りです。

- `car.ini`: 慣性半径、ドライバー視点、ステア比
- `suspensions.ini`: トレッド、wheelbase、キャンバ、トー、ばね / ダンパ相当
- `drivetrain.ini`: ギア比、LSD ramp / preload
- `engine.ini` + `power.lut`: エンジントルク曲線
- `brakes.ini`: 最大制動トルクと前後配分
- `tyres.ini` + LUT: 摩擦係数から簡易的な AC 曲線を構成
- `aero.ini` + LUT: `CL_A`, `CD_A`, `aero_balance` から front / rear wing gain を構築

## 7. テンプレート利用

`TemplateCarDir` が指定された場合、`data/` と `data.acd` を除くテンプレート車両ファイルを先にコピーできます。  
このため、既存の 3D モデルやサウンドなどを保ちつつ、物理だけ差し替える運用が可能です。

## 8. メタデータ出力

`sim_overall_export_metadata.json` には次を記録します。

- 生成時刻
- 元 JSON パス
- 元 JSON ラベル
- 出力先
- 生成ファイル一覧
- 利用手順

これは export 成果物を後から追跡しやすくするための記録層です。

## 9. システム上の位置付け

この関数は、sim_overall の統一車両 JSON を外部ゲーム環境へ橋渡しする変換器です。  
MATLAB 側で完結し、`EasyStart.m` から呼べるため、RL 学習中でも別ラインで官能評価用 MOD 準備を進められるようになっています。
