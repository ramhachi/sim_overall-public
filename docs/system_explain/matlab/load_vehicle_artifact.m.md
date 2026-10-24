# `matlab/load_vehicle_artifact.m`

## 1. 役割

この関数は、vehicle artifact JSON でも legacy な flat vehicle JSON でも、  
**後続コードが同じ形で扱える `artifact.params` 付き struct** に正規化して返す読み込み入口です。

## 2. 処理の流れ

1. ファイル存在確認
2. `fileread` + `jsondecode`
3. `extract_vehicle_params_from_artifact()` で車両 params を抽出
4. `artifact.params = params` を必ず設定
5. `schema_version`, `generated_by`, `optimization`, `performance.events` を最低限補完

このため、読み込まれたデータが完全 artifact でなくても、呼び出し側は比較的一定の前提で扱えます。

## 3. 互換性レイヤとしての重要性

本関数があることで、下流コードは

- `decoded.params` があるか
- ただの flat JSON か
- `optimization.stages` が cell か struct か

を毎回気にしなくて済みます。

特に `optimization.stages` を cell に揃える処理は、履歴処理コードの分岐を減らします。

## 4. 注意点

この関数は「読めるようにする」ためにかなり寛容です。  
したがって、**読み込みに成功したこと** と **その JSON が export や運用に十分であること** は別問題です。

今回の AC export ではまさにこれが問題になり、  
`load_vehicle_artifact()` で読めるだけでは export 候補にしないよう、別途 complete vehicle 判定を入れています。

## 5. システム上の位置付け

この関数は、sim_overall における車両 JSON の共通デシリアライザです。  
最適化結果の再利用、EasyStart での候補選択、AC export、POU 更新、RL quickstart の固定パラメータ読み込みなど、  
JSON を使うほぼすべての導線がここへ収束します。
