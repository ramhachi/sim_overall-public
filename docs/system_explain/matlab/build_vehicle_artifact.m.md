# `matlab/build_vehicle_artifact.m`

## 1. 役割

この関数は、車両諸元 `params` を中心に、最適化段階情報・イベント結果・メタ情報をまとめて  
**単一 vehicle artifact 構造体** を組み立てる標準関数です。

sim_overall で「最適化後の値を 1 つの JSON に集約する」という思想の中心にあるのがこのファイルです。

## 2. 生成する階層

最低限、出力には次の骨格が入ります。

- `schema_version`
- `generated_by`
- `params`
- `optimization.stages`
- `performance.events`

ここで `params` が車両の現時点正本、`optimization.stages` が履歴、`performance.events` が評価結果です。

## 3. 引数の意味

- `params`: 現在の正本車両パラメータ
- `base_artifact`: 既存 artifact を引き継いで増分更新したい時の土台
- `stage_entry`: 今回追加する最適化 / 更新ステージ
- `event_name`, `event_result`: イベント結果の記録

この設計により、単発保存にも、既存 artifact へ後工程を追記する用途にも使えます。

## 4. 履歴の積み方

`stage_entry` が渡されると `artifact.optimization.stages{end+1}` に追記されます。  
しかも `stage_entry.params` が欠けている場合は現在の `params` を補うため、  
履歴 1 件ごとに「その時点でどの車両仕様だったか」を残しやすくなっています。

これは後から見た時に、

- どの optimizer が
- どのイベント / ホイール / 条件で
- どの params を作ったか

を追跡しやすくするためです。

## 5. イベント結果の格納

`event_name` と `event_result` が与えられると、  
`artifact.performance.events.<event_key>` に保存されます。

そのため artifact JSON 1 つの中に

- 車両仕様
- 最適化履歴
- 各イベントの性能結果

を同居させられます。

## 6. 単一 JSON 管理という観点での意味

サスペンション、空力、LSD、タイヤ、POU 係数、フレーム剛性目標など、  
シミュレーションが実際に使う車両値を `params` に集約できるのはこの関数があるためです。

一方で、本関数は **何でも自動で集約する魔法の関数ではありません**。  
後工程が明示的に `base_artifact` を渡して呼ばなければ、その工程の結果は artifact に入りません。  
今回の確認でも、POU 更新はここを通って JSON へ戻されますが、フレーム最適化の詳細結果は現状この経路へ戻していません。
