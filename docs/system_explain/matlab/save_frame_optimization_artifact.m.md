# `matlab/save_frame_optimization_artifact.m`

## 1. 役割

この関数は、フレーム最適化の最終結果を **統一 vehicle artifact JSON へ書き戻す保存器** です。  
今回の JSON 統合の中心であり、これによってフレーム詳細も単一 JSON 管理へ入ります。

## 2. 何を保存するか

保存対象は大きく 3 系統です。

1. `params.suspension.K_chassis`
   - 最終的に得られた `K_torsion` で更新
2. `params.frame`
   - フレーム最適化の詳細結果を格納
3. `optimization.stages`
   - `frame_optimization` ステージを追記

## 3. `params.frame` の内容

この関数は `params.frame` に次をまとめます。

- schema / source / timestamp
- 選択 pipe catalog 情報
- 設計変数ベクトル
- mass / CG / torsional stiffness / penalties
- target stiffness / target mass
- nodes / connectivity / group indices / group names
- GA 実行サマリ

つまり、これまで外部出力や可視化内に散っていたフレーム設計情報を、  
車両 artifact JSON から直接辿れるようにしています。

## 4. 既存 artifact の更新

`BaseArtifactPath` が存在する場合は `load_vehicle_artifact()` で読み込み、  
既存の `params` や `optimization` 履歴を保持したまま frame 情報だけ上積みします。

これにより、

- 既存の `optimized_autocross.json`
- 既に POU 更新済みの artifact

なども、**同じ単一 JSON を保ったまま frame 結果を追記** できます。

## 5. 入力 JSON がない場合

元の vehicle JSON が指定されない場合は、`create_vehicle_baseline_template()` を土台にしつつ  
現在の geometry / mass / track / CG 情報を反映した新規 unified artifact を作ります。

標準の保存先は `output/runtime/frame_optimized_vehicle_artifact.json` です。

## 6. システム上の位置付け

この関数が追加されたことで、sim_overall の単一 JSON 管理は

- 車両ラップタイム最適化
- POU 物理ハードポイント
- フレーム最適化詳細

までを一貫して抱えられるようになりました。
