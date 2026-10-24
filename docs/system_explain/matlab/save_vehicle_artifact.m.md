# `matlab/save_vehicle_artifact.m`

## 1. 役割

この関数は `build_vehicle_artifact()` 等で組み上がった artifact struct を、  
**JSON ファイルとして永続化する唯一の基本保存関数** です。

## 2. 実装の特徴

処理は短いですが重要です。

1. `jsonencode(..., 'PrettyPrint', true)` で可読 JSON を生成
2. `fopen` に失敗したら即 error
3. `onCleanup` で `fclose` を保証
4. `fwrite` で文字列を書き出す

余計な整形や暗黙補完を入れず、**artifact の中身をそのまま保存する薄い出口** に徹しています。

## 3. なぜ重要か

sim_overall では「最適化で得た値を単一 JSON に残す」思想を採っていますが、  
その思想を物理的なファイルに変える最終地点がこの関数です。

したがって、最適化や後処理が JSON に反映されるかどうかは、

- 上流が `build_vehicle_artifact()` を呼ぶか
- その結果を `save_vehicle_artifact()` へ渡すか

で決まります。

## 4. 単一 JSON 管理との関係

POU 更新のように既存 artifact を読み、項目を追加し、また同じパスへ保存する処理は、  
この関数によって「既存 JSON を正本のまま育てる」方式を取れます。

逆に、フレーム最適化のようにこの保存関数を通っていない工程は、  
現在の vehicle artifact JSON へ自動統合されません。
