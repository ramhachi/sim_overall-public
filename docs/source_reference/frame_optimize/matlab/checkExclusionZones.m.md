# frame_optimize/matlab/checkExclusionZones.m

## 役割

部材が exclusion zone に侵入していないかを調べ、**要素ごとのペナルティ**を返す helper です。

## 主な処理

- node ペアから部材線分を作る
- `config.exclusion_zones` の AABB / その他 zone と交差判定する
- 交差した要素へ penalty を与え、zone 名も返す

## 実装上の注意

- `frameObjectiveFunction.m` はこの結果を合計し、衝突がある個体を `Inf` で棄却します。
- group-based exemption を考慮した評価に使われます。
