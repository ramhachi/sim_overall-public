# frame_optimize/matlab/load_suspension_hardpoints.m

## 役割

前輪側サスペンションのハードポイント JSON を読み、フレーム最適化で使う前側取付点へ変換する関数です。あわせて Front Bulkhead (FBH) の 4 節点も作ります。

## 変換している内容

- JSON 側の `FL` データを基準に使う
- 左右対称を前提に FR 側を鏡映で生成する
- 座標系をサスペンション JSON からフレーム側へ写像する
- `offset_x_m` と `offset_z_m` で FBH の相対位置を決める

## 出力

- `fbh_nodes`: FBH の 4 点
- `front_mount_nodes`: 前輪サスペンションの 8 取付点
- `upright_nodes`: 可視化用のアップライト点群
- `success`: 読込成功フラグ

## どこで使うか

`generate_parametric_nodes.m` から呼ばれ、前側固定節点群の基準になります。ここで返す 8 点の順序は、後段の荷重点設定やブラケット生成でそのまま使われます。
