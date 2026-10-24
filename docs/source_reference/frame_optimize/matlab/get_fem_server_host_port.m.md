# frame_optimize/matlab/get_fem_server_host_port.m

## 役割

ホスト側 FEM TCP サーバのポート番号を返す小さな helper です。

## 主な処理

- `FSAE_HOST_FEM_PORT` を読む
- 未設定や不正値なら既定値 `50051` に戻す

## 実装上の注意

- ホスト名は固定で `localhost` 前提です。
- Java TCP client と server manager の両方から参照されます。
