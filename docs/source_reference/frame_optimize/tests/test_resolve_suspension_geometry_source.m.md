# frame_optimize/tests/test_resolve_suspension_geometry_source.m

## 役割

`resolve_suspension_geometry_source.m` の **fail-fast 契約**を確認する回帰テストです。

## 主な確認

- checked-in fixture を preferred file に渡したとき、そのまま返る
- `ErrorIfMissing=true` かつ artifact が無い root では `MissingGeometry` で落ちる

## 実装上の注意

- canonical flow が warning 続行ではなく明示失敗に寄ったことを固定する目的のテストです。
