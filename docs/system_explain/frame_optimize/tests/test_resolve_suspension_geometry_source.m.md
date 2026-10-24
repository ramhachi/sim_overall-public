# `frame_optimize/tests/test_resolve_suspension_geometry_source.m`

## 役割

resolver の fail-fast 契約を固定するスモークテストです。

## 何を確認するか

- checked-in fixture を preferred file にすると、そのまま返る
- `ErrorIfMissing=true` で artifact が存在しない root では `MissingGeometry` になる
