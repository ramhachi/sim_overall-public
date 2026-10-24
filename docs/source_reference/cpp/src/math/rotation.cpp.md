# rotation.cpp

- **目的**: `rotation.hpp` のテンプレート関数を `float` / `double` で明示的インスタンス化します。
- **主要要素**: `euler_to_rotation_matrix`、`angular_velocity_to_euler_rate`、`gravity_in_body` の明示的生成。
- **理論背景**: テンプレート実装のリンク安定化とコンパイル単位整理が狙いです。
- **依存**: `rotation.hpp`。
- **システム上の位置づけ**: 姿勢変換ユーティリティを静的ライブラリへ取り込むための実装ファイルです。

