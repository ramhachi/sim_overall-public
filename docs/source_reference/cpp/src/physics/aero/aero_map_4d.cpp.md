# aero_map_4d.cpp

- **目的**: `AeroMap4D` テンプレートを `float` / `double` で明示的インスタンス化します。
- **主要要素**: `template class AeroMap4D<double/float>;`。
- **理論背景**: 4D 補間クラスのコード生成をライブラリ側で固定し、利用側のビルド負荷を減らします。
- **依存**: `aero_map_4d.hpp`。
- **システム上の位置づけ**: 空力マップ機能を `fsae_core` に組み込むための実装ファイルです。

