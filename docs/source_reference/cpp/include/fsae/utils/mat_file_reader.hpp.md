# mat_file_reader.hpp

- **目的**: MATLAB `.mat` ファイルから 1D/5D 配列やスカラーを読み込みます。
- **主要要素**: `Array5D`、`MatFileReader`、`MatFile`。
- **理論背景**: MATLAB の column-major 配列を C++ 側データ構造へ安全に移し替えます。
- **依存**: `matio.h`、`vector`、`array`。
- **システム上の位置づけ**: ルックアップテーブルや MATLAB 連携データの読込ユーティリティです。
