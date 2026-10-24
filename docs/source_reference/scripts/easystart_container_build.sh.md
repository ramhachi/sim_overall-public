# easystart_container_build.sh

- **目的**: EasyStart 系のコンテナ内ビルドを初期化し、必要ならビルドディレクトリを掃除して CMake/Ninja ビルドを実行します。
- **主要要素**: 引数解析、ビルド署名比較、`cmake -S/-B` と `cmake --build` の呼び出し。
- **理論背景**: ビルド設定が変わった場合のみ `/workspace/build` をクリーンにし、CPU 最適化プロファイル込みで再構成します。
- **依存**: `bash`、`cmake`、`nproc`、`find`。コンテナ内の標準 UNIX ツール前提です。
- **システム上の位置づけ**: 開発環境の再現性あるビルド入口です。
