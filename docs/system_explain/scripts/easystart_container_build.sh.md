# `scripts/easystart_container_build.sh`

## 1. 役割

EasyStart 系コンテナ実行で使う **再構成付き CMake ビルドラッパ** です。  
ビルド署名が変わった時だけ `build` を掃除し、指定 generator と CPU 最適化プロファイルで再構成します。

## 2. 入出力

- 実行形式: `easystart_container_build.sh <build-signature> <generator> <cpu-profile> [targets...]`
- 入力:
  - ビルド署名
  - CMake generator
  - CPU 最適化プロファイル
  - 任意のビルドターゲット列
- 出力:
  - `/workspace/build`
  - `/workspace/build/.easystart_build_profile`

## 3. トップレベルフロー

1. 引数不足時は usage を出して終了コード `64` で終了します。
2. `build_signature`, `generator`, `cpu_profile` を切り出します。
3. `build/.easystart_build_profile` が存在しない、または署名不一致なら `build` 直下を掃除します。
4. `cmake -S /workspace -B /workspace/build` を Release・テスト有効・Python binding 無効で実行します。
5. 成功した署名を profile ファイルへ保存します。
6. ターゲット指定が無ければ全体ビルド、あれば指定ターゲットのみビルドします。

## 4. 副作用

- 署名変更時に `build` 直下の成果物を削除します。
- `.easystart_build_profile` に前回署名を書き込みます。

## 5. 注意点

- 実行環境は `/workspace` 前提です。
- `BUILD_PYTHON_BINDINGS=OFF` が固定なので Python 拡張ビルド用途ではありません。
