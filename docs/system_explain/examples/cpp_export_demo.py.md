# `examples/cpp_export_demo.py`

## 1. 役割

ハイブリッドタイヤモデル設定を **YAML と C++ ヘッダへ書き出す一連の流れ** を説明するデモスクリプトです。

## 2. 入出力

- 実行例: `python3 examples/cpp_export_demo.py`
- 入力: スクリプト内で定義した FX/FY パラメータ、評価指標、メタデータ
- 出力:
  - `output/cpp_demo/hoosier_tire_config.yaml`
  - `output/cpp_demo/hoosier_tire_params.h`
  - 標準出力への生成物表示

## 3. `main()` の流れ

1. `output/cpp_demo` を生成します。
2. Magic Formula 用の `fx_params` と TMeasy 用の `fy_params` を定義します。
3. `HybridConfigData` を組み立て、選択理由・荷重・圧力・反復数などのメタ情報も詰めます。
4. `export_hybrid_config()` で YAML を保存します。
5. `export_cpp_header()` で C++ ヘッダを保存します。
6. 保存済みヘッダを再読込して中身を標準出力へ表示します。
7. 最後に C++ 側利用方法の例文を表示します。

## 4. 重要なデータ構造

- `ModelConfig`
  - モデル種別
  - パラメータ辞書
  - `FittingMetrics`
- `HybridConfigData`
  - FX/FY の個別設定
  - 選択モード
  - タイヤ名
  - 代表荷重/圧力
  - データ点数
  - 最適化手法と収束反復数

## 5. 副作用

- 出力ディレクトリの作成
- YAML/C++ ヘッダのファイル書き込み
- 生成済みヘッダの再読込

## 6. 注意点

- 実フィッティング結果を読むのではなく、設計書相当の固定値でデモします。
- 生成された C++ ヘッダの実 API 形状を人間が確認する用途が強いです。
