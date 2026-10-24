# `tire_fitting/exporters/yaml_exporter.py`

## 役割

hybrid タイヤ設定の YAML 入出力を担当するモジュールです。  
新しい hybrid 形式の保存・読込に加えて、旧 TMeasy 専用 YAML を読み込んで hybrid 形式へ持ち上げる互換処理も持ちます。

## 実行と入出力

- 直接実行するファイルではありません。
- 主な入力:
  - `HybridConfigData`
  - YAML ファイルパス
  - 旧形式または新形式の YAML 内容
- 主な出力:
  - hybrid 形式 YAML
  - `HybridConfigData`
- 副作用:
  - 親ディレクトリ作成
- 例外:
  - `ValueError`: モデル種別・選択モード・パラメータ値が不正
  - `FileNotFoundError`, `yaml.YAMLError`, `PermissionError`

## 主要クラス

### `FittingMetrics`
- `rmse`, `tic`, `r2`, `peak_error` を保持します。

### `ModelConfig`
- 片方向の `model_type`, `parameters`, `metrics` を束ねます。

### `HybridConfigData`
- 縦横両方向の設定、選択理由、タイヤ情報、フィット統計を束ねる最上位 dataclass です。

## 主要関数

### `_convert_numpy_types(obj)`
- `numpy.ndarray`, `np.integer`, `np.floating` を YAML 向けの Python 型へ変換します。

### `_validate_config_data(config)`
- モデル種別、選択モード、有限値を検査します。

### `export_hybrid_config(config, filepath)`
- `model_config`, `longitudinal_force`, `lateral_force`, `tire_info`, `fitting_info` を持つ YAML を出力します。
- コメントも手書きで付与し、人間可読性を上げています。

### `import_hybrid_config(filepath)`
- ファイル存在確認と YAML 読込を行い、新形式か旧形式かを判定して下位パーサへ分岐します。

### `_parse_hybrid_format(yaml_data, filepath)`
- 新形式を `HybridConfigData` に復元します。

### `_parse_legacy_format(yaml_data, filepath)`
- 旧 TMeasy YAML の `longitudinal`, `lateral`, `camber`, `fitting_info` を読み、両方向 `tmeasy` の hybrid 設定へ変換します。

## データフロー

1. dataclass 群で設定を構造化します。
2. export 時は妥当性確認と `numpy` 型変換を行います。
3. import 時は YAML 形式を判定し、新旧どちらでも `HybridConfigData` に正規化します。

## システム内での位置づけ

hybrid 設定の **永続化・互換変換層** です。  
`main_hybrid.py` の成果物保存に加え、旧 TMeasy 資産を新しい hybrid 表現へ橋渡しする役割があります。
