# `python/fsae_sim/data/ttc_parser.py`

## 1. 役割

TTC (FSAE Tire Test Consortium) の `.mat` データを読み、**標準化辞書化・簡易 Pacejka フィット・JSON 出力** まで行う小型ユーティリティです。

## 2. 入出力

- 実行例: `python3 python/fsae_sim/data/ttc_parser.py <input_dir> <output_dir>`
- 入力: TTC の `.mat` ファイル群
- 出力: `tire_params.json`

## 3. クラス `TTCDataParser`

### `__init__(self, data_dir: Path)`
- 役割: データディレクトリを保持します。
- 副作用: `Path` 化のみ。

### `load_mat_file(self, filename: str) -> Dict[str, np.ndarray]`
- 役割: `scipy.io.loadmat` で MAT ファイルを読み込み、標準化辞書へ変換します。
- 戻り値: `slip_angle`, `lateral_force`, `vertical_load` などの 1 次元配列辞書。

### `_extract_tire_data(self, mat_data: dict) -> Dict[str, np.ndarray]`
- 役割: TTC の典型チャンネル名を標準キーへ写像します。
- 変換例:
  - `SA -> slip_angle`
  - `FY -> lateral_force`
  - `FZ -> vertical_load`
- 副作用: なし。存在するチャンネルのみ採用します。

### `fit_pacejka(self, data: Dict[str, np.ndarray]) -> Dict[str, float]`
- 役割: 横力データに対して簡易 Magic Formula を当てます。
- 手順:
  1. `slip_angle`, `lateral_force`, `vertical_load` を抽出
  2. `Fz ≈ 1500N ±200N` でフィルタ
  3. 横力を `Fz_nominal` で正規化
  4. `curve_fit()` で `B, C, D, E` を最適化
- 戻り値: `B/C/D/E/Fz_nom` 辞書。失敗時は空辞書。
- 副作用: 例外時に失敗理由を print します。

### `export_to_json(self, params: Dict[str, Any], output_path: Path)`
- 役割: パラメータ辞書を JSON 保存します。
- 副作用: ファイル書き込み。

## 4. トップレベル関数

### `process_ttc_data(input_dir: str, output_dir: str)`
- 役割: ディレクトリ内の全 MAT ファイルを一括処理します。
- 流れ:
  1. 出力先ディレクトリ生成
  2. `*.mat` を列挙
  3. 各ファイルで読み込み・フィット
  4. 成功したものだけ `all_params` に格納
  5. `tire_params.json` として保存

## 5. CLI 挙動

- 引数が 2 つ未満なら使い方を表示するだけです。
- 本実装は横力中心の簡易フィットで、完全な TTC 全チャネル同定器ではありません。
