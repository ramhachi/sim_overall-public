# python/fsae_sim/data/ttc_parser.py

## 役割
MATLAB 形式の TTC データを標準キーへ変換し、簡易 Magic Formula パラメータを推定して JSON に書き出すツールです。

## 主なクラス・関数
- `TTCDataParser.load_mat_file()`: `scipy.io.loadmat` で MAT を読む
- `_extract_tire_data()`: TTC チャネル名を標準キーへ写像
- `fit_pacejka()`: 横力データから `B,C,D,E` を `curve_fit` で推定
- `export_to_json()`: 結果保存
- `process_ttc_data()`: ディレクトリ内 `.mat` を一括処理

## 背景・理論
Pacejka Magic Formula の簡易 4 パラメータ形を使い、基準荷重近傍の横力曲線を近似します。後段の高度なタイヤフィットほど厳密ではなく、初期解析・データ整理向けです。

## 使用ライブラリ
- `scipy.io`: MAT 読み込み
- `scipy.optimize`: 曲線フィット
- `numpy`, `json`, `pathlib`

## 入出力
- 入力: TTC `.mat` 群
- 出力: `tire_params.json`

## パイプライン上の位置
古い TTC データ資産を Python 解析系へ取り込む橋渡しです。
