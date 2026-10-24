# `tire_pinn/train.sh`

## 1. 役割

PINN 学習 CLI を素早く起動するための **薄いシェルラッパ** です。  
Python/PyTorch の存在確認と `main_pinn_train.py` 呼び出しをまとめています。

## 2. 入出力

- 実行例:
  - `./tire_pinn/train.sh`
  - `./tire_pinn/train.sh --list-tires`
  - `./tire_pinn/train.sh --tire-model Hoosier_18p0X6p0-10_R20 --device cpu`
- 入力:
  - シェル引数 `"$@"`
  - `python3`, `torch`
- 出力:
  - 標準出力への環境確認ログ
  - `main_pinn_train.py` が生成する学習成果物

## 3. 処理の流れ

1. 冒頭でタイトルを表示します。
2. `--list-tires` / `-l` のときは `python3 tire_pinn/main_pinn_train.py --list-tires` を即実行して終了します。
3. `python3 --version` で Python を確認します。
4. `import torch` に失敗したらインストール方法を表示して終了します。
5. CUDA 利用可否を表示します。
6. `output/pinn/checkpoints` と `output/pinn/tensorboard` を作成します。
7. 残りの引数をそのまま `main_pinn_train.py` へ渡します。

## 4. システム内の位置付け

- 学習ロジック自体は持ちません。
- 開発者が毎回長い Python コマンドを書かずに済む補助入口です。

## 5. 副作用

- 出力ディレクトリを作成します。
- PyTorch 未導入時は exit 1 で終了します。
