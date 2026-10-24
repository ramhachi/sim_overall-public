# `frame_optimize/cpp/beam_fem_server.cpp`

## 役割

- `beam_fem_standalone` と同系統の FEM コアを、JSON over TCP で提供するサーバ実装です。
- MATLAB/Java/Python などから高速に繰り返し呼ばれることを想定しています。

## サーバ全体の流れ

- ポートを開いて待受します。
- 受信したクライアントソケットをキューへ積みます。
- ワーカースレッドが要求 JSON を取り出して処理します。
- `SOLVE`、`PING`、`SHUTDOWN` に応じた応答を返します。

## グローバル要素

- `log_mutex`, `LOG`, `ERR`
  - スレッド安全なログ出力
- `should_shutdown`
  - サーバ停止フラグ
- `task_queue`, `queue_mutex`, `queue_cv`
  - クライアントソケットの処理待ちキュー

## データ構造

### `BeamElement`

- スタンドアロン版と同じく、接続・断面・物性・長さ・回転行列を持つ要素構造です。

## 関数詳細

### `send_all(socket_handle_t socket_fd, const std::string& message)`

- **入力**
  - ソケット、送信文字列
- **出力**
  - 成否
- **責務**
  - 部分送信を考慮して最後まで送ります。

### `computeSectionProperties`

- スタンドアロン版と同じく断面特性計算を担当します。

### `computeRotationMatrix`

- スタンドアロン版と同じく局所座標基底を作ります。

### `computeLocalStiffness`

- スタンドアロン版と同等の要素剛性を返します。

### `transformToGlobal`

- ローカル剛性を全体座標へ写像します。

### `solve_fem(const json& input)`

- **入力**
  - `nodes`, `elements`, `bcs`, `loads` を持つ JSON
- **出力**
  - 結果 JSON
- **責務**
  - FEM コア全体を 1 関数にまとめており、TCP サーバ側の主処理です。
- **主要ロジック**
  - JSON 解析
  - 要素データ化
  - BFS による接続性確認
  - 疎行列求解
  - ねじり剛性・質量計算
- **注意点**
  - `Eigen::setNbThreads(1)` を使い、外側スレッドとの過剰並列を避けています。

### `main(int argc, char* argv[])`

- **入力**
  - 任意のポート番号
- **責務**
  - ソケット初期化、待受、ワーカ起動、接続受理、終了処理を管理します。
- **対応コマンド**
  - `SOLVE`
  - `PING`
  - `SHUTDOWN`

## 依存関係

- `Eigen`
- `nlohmann_json`
- 標準スレッドライブラリ
- POSIX socket または Winsock

## 前提・注意点

- newline 区切り JSON を前提にしています。
- キューと 16 ワーカによる簡易並列で、厳密なリクエスト再試行や認証はありません。
- `SHUTDOWN` を受けると全体停止するため、共有環境では利用者の調整が必要です。
