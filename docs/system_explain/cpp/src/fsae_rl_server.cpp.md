# `cpp/src/fsae_rl_server.cpp`

## 1. ファイルの目的
14DOF 車両シミュレータを TCP サーバーとして公開し、MATLAB RL Toolbox など外部学習器から高速にステップ実行できるようにするファイルです。

## 2. 実行上の役割
- RL 学習時のオンライン環境本体です。
- バイナリプロトコルで初期化、ステップ、リセット、パラメータ更新を処理します。

## 3. 依存関係
- `fsae/vehicle/vehicle_14dof_new.hpp`
- `fsae/vehicle/vehicle_params.hpp`
- `fsae/track/track_model.hpp`
- `fsae/track/frenet.hpp`
- `fsae/track/track_distance.hpp`
- `fsae/track/waypoint_loader.hpp`
- `fsae/laptime/velocity_profile.hpp`
- POSIX socket API (`socket`, `bind`, `listen`, `accept`, `send`, `recv`)
- `nlohmann/json.hpp`

## 4. 入力/出力
- 入力:
  - 起動引数: ポート番号（既定 `12345`）
  - ソケットコマンド:
    - `CMD_INIT`
    - `CMD_STEP`
    - `CMD_RESET`
    - `CMD_PARAM`
    - `CMD_QUIT`
- 出力:
  - ソケット応答:
    - `RSP_OK`
    - `RSP_OBS`
    - `RSP_ERR`
  - 観測次元 `OBS_DIM=34`
  - 追加情報次元 `INFO_DIM=12`
  - 標準出力/標準エラーに接続、トラック初期化、障害情報

## 5. 関数・メソッド・クラスの網羅的な説明

### プロトコル定数
- `CMD_INIT`, `CMD_STEP`, `CMD_RESET`, `CMD_PARAM`, `CMD_QUIT`
- `RSP_OK`, `RSP_OBS`, `RSP_ERR`
- `OBS_DIM`, `INFO_DIM`, `N_SUBSTEPS`, `SIM_DT`, `CONTROL_DT`
- 役割: クライアントとの通信契約を固定します。

### `signal_handler(int)`
- `SIGINT` / `SIGTERM` 受信時に `g_running=false` とし、サーバーループを終了させます。

### `send_all(int fd, const void* data, size_t len)`
- 指定バイト数を送り切るまで `send()` を繰り返します。

### `recv_all(int fd, void* data, size_t len)`
- 指定バイト数を受け切るまで `recv()` を繰り返します。

### `send_double_array(int fd, const double* data, int count)`
- `double` 配列送信用の薄いラッパです。

### `recv_double_array(int fd, double* data, int count)`
- `double` 配列受信用ラッパです。

### `send_string(int fd, const std::string& s)`
- `[uint32_t length][bytes...]` 形式で文字列を送信します。

### `recv_string(int fd, std::string& s)`
- 上記形式の文字列を受信します。
- 100000 文字超は sanity check で拒否します。

### `class RLServer`
- 学習環境 1 セッション分の状態を保持する本体クラスです。
- 主メンバ:
  - `track_name_`, `params_`
  - `track_`, `frenet_`, `vehicle_`
  - `step_count_`, `prev_s_`, `prev_action_`, `lap_completed_`, `initial_speed_`

#### `RLServer::create_track(const std::string& name, double radius, double width)`
- トラック名から `CurvatureBasedTrack` を構築します。
- `circle_R<num>`、`circle_cw_R<num>` のパラメトリック指定にも対応します。
- `autocross` 系は環境変数 `FSAE_DATA_DIR` や既知パスから CSV を探します。
- 不明トラックや読込失敗時は円形トラックにフォールバックします。

#### `RLServer::init(const std::string& track_name, const std::string& params_json)`
- 車両パラメータ JSON を読み、トラック・Frenet 変換器・14DOF 車両を作成し、`reset_vehicle()` まで行います。

#### `RLServer::current_track_name() const`
- 現在のトラック名を返します。

#### `RLServer::reset_vehicle()`
- 車両状態と Frenet 変換器を初期化し、トラック始点へ再配置します。
- `prev_action_` や `step_count_` もリセットします。

#### `RLServer::step(double steer, double throttle, double brake)`
- 1 回の RL アクションを受け取り、20 個の 1 ms サブステップに分けて物理シミュレーションを進めます。
- 速度から `select_gear()` でギア選択します。
- 高ヨーレート時は安全のため自動スロットルカット/ブレーキ付与を行います。

#### `RLServer::build_observation(double obs[OBS_DIM])`
- 34 次元観測ベクトルを構築します。
- 内容:
  - 車両速度、ヨーレート、近似加速度、ロール角
  - 前回 action
  - 各輪スリップ比
  - 前後平均スリップ角
  - Frenet 横偏差、ヘディング誤差
  - 現在/先読み曲率
  - 左右余裕、進行率、先読みトラック幅
  - フレームねじり、ステアリングホイール状態

#### `RLServer::build_info(double info[INFO_DIM])`
- 報酬計算補助情報 12 次元を構築します。
- 速度、ヨーレート、Frenet 位置、左右幅、ロール角、車体スリップ角、最大スリップ角、トラック全長などを返します。

#### `RLServer::get_track_length() const`
- 現在トラック長を返します。

#### `RLServer::get_prev_s() const`
- 前回 `s` を返します。

#### `RLServer::set_prev_s(double s)`
- 前回 `s` を更新します。

#### `RLServer::set_prev_action(double s, double t, double b)`
- 前回 action を更新します。

#### `RLServer::select_gear(double speed) const`
- km/h ベースの閾値で 1〜6 速を選ぶ簡易ギアロジックです。

### `handle_client(int client_fd)`
- 1 クライアント接続分のコマンドループを処理します。
- `CMD_INIT` では track と params を受信し初期化、`track_length` を返します。
- `CMD_STEP` では action 受信→`step()`→観測と info をまとめて送信します。
- `CMD_RESET` では環境を再初期化して観測を返します。
- `CMD_PARAM` では現在トラックのままパラメータ再初期化します。
- `CMD_QUIT` では ACK 後に接続を閉じます。

### `main(int argc, char** argv)`
- ポート解釈、シグナル設定、ソケット生成、`SO_REUSEADDR/SO_REUSEPORT/TCP_NODELAY` 設定、`bind/listen/accept` を行います。
- クライアントごとに `handle_client()` を呼ぶ逐次処理サーバーです。

