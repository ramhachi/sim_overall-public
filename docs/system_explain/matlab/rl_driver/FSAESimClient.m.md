# `matlab/rl_driver/FSAESimClient.m`

## 1. 役割

`FSAESimClient` は、MATLAB 側から Docker 内の C++ RL サーバーへ TCP 接続するための低レベル通信クラスです。  
RL Toolbox 用の環境 `FSAEEnvironment` はこのクラスを使って、サーバーの初期化、1 ステップ進行、リセット、車両パラメータ更新を行います。

この実装では MATLAB 標準の `tcpclient` ではなく `java.net.Socket` を直接使っています。狙いは、Nagle 無効化と Java NIO を使ったバイナリ転送で、学習ループ中の通信オーバーヘッドを抑えることです。

## 2. クラス全体の責務

- RL サーバーとの接続確立と切断
- 独自コマンドプロトコルの送受信
- 文字列・倍精度配列のバイナリ化
- 観測 `OBS_DIM=34` と補助情報 `INFO_DIM=12` の受信
- オブジェクト破棄時の後始末

## 3. 定数プロパティ

### コマンド種別

- `CMD_INIT = 0x01` : トラックと車両パラメータでサーバーを初期化
- `CMD_STEP = 0x02` : 1 制御周期分進める
- `CMD_RESET = 0x03` : エピソードをリセット
- `CMD_PARAM = 0x04` : 車両パラメータのみ更新
- `CMD_QUIT = 0xFF` : サーバーへ終了要求

### 応答種別

- `RSP_OK = 0x01` : 成功応答
- `RSP_OBS = 0x02` : 観測データ返却応答
- `RSP_ERR = 0xFF` : エラー応答を表す予約値

### データ寸法

- `OBS_DIM = 34` : サーバーが返す生観測ベクトル長
- `INFO_DIM = 12` : 報酬計算や終了判定に使う補助情報長

ここでの `34` はサーバーの観測契約に依存します。`FSAEEnvironment` 側ではさらに車両諸元 11 要素を後段で付加し、最終的に 45 次元観測へ正規化します。

## 4. インスタンスプロパティ

### 公開プロパティ

- `Host` : 接続先ホスト名。既定値は `localhost`
- `Port` : 接続先ポート。既定値は `22345` だが、コンストラクタで `get_rl_server_host_port()` を通して上書きされうる
- `Timeout` : ソケット読み取りタイムアウト秒数

### 非公開プロパティ

- `Socket` : `java.net.Socket` 本体
- `InputStream` / `OutputStream` : Java の入出力ストリーム
- `DataInputStream` / `DataOutputStream` : バイト単位送受信を行うラッパー
- `Connected` : 接続済みフラグ

## 5. ライフサイクル

1. コンストラクタで接続先を決める  
2. `connect()` で Java Socket を開く  
3. `init()` でトラックと車両設定をサーバーへ反映  
4. 学習中は `step()` / `reset()` / `setParams()` を繰り返す  
5. 終了時に `disconnect()`、またはオブジェクト破棄で `delete()` が走る

## 6. 公開メソッド詳細

### `FSAESimClient(host, port)`

接続先を初期化するコンストラクタです。

- 第 1 引数があれば `Host` を上書き
- 第 2 引数があれば `Port` を上書き
- 第 2 引数がなければ `get_rl_server_host_port()` で環境変数連動のポートを採用

まだソケットは開かず、実接続は `connect()` で行います。

### `connect(obj)`

Java ソケットを開いて通信路を準備します。

処理内容:

- 既に `Connected` なら何もしない
- `java.net.Socket(host, port)` を生成
- `setSoTimeout()` で受信タイムアウト設定
- `setTcpNoDelay(true)` で Nagle アルゴリズムを無効化
- 入出力ストリームと `Data*Stream` を構築
- 成功時に `Connected = true`

失敗時は `Java Socket Connect Failed` エラーを投げます。

### `disconnect(obj)`

サーバー終了を伴う通常切断です。

- `CMD_QUIT` を送信
- `flush()` 後に短い `pause(0.01)` を入れる
- 入出力ストリームとソケットを閉じる
- 最後に `Connected = false`

途中エラーは握りつぶして後始末を優先しています。

### `disconnectQuiet(obj)`

サーバーを落とさず、クライアント側だけ静かに切断します。  
`disconnect()` と違い `CMD_QUIT` を送らないため、サーバープロセスを存続させたいケース向けです。

### `trackLength = init(obj, trackName, paramsJson)`

サーバーを指定トラック・指定車両で初期化します。

手順:

1. 未接続なら `connect()`
2. `CMD_INIT` を送信
3. `trackName` を `sendString()` で送信
4. `paramsJson` を `sendString()` で送信
5. 応答が `RSP_OK` か確認
6. 続けて 1 個の `double` を読み、トラック全長として返す

`paramsJson` を省略すると空文字列が送られます。

### `[obs, info] = step(obj, steer, throttle, brake)`

1 ステップ分の制御入力をサーバーへ送ります。

- `CMD_STEP` を送信
- `steer, throttle, brake` の 3 値を `sendDoubleArray()` で送信
- 応答種別が `RSP_OBS` であることを確認
- `OBS_DIM + INFO_DIM` 個の `double` をまとめて受信
- 前半 34 要素を `obs`、後半 12 要素を `info` として返却

`FSAEEnvironment.step()` はこの `info` を使って報酬計算と終了判定を行います。

### `obs = reset(obj)`

サーバー側のエピソード状態をリセットします。

- `CMD_RESET` を送信
- 応答が `RSP_OBS` であることを確認
- 34 要素の生観測を受信して返す

### `setParams(obj, paramsJson)`

車両パラメータ JSON をサーバーへ送り、以後のシミュレーション条件を更新します。

- `CMD_PARAM` を送信
- `paramsJson` を `sendString()` で送信
- 応答が `RSP_OK` であることを確認

### `delete(obj)`

MATLAB のオブジェクト破棄時フックです。  
接続中なら `disconnect()` を呼び、ソケットを閉じます。

## 7. 非公開ヘルパー詳細

### `sendString(obj, s)`

文字列を「長さ 4 バイト + 本体バイト列」で送信します。

- 文字数を `uint32` 化
- `typecast(len, 'int8')` で 4 バイト化して送信
- 長さが正なら文字列本体を `int8` 配列化して続けて送信

コメントにもある通り、ここでは Java 標準のビッグエンディアン `writeInt()` ではなく、MATLAB 側ネイティブ順の 4 バイト列を送っています。つまりサーバー実装とエンディアン契約を合わせる前提です。

### `sendDoubleArray(obj, vals)`

`double` 配列を MATLAB ネイティブエンディアンのバイト列に `typecast` し、そのまま送信します。

### `readDoubleArray(obj, count)`

`count` 個の `double` を受信する中心ヘルパーです。

処理の流れ:

- 必要バイト数 `count * 8` を計算
- Java NIO の `ByteBuffer` を確保
- `nativeOrder()` を指定して MATLAB/ホスト側のバイト順に合わせる
- `Channels.newChannel(obj.InputStream)` で `ReadableByteChannel` 化
- 指定バイト数ぶん埋まるまで読み続ける
- `DoubleBuffer` として順に取り出して MATLAB 配列へ格納

途中でストリームが閉じた場合は `Stream closed unexpectedly` を投げます。

## 8. 実運用上のポイント

- 観測次元と応答フォーマットはサーバーと厳密同期が必要です。  
- `disconnect()` はサーバー停止を含むため、サーバーを残したいなら `disconnectQuiet()` を使います。  
- `sendString()` と `sendDoubleArray()` はネイティブバイト順前提なので、異種環境接続時は互換性確認が必要です。
