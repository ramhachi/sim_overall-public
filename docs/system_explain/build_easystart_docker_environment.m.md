# `build_easystart_docker_environment.m`

## 役割

`build_easystart_docker_environment.m` は、EasyStart の `[1] 環境構築・更新` を実装する関数です。責務は、**Docker の稼働確認、ホスト環境に応じたビルドプロファイル選択、ポート確保、`docker compose` によるコンテナ更新、コンテナ内ビルド実行、失敗時の案内表示** を一括して行うことです。

この関数は単なる `docker compose up` ラッパではなく、ホスト CPU アーキテクチャ差分やポート競合も吸収しながら、EasyStart 利用者向けに安全な初期化体験を提供します。

---

## 運用上の入口

### `success = build_easystart_docker_environment()`
- 戻り値 `success` はビルド完了時のみ `true` になります。
- 冒頭で `success = false` とし、正常終了時だけ反転させる保守的設計です。
- `project_root` をこのファイルの配置ディレクトリから決定し、処理完了後は `onCleanup` により元ディレクトリと `DOCKER_DEFAULT_PLATFORM` を復元します。

---

## 全体フロー

1. 作業開始メッセージを表示。
2. `project_root`、現在ディレクトリ、旧 `DOCKER_DEFAULT_PLATFORM` を退避。
3. `resolve_easystart_docker_build_profile()` でビルドプロファイルを決定。
4. UI / RL / FEM 用ホストポートを `select_host_port()` で確保。
5. `docker info` を叩いて Docker 稼働確認。
6. 必要なら `DOCKER_DEFAULT_PLATFORM` をプロファイル値に設定。
7. 選ばれた target platform / CPU profile / generator / host ports を表示。
8. `docker compose up -d --build --remove-orphans` を実行。
9. `wait_for_service_ready('fsae-sim', 20)` でコンテナ応答待ち。
10. `/workspace/scripts/easystart_container_build.sh` をコンテナ内で実行。
11. 成功なら `success = true`。
12. 失敗時は、構成失敗かビルド失敗かを分けて復旧ヒントを表示。

---

## 関数詳細

### `build_easystart_docker_environment()` 本体

#### 主要な前処理
- `ispc()` で Windows / 非 Windows を分岐し、`docker info` のリダイレクト書式を切り替えます。
- `resolve_easystart_docker_build_profile()` が返す `profile` を用いて、
  - `docker_platform`
  - `cpu_optimization_profile`
  - `generator`
  - `build_signature`
  - `build_targets`
  を取得します。
- `select_host_port()` により、
  - `FSAE_HOST_UI_PORT`
  - `FSAE_HOST_RL_PORT`
  - `FSAE_HOST_FEM_PORT`
  を環境変数として確定させます。

#### Docker 稼働確認
- `docker info` が失敗した場合は、Docker Desktop 未起動とみなして即 return します。
- ビルド前に明確に停止することで、後続の compose エラーを利用者へ分かりやすくします。

#### `DOCKER_DEFAULT_PLATFORM` の扱い
- プロファイル側が空文字でなければ、実行中だけ設定します。
- 復元は `restore_environment(...)` により保証されます。
- これにより、EasyStart からの Docker 実行だけに target platform 指定を局所化できます。

#### コンテナ起動
- `docker compose up -d --build --remove-orphans` で、
  - 既存サービス更新
  - イメージ再ビルド
  - 不要 orphan コンテナ削除
  をまとめて行います。
- 失敗時は `docker compose logs` を案内します。

#### サービス応答待ち
- `wait_for_service_ready('fsae-sim', 20)` で、`docker compose exec -T ... bash -lc "true"` が成功するまで待ちます。
- 1 秒ごと最大 20 回試行するので、コンテナ内部シェルが実行可能になるまでの起動揺らぎを吸収できます。

#### コンテナ内ビルド
- `profile.build_targets` を空白区切りで連結。
- 以下の引数列を `/workspace/scripts/easystart_container_build.sh` へ渡します。
  1. `profile.build_signature`
  2. `profile.generator`
  3. `profile.cpu_optimization_profile`
  4. ビルドターゲット群
- これにより、ホスト依存の意思決定は MATLAB 側で完了し、コンテナ内スクリプトはその指示に従うだけの形になります。

#### エラー分岐
- `status == 0`: 完了。
- `status == 2`: CMake 設定失敗とみなし、`cmake -S /workspace -B . ...` の手動再現コマンドを提示。
- その他: 一般ビルド失敗とみなし、`cmake --build . --verbose` を案内。

---

## 補助関数

### `ready = wait_for_service_ready(service_name, max_attempts)`
- `docker compose exec -T <service> bash -lc "true"` が成功するまで待つ簡易ヘルスチェックです。
- 成功した時点で `true` を返し、最大試行数を超えると `false` です。
- Docker healthcheck に依存せず、実際に exec 可能かを見る点が実務的です。

### `restore_environment(original_dir, previous_docker_platform)`
- `onCleanup` から呼ばれる後始末関数です。
- 元ディレクトリへ戻し、`DOCKER_DEFAULT_PLATFORM` を元の値に戻します。
- 以前未設定なら空文字を設定して実質解除します。

### `value = display_value(primary_value, fallback_value)`
- 表示用の簡易関数です。
- `primary_value` が空なら `fallback_value` を返します。
- target platform や host arch 表示の見栄え調整に使われます。

### `port = select_host_port(env_name, preferred_port)`
- 環境変数がすでにあれば、その値を整数ポートとして再利用します。
- 無効値なら無視し、`find_available_tcp_port(preferred_port)` で空きポートを探します。
- 最終結果は `setenv(env_name, num2str(port))` で環境変数へ書き戻します。

#### 役割
外部 UI / RL / FEM サービスに必要なポートを、ユーザー指定優先・空き検知フォールバック付きで決める関数です。

### `port = find_available_tcp_port(preferred_port)`
- まず `can_bind_tcp_port(preferred_port)` を試し、空いていればそのまま採用します。
- 使えない場合は `java.net.ServerSocket(0)` を使って OS に任意空きポートを割り当てさせます。
- 取得後はソケットを閉じてポート番号だけ返します。

### `available = can_bind_tcp_port(port)`
- 指定ポートで `ServerSocket(port)` を開けるかを調べます。
- 開ければ `true`、例外なら `false`。
- 最後に必ずソケットを閉じるので、判定のためだけに一時利用する実装です。

---

## このファイルが解決している実務課題

1. **Docker 未起動問題の早期検出**
   - `docker compose` 失敗まで進む前に止めます。
2. **Apple Silicon / x86 などの差分吸収**
   - ビルドプロファイル関数と連携して `DOCKER_DEFAULT_PLATFORM` を制御します。
3. **ポート競合回避**
   - 既定値が埋まっていれば自動的に別ポートへ逃がします。
4. **MATLAB 実行後の環境汚染防止**
   - `onCleanup` でカレントディレクトリと Docker プラットフォーム設定を戻します。
5. **失敗時の次アクション明示**
   - 単なる失敗表示ではなく、次に打つべきコマンドまで示します。

---

## 依存関係

- `resolve_easystart_docker_build_profile.m`
- Docker CLI / Docker Compose
- コンテナ内 `/workspace/scripts/easystart_container_build.sh`
- Java Runtime（`ServerSocket` 使用のため）

---

## 読みどころ

このファイルは「Docker を呼ぶ」だけでなく、**EasyStart をホスト差分に強いものにするための運用ハブ** です。特に、`resolve_easystart_docker_build_profile()` との分担、`onCleanup` による環境復元、ポート確保の Java ベース実装が重要です。
