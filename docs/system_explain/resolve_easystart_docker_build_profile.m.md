# `resolve_easystart_docker_build_profile.m`

## 役割

`resolve_easystart_docker_build_profile.m` は、EasyStart の Docker ビルド時に使う **ホスト依存プロファイル解決関数** です。ホスト CPU アーキテクチャを検出し、それに応じて以下をまとめて決定します。

- Docker target platform
- CPU 最適化プロファイル名
- CMake generator
- build signature
- 標準ビルドターゲット一覧

`build_easystart_docker_environment.m` はこの関数の結果をそのまま利用して、コンテナ起動とビルドを実行します。

---

## 運用上の入口

### `profile = resolve_easystart_docker_build_profile()`
- 生のホストアーキテクチャを `detect_raw_architecture()` で取得します。
- それを `normalize_architecture()` で `arm64` / `amd64` 系へ正規化します。
- 正規化結果に応じて Docker platform と CPU profile を選びます。
- 最後にビルドに必要な情報を 1 つの `struct` として返します。

---

## 処理フロー

1. `raw_arch = detect_raw_architecture()`
2. `normalized_arch = normalize_architecture(raw_arch)`
3. `switch normalized_arch` でプロファイル分岐
   - `arm64` → `linux/arm64`, `arm-native`
   - `amd64` → `linux/amd64`, `x86-native`
   - その他 → platform 未固定, `auto`
4. `platform_tag` を build signature 用に整形
   - 空なら `host-default`
   - `/` を `-` に変換
5. `profile` struct を返す

---

## 返却される `profile` の各フィールド

### `host_arch`
- 正規化後のホストアーキテクチャ名です。
- 代表値は `arm64` または `amd64`。

### `raw_host_arch`
- OS / コマンドから直接得た生文字列です。
- 例: `aarch64`, `maca64`, `x86_64` など。

### `docker_platform`
- Docker に明示指定する target platform。
- `arm64` の場合は `linux/arm64`。
- `amd64` の場合は `linux/amd64`。
- 不明時は空文字で、Docker の host default に委ねます。

### `cpu_optimization_profile`
- CMake 側やビルドスクリプト側に渡す CPU 最適化プロファイル名です。
- `arm-native` / `x86-native` / `auto` のいずれか。

### `generator`
- CMake generator 名。
- 現状は固定で `Ninja` です。

### `build_signature`
- ビルド成果物や構成差を識別するための署名文字列です。
- 形式:
  - `<platform-tag>__<cpu-profile>__release__ninja`
- 例:
  - `linux-arm64__arm-native__release__ninja`
  - `linux-amd64__x86-native__release__ninja`
  - `host-default__auto__release__ninja`

### `build_targets`
- EasyStart 標準ビルドで要求するターゲット一覧です。
- 含まれるもの:
  - `laptime_sim`
  - `test_14dof_new`
  - `test_velocity_profile`
  - `test_aero_integration`
  - `test_aero_map_4d`
  - `test_lsd`
  - `fsae_rl_server`
  - `rl_actor_check`

この構成から、EasyStart の初期ビルドが「単に本番バイナリを 1 つ作る」だけでなく、主要テストや RL サーバー関連もまとめて準備する前提であることが分かります。

---

## 補助関数

### `raw_arch = detect_raw_architecture()`
ホストからアーキテクチャ文字列を取得する関数です。

#### 分岐
- Windows (`ispc`):
  - `PROCESSOR_ARCHITECTURE`
  - 空なら `PROCESSOR_ARCHITEW6432`
- それ以外:
  - `system('uname -m')`
- どちらでも取得できない場合:
  - `computer('arch')`

#### 役割
複数 OS でアーキテクチャ取得方法が異なる問題を吸収し、後段正規化の入力を 1 つに揃えます。

### `normalized_arch = normalize_architecture(raw_arch)`
生アーキ文字列を EasyStart 用の代表カテゴリへ正規化します。

#### 正規化ルール
- `arm64`, `aarch64`, `maca64` → `arm64`
- `x86_64`, `amd64`, `maci64`, `glnxa64`, `win64` → `amd64`
- その他 → 小文字化した元値をそのまま返す

#### 意味
MATLAB、Windows 環境変数、`uname -m` がそれぞれ異なる名前を返しても、EasyStart では 2 系統へ整理して扱えます。

---

## 設計上のポイント

### 1. platform 未知時に壊さない
未知アーキではエラーにせず、
- `docker_platform=''`
- `cpu_profile='auto'`
とし、Docker / ビルドスクリプト側の既定に委ねます。つまり「分からないなら止める」ではなく「可能な限り前進する」設計です。

### 2. プロファイル決定を 1 箇所へ集約
ホスト判定ロジックを `build_easystart_docker_environment.m` に埋め込まず、この関数へ分離しています。これにより、Docker ビルド条件の変更はここだけを見ればよくなっています。

### 3. build signature を生成する
単に platform を返すだけでなく、release / generator / CPU profile を含む署名文字列まで作るため、コンテナ内ビルドスクリプトやキャッシュ戦略と連携しやすい設計です。

---

## このファイルが支えている運用

- Apple Silicon と x86 系ホストの差分吸収
- Docker target platform の明示制御
- CMake / CPU 最適化設定の統一
- EasyStart 標準ビルドターゲットの中央管理

---

## 読みどころ

短い関数ですが、EasyStart の Docker ビルドを **ホスト依存差分に対して安定化する中核部品** です。特に `normalize_architecture()` と `build_targets` の定義は、どのホストで何をビルドするかという運用ポリシーそのものを表しています。
