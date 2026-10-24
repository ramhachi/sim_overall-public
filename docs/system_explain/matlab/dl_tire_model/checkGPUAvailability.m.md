# `matlab/dl_tire_model/checkGPUAvailability.m`

## 1. 概要

`checkGPUAvailability` は、Parallel Computing Toolbox の有無、GPU 台数、GPU 初期化可否を確認し、学習で GPU を使えるかどうかと GPU 情報構造体を返す関数です。

## 2. ファイルの役割

- GPU 学習に必要なライセンス有無を確認します。
- `gpuDeviceCount` と `gpuDevice` を使って GPU を列挙・初期化します。
- 利用不能時は CPU フォールバック前提のメッセージを出し、空構造体を返します。

## 3. インターフェース

### 3.1 関数シグネチャ

```matlab
[gpu_available, gpu_info] = checkGPUAvailability()
```

### 3.2 戻り値

- `gpu_available`
  - GPU 利用可否を表す論理値です。
- `gpu_info`
  - GPU 利用可能時に以下を持つ構造体です。
    - `Name`
    - `Index`
    - `ComputeCapability`
    - `TotalMemory`
    - `AvailableMemory`
  - 利用不可時は空の `struct()` です。

## 4. 依存関係

- MATLAB 標準関数: `license`, `gpuDeviceCount`, `gpuDevice`, `fprintf`, `sprintf`
- Deep Learning 側の学習関数は呼ばず、GPU 可用性判定だけを担当します。

## 5. 処理の流れ

1. 冒頭で GPU 検出開始メッセージを表示します。
2. `license('test', 'Distrib_Computing_Toolbox')` で Parallel Computing Toolbox の有無を確認します。
3. ライセンスがなければ `gpu_available = false`, `gpu_info = struct()` を返して終了します。
4. `gpuDeviceCount` を `try/catch` で呼び、例外時も CPU 利用前提で終了します。
5. GPU 台数が 0 の場合は GPU なしとして終了します。
6. GPU がある場合は `gpuDevice` でデバイスを取得し、名前・インデックス・計算能力・総メモリ・空きメモリを格納します。
7. 初期化に失敗した場合は警告を出し、`false` と空構造体に戻します。

## 6. 関数・ローカル関数詳細

### 6.1 `checkGPUAvailability`

- ライセンス確認とデバイス初期化確認を分けているため、ライセンス不足とデバイス初期化失敗を別メッセージで扱えます。
- `ComputeCapability` は `gpu_device.ComputeCapability` の 2 要素を `'%d.%d'` 形式で文字列化しています。
- メモリは `1e9` で割って GB 表示に変換しています。

## 7. 注意点

- 戻り値だけでなく `fprintf` によるコンソール出力も多く、診断用関数として設計されています。
- 複数 GPU の選択は行わず、`gpuDevice()` が返す現在のデフォルトデバイスだけを扱います。
- `gpu_info` の各値は取得時点の状態であり、後続処理中のメモリ変化は反映しません。
