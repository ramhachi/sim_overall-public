# TCP通信版FEMソルバー 実装完了レポート

**実装日**: 2026-02-26
**目的**: フレーム最適化の高速化（Docker起動オーバーヘッド削減）
**結果**: ✅ **500倍高速化達成**

---

## 実装内容

### 1. C++ TCPサーバー (`beam_fem_server.cpp`)
- Docker内で常駐するTCPサーバー
- JSON over TCP（改行区切り）でリクエスト/レスポンス
- ポート: 50051
- 既存の`beam_fem_standalone.cpp`のロジックを再利用

**特徴:**
- 各リクエストを逐次処理（シングルスレッド）
- リクエスト処理後に接続を閉じる設計
- SHUTDOWNコマンドでサーバー終了

### 2. MATLAB TCPクライアント

#### 2.1 tcpclient版 (`beam_fem_solver_tcp.m`)
- MATLAB純正tcpclient関数を使用
- **問題**: レイテンシが大きい（~1.9ms/通信）
- **結果**: タイムアウトエラー多発

#### 2.2 **Java Socket版 (`beam_fem_solver_tcp_java.m`)** ✅ 推奨
- MATLAB内部Java Engine（`java.net.Socket`）を直接使用
- **レイテンシ**: 最小1.13ms、平均2.15ms
- **安定性**: 100回連続呼び出しで問題なし

---

## 性能比較

| 実装方式 | 計算時間/評価 | 速度比 | 備考 |
|---------|-------------|-------|------|
| **Docker版（従来）** | ~1000 ms | 1x | 毎回コンテナ起動 |
| **Java Socket版（新）** | **2.15 ms** | **500x** | 初回のみ2.6秒（起動） |

### 詳細結果（Java Socket版、100回テスト）
- **Mean**: 2.15 ms
- **Median**: 2.02 ms
- **Min**: 1.13 ms
- **Max**: 5.66 ms
- **Std**: 0.75 ms

---

## 使い方

### 基本的な使い方

```matlab
% データ準備
nodes = [...];  % Nx3
elements = [...];  % Mx9
bcs = [...];  % Bx3
loads = [...];  % Lx3

% FEM計算（初回はサーバー起動で約2秒）
result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads);

% 結果取得
mass = result.mass;         % 質量 [kg]
K_torsion = result.K_torsion;  % ねじり剛性 [Nm/deg]
U = result.U;               % 変位 [Nx6]

% 終了時
beam_fem_solver_tcp_java('shutdown');
```

### 最適化ループでの使用例

```matlab
% GA最適化（50個体×100世代）
for gen = 1:100
    for ind = 1:50
        % 各個体の評価（2.15ms/評価）
        result = beam_fem_solver_tcp_java(nodes, elements, bcs, loads);
        fitness(ind) = objective_function(result);
    end
end

% 終了時にサーバーをシャットダウン
beam_fem_solver_tcp_java('shutdown');
```

**期待性能:**
- 5000評価 × 2.15ms = **約11秒**（従来は1時間以上）

---

## 技術的詳細

### プロトコル

**リクエスト** (JSON + LF):
```json
{"command":"SOLVE","nodes":[[x,y,z],...], "elements":[...], "bcs":[...], "loads":[...]}
```

**レスポンス** (JSON + LF):
```json
{"success":true, "mass":9.34, "K_torsion":245.6, "U":[[...],...]}
```

### Java Socketの最適化

1. **TCP_NODELAY**: Nagleアルゴリズム無効化（低レイテンシ化）
2. **BufferedReader/PrintWriter**: 効率的なI/O
3. **接続管理**: 各リクエストで新規接続（サーバー側が接続を閉じるため）

### サーバーライフサイクル

```
[起動] docker compose run -d -p 50051:50051 fsae-sim beam_fem_server
  ↓
[待機] サーバーがポート50051でリッスン
  ↓
[処理] リクエストを受信 → FEM計算 → レスポンス送信 → 接続閉じる
  ↓  （繰り返し）
[終了] SHUTDOWNコマンド受信 → docker stop
```

---

## ファイル一覧

### C++
- `cpp/beam_fem_server.cpp` - TCPサーバー実装（NEW）
- `cpp/beam_fem_standalone.cpp` - 従来のスタンドアロン版
- `cpp/CMakeLists.txt` - ビルド設定（更新）

### MATLAB
- `matlab/beam_fem_solver_tcp_java.m` - **Java Socket版クライアント（推奨）**
- `matlab/beam_fem_solver_tcp.m` - tcpclient版（非推奨）
- `matlab/beam_fem_solver.m` - Docker版（従来）

### テスト
- `examples/test_tcp_java.m` - Java Socket版性能テスト
- `examples/test_tcp_simple.m` - 簡単なテスト
- `examples/test_tcp_performance.m` - 性能比較

### ビルド
- `bin/beam_fem_server` - サーバーバイナリ（281KB）
- `docker-compose.yml` - ポート50051マッピング追加

---

## 既知の制限事項

1. **サーバーは単一クライアントのみ対応**
   - 複数MATLABセッションからの同時接続は未対応
   - ポート50051が他のプロセスと衝突する可能性

2. **エラーハンドリング**
   - サーバークラッシュ時の自動再起動なし
   - ユーザーが手動で`beam_fem_solver_tcp_java('start')`を呼ぶ必要

3. **ポート12345との共存**
   - 強化学習サーバーがポート12345を使用中
   - FEMサーバーは50051を使用（衝突なし）

---

## 今後の改善案

### 短期（実装済みで十分な場合はスキップ可）
- [ ] サーバーのヘルスチェック機能
- [ ] 自動再起動機能

### 中期（最適化性能をさらに向上）
- [ ] 永続的接続のサポート（接続再利用で1ms未満に）
- [ ] バッチ処理（複数評価を1リクエストで送信）

### 長期（高度な機能）
- [ ] 並列計算サポート（複数サーバーインスタンス）
- [ ] GPU FEM実装（10倍高速化）

---

## まとめ

✅ **TCP通信版により500倍高速化を達成**
✅ **Java Socket版が最も高性能（平均2.15ms/評価）**
✅ **GA最適化が実用的な時間で実行可能に**

これにより、フレーム最適化の実用化への大きな一歩が踏み出されました。

---

**作成者**: Claude Sonnet 4.5
**プロジェクト**: Formula TMU FSAE Team
**実装方式**: MATLAB Java Socket + Docker C++ TCP Server
