# `matlab/rl_driver/getDefaultRLPIDBaselineLapTimes.m`

## 1. 役割

`getDefaultRLPIDBaselineLapTimes` は、RL readiness 判定で使う **既定の PID ラップタイム基準値** を返すだけの小さな定数提供関数です。  
特に `validateRLReadiness` や `runRLReadinessGate` で、RL の中央値ラップタイムを PID 実績と比較したいときの基準源として使われます。

---

## 2. 入口関数 `baselines = getDefaultRLPIDBaselineLapTimes()`

### 2.1 入力

入力引数はありません。

### 2.2 出力

戻り値 `baselines` は struct で、既定では次の 3 フィールドを持ちます。

- `circle = 15.684`
- `skidpad = 13.014`
- `autocross = 60.010`

単位はコメントと文脈から秒です。

### 2.3 データの由来

ファイル冒頭コメントに、値の出典レポートが明記されています。

- `circle`
  - `report_circle_20260216_202544.txt`
- `skidpad`
  - `report_skidpad_20260302_010340.txt`
- `autocross`
  - `report_ASE_autocross_20260221_161143.txt`

つまりこの関数は、手計算や外部設定読込ではなく、**検証済みレポートから固定値をコード化した基準表** です。

---

## 3. 処理内容

実装は単純で、`struct(...)` を 1 回組み立てて返すだけです。  
計算・I/O・外部依存はなく、実行コストは極小です。

この単純さには意味があります。

- readiness 判定の比較基準を毎回同じにできる
- 実行環境に依存せず再現性が高い
- ゲート処理からすぐ参照できる

---

## 4. このファイルのシステム上の位置付け

このファイルは RL readiness 系における **基準値カタログ** です。  
判定ロジック本体は持ちませんが、比較対象がなければ readiness gate の意味が弱くなるため、運用上は重要です。

役割を分けると次の通りです。

- `getDefaultRLPIDBaselineLapTimes`
  - 比較基準を供給
- `validateRLReadiness`
  - RL 実測値を集計し基準と比較
- `runRLReadinessGate`
  - 全工程を束ねて合格可否や publish を判断

---

## 5. 注意点

- 基準値は固定値なので、車両仕様・タイヤ・トラック定義が変わった場合は見直しが必要です。
- フィールド名は readiness 系が期待する `circle/skidpad/autocross` に揃えられています。
- ローカル helper は存在せず、このファイルの実体は 1 関数のみです。
