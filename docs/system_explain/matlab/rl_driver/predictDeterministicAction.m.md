# `matlab/rl_driver/predictDeterministicAction.m`

## 役割

`predictDeterministicAction` は、Actor の mean-path モデルに対して **サンプリング無しの deterministic 行動**を計算する関数です。  
SAC Actor は学習時には確率分布を持ちますが、この関数は deployment 相当の「平均経路だけを通す推論」を MATLAB 上で再現します。

---

## 処理の流れ

1. 観測 `obs` を `dlarray(obs, 'CB')` に変換する。
2. `predict(actorModel, ...)` を呼ぶ。
3. 戻り値が cell なら先頭要素を採用する。
4. `toNumeric` で `double` 化する。
5. 最後に `action(:)` で縦ベクトルへ整形する。

---

## 関数詳細

## `predictDeterministicAction(actorModel, obs)`

### 入力

- `actorModel`: `getDeterministicActorModel` 等で得た Actor モデル
- `obs`: 1 サンプル分の観測ベクトル

### `dlarray(obs, 'CB')` の意味

`'CB'` は Channel-Batch 形式を表します。  
この関数は 1 サンプル推論を想定しており、観測ベクトルを「1 バッチの特徴量列」として Actor に渡します。

### cell 戻り値への対応

Actor モデルによっては `predict` が cell を返すことがあります。  
この関数はその場合 `result{1}` を採用し、下流コードが返り値の型差を意識しなくて済むようにしています。

### 出力整形

最後に `action(:)` をかけるため、出力は必ず列ベクトルになります。  
行ベクトルや 1xN 配列のまま返さない点が、C++ 比較や MATLAB 内の後段処理で扱いやすい理由です。

---

## `toNumeric(x)`

`predict` の結果を `double` へ落とす helper です。

- `dlarray` なら `extractdata` → `double`
- それ以外も `double(x)` で変換

この helper により、GPU / dlarray / 通常配列といった出力差を吸収し、呼び出し側は常に数値配列を受け取れます。

---

## 想定用途

- export 前の Actor 応答確認
- MATLAB 側の deterministic 回帰テスト
- C++ 実装と mean-path 出力を比較するときの基準計算
