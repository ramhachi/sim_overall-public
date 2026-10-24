# magic_formula.hpp

## ファイルの目的
- Pacejka Magic Formula 5.2 を使ったタイヤ力計算を提供します。

## 実行上の役割
- 純横力、純縦力、複合スリップ低減、セルフアライニングトルク、テキストパラメータ読込を 1 つのヘッダで完結させています。
- `TireForces` を返すため、他のタイヤモデルとの差し替え先としても使えます。

## 依存関係
- リポジトリ内: `fsae/physics/tire/nn_lookup_tire.hpp`（`TireForces` 使用）
- 外部: `<cmath>`, `<string>`, `<fstream>`, `<sstream>`, `<stdexcept>`, `<algorithm>`

## 入力/出力
- 入力: `MagicFormulaParams`, `kappa`, `alpha_rad`, `Fz`, `gamma_rad`
- 出力: `Fx`, `Fy`, `Mz` を含む `TireForces`、または個別係数・個別力

## 関数・メソッド・クラスの網羅的な説明
- `struct MagicFormulaParams`
  - 基本値 `Fz0`, `R0` を持ちます。
  - Stage 1 横力用: `pCy1`, `pDy1`, `pDy2`, `pDy3`, `pKy1`, `pKy2`, `pKy4`, `pEy1`, `pEy2`, `pEy3`, `pEy4`, `pHy1`, `pHy2`, `pVy1`, `pVy2`
  - Stage 2 縦力用: `pCx1`, `pDx1`, `pDx2`, `pDx3`, `pKx1`, `pKx2`, `pKx3`, `pEx1`, `pEx2`, `pEx3`, `pEx4`, `pHx1`, `pHx2`, `pVx1`, `pVx2`
  - Stage 3 複合スリップ用: `rBx1`, `rBx2`, `rCx1`, `rHx1`, `rEx1`, `rBy1`, `rBy2`, `rBy3`, `rCy1`, `rHy1`
  - Stage 4 Mz 用: `qBz1`, `qBz2`, `qCz1`, `qDz1`, `qDz2`, `qEz1`, `qHz1`, `qHz2`, `qBz9`, `qDz6`, `qDz8`, `qDz9`
  - すべて MATLAB フィッティング済み係数の受け皿です。
- `compute_mf_Fy0(...)`
  - 水平/鉛直シフト、形状係数、ピーク係数、コーナリング剛性、曲率係数を計算し、純横力 `Fy0` を返します。
- `compute_mf_Fx0(...)`
  - 純縦力 `Fx0` を同様の手順で返します。
- `compute_mf_Gxa(...)`
  - スリップ角による縦力低減係数 `Gxα` を返します。
  - `α=0` で 1 になるよう正規化しています。
- `compute_mf_Gyk(...)`
  - スリップ率による横力低減係数 `Gyκ` を返します。
- `compute_mf_Kya(...)`
  - Mz 計算の前処理としてコーナリング剛性 `Kyα` を返します。
- `compute_mf_Mz(...)`
  - ニューマチックトレイル成分と残留トルク成分を合成し、セルフアライニングトルクを返します。
  - `gamma` はシグネチャにはありますが、現実装では `[[maybe_unused]]` です。
- `compute_magic_formula(...)`
  - `Fy0/Fx0`、`Gxa/Gyk`、`Mz` をまとめて計算し、`TireForces{Fx,Fy,Mx=0,Mz}` を返します。
- `load_magic_formula_params(const std::string& filepath)`
  - `key:value` 形式のテキストから各係数を読み込み、`MagicFormulaParams` を構築します。
  - コメント行、空行、`[]` セクション行は無視します。
