# magic_formula.hpp

- **目的**: Pacejka MF5.2 ベースのタイヤ力・セルフアライニングトルク計算を提供します。
- **主要要素**: `MagicFormulaParams`、`compute_mf_Fx0/Fy0/Gxa/Gyk/Mz()`、`compute_magic_formula()`、パラメータローダ。
- **理論背景**: 純スリップと複合スリップを段階的に組み合わせる Magic Formula を TTC/SAE 符号で扱います。
- **依存**: `nn_lookup_tire.hpp`（`TireForces` 再利用）、`fstream`、`sstream`。
- **システム上の位置づけ**: TMeasy に代わる高忠実度タイヤモデル候補です。

