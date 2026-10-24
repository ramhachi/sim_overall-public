# tests/unit/test_tire_pinn_runtime.py

## 役割
PINN 実行時ラッパと C++ エクスポータ、物理バリデータが後処理ルールを正しく適用するかを確認します。

## 守っている振る舞い
- `PhysicsEnforcedTireModel` がゼロスリップ近傍で Fx/Fy/Mz を 0 へ投影し、摩擦円上限を守ること。
- 対称性投影により、ほぼゼロの SR・IA 周りで左右反転整合が取れること。
- `CPPExporter._export_runtime_model()` が postprocess 設定を JSON に落とすこと。
- `export_test_vectors()` が指定した入力範囲を超えないこと。
- `PhysicsValidator.test_monotonicity()` が負勾配許容率と実データ由来の経験的基準を正しく使うこと。

## 主なフィクスチャ/ヘルパー
- `AffineStubModel`、`MildNegativeGradientModel`、`StrongNegativeGradientModel` が挙動を制御しやすい簡易 `torch.nn.Module` として使われます。
- 一部テストは `NormalizedTirePINN(TirePINN(...))` を最小構成で生成します。

## アサーションの意味
ゼロスリップ・摩擦上限制約の比較は、推論後処理が物理破綻を抑える最後の砦であることを示します。JSON と入力境界の確認は、C++ 実行系へ持ち出した際にも同じ制約が再現されるための保証です。
