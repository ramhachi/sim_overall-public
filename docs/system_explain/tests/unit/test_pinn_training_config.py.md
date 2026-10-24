# tests/unit/test_pinn_training_config.py

## 役割
PINN 学習設定のタイヤ別上書きと、学習終了コード判定ポリシーを検証します。

## 守っている振る舞い
- `apply_tire_profile_overrides()` が MRF と Hoosier の既定レシピを正しくマージすること。
- デフォルトの `validation_targets.r2_min` が 0.90 に緩和されていること。
- 未知タイヤ名では設定を変えず `overrides is None` を返すこと。
- `resolve_training_exit_code()` が既定では非 deployable 終了を失敗コード 1 とし、探索用の許可モードだけ成功コード 0 を返すこと。

## 主なフィクスチャ/ヘルパー
- 実設定ファイル `tire_pinn/config/pinn_config.yaml` を毎回 `load_config()` で読み込みます。
- 一時ファイルやモックは使わず、純粋な辞書マージ結果だけを比較します。

## アサーションの意味
個々の値比較は、訓練 recipe の意図した差分が実際に反映されることの保証です。終了コード判定は CI やバッチ訓練の成否判定に直結するため、小さく見えて運用上重要です。
