# PINN Tire Model - クイックスタートガイド

## 📋 実装完了！

PINNタイヤモデルの実装が完了しました。以下の手順で学習を開始できます。

## 🚀 1分で始める

```bash
# 1. 依存関係のインストール（初回のみ）
pip3 install torch torchvision torchaudio
pip3 install numpy pandas pyyaml tqdm

# 2. 利用可能なタイヤモデルを確認
cd /Users/sota/projects/FomulaTMU/sim_overall
./tire_pinn/train.sh --list-tires

# 3. 学習開始（デフォルト: Hoosier 18.0×6.0）
./tire_pinn/train.sh

# または特定のタイヤモデルを指定
python3 tire_pinn/main_pinn_train.py --tire-model MRF_18p0X6p0-10
```

## 📂 実装されたファイル

### モデル (models/)
- ✅ `pinn_model.py` - TirePINN本体（8層、~165kパラメータ）
- ✅ `activations.py` - Swishアクティベーション

### データ (data/)
- ✅ `ttc_loader.py` - TTCタイヤデータローダー（正規化、分割）

### ロス関数 (losses/)
- ✅ `data_loss.py` - データロス + R²スコア
- ✅ `physics_losses.py` - 4つの物理制約
  - ゼロスリップ: Fx=Fy=0 at SR=SA=0
  - 対称性: Fy(-SA) = -Fy(SA) at IA=0
  - 摩擦円: √(Fx²+Fy²) ≤ μ*Fz
  - 単調性: ∂Fy/∂Fz ≥ 0

### 学習システム (training/)
- ✅ `trainer.py` - メイントレーナー（早期停止、チェックポイント）
- ✅ `collocation_sampler.py` - コロケーション点サンプリング
- ✅ `curriculum_scheduler.py` - 4段階カリキュラム学習

### 検証 (validation/)
- ✅ `physics_tests.py` - 物理制約の自動テスト

### エクスポート (export/)
- ✅ `cpp_exporter.py` - C++用高精度エクスポート（15桁）

### 設定
- ✅ `config/pinn_config.yaml` - ハイパーパラメータ
- ✅ `requirements.txt` - Python依存関係
- ✅ `train.sh` - ワンライナー実行スクリプト

## 🎓 カリキュラム学習スケジュール

### Phase A (Epoch 1-50): データロスのみ
- TTCデータでベースライン精度を確立
- 目標: R² > 0.95

### Phase B (Epoch 51-120): + ゼロスリップ + 対称性
- 重要な制約から導入
- ウェイト: zero 0→5, sym 0→2

### Phase C (Epoch 121-200): + 摩擦円 + 単調性
- すべての制約を有効化
- ウェイト: zero 5→10, sym 2→5, fric 0→2, mono 0→1

### Phase D (Epoch 201-260): 最終ウェイト
- 固定ウェイト: zero=10, sym=5, fric=2, mono=1
- ファインチューニング

## 📊 検証基準

| 項目 | 目標値 | 従来DNN |
|-----|-------|---------|
| R² (Fx, Fy) | ≥ 0.90 (`validation_targets.r2_min`) | 0.9945 |
| ゼロスリップ誤差 | < 2N | ~5N |
| 対称性誤差 | < 1% | ~2% |
| 摩擦円違反率 | < 2% | ~5% |
| 単調性違反 | 0% | ~3% |

## 🏁 利用可能なタイヤモデル

```bash
# 全タイヤモデルをリスト
python3 tire_pinn/main_pinn_train.py --list-tires
```

**完全なデータセット（推奨）**:
- ✅ `Hoosier_18p0X6p0-10_R20_Rim6` - Hoosier 18.0×6.0-10 R20、6インチリム（デフォルト）
- ✅ `Hoosier_18p0X6p0-10_R20_Rim7` - Hoosier 18.0×6.0-10 R20、7インチリム
- ✅ `Hoosier_18p0X6p0-10_LCO_C2000_Rim6` - Hoosier 18.0×6.0-10 LCO C2000、6インチリム
- ✅ `Hoosier_18p0X6p0-10_LCO_C2000_Rim7` - Hoosier 18.0×6.0-10 LCO C2000、7インチリム
- ✅ `MRF_18p0X6p0-10` - MRF 18.0×6.0-10
- ✅ `Goodyear_18p0X6p5-10` - Goodyear 18.0×6.5-10
- ✅ `Hoosier_20p5X7p0-13_R20` - Hoosier 20.5×7.0-13 R20
- ✅ `Goodyear_20p0X7p0-13` - Goodyear 20.0×7.0-13

**不完全なデータセット**:
- ❌ `Hoosier_16p0X6p0-10_R20_C2000` - Corneringのみ
- ❌ `Hoosier_16p0X7p5-10_R20_C2000` - Corneringのみ

リム幅は5D PINN入力に含まれないため、Hoosier TTCデータはRim6/Rim7を
別データセットとして扱います。ホイール径はタイヤサイズ末尾の `-10`
などに含めています。

## 💻 コマンド一覧

### タイヤモデル選択
```bash
# デフォルト（Hoosier 18.0×6.0）
python3 tire_pinn/main_pinn_train.py

# MRFタイヤで学習
python3 tire_pinn/main_pinn_train.py --tire-model MRF_18p0X6p0-10

# Goodyearタイヤで学習
python3 tire_pinn/main_pinn_train.py --tire-model Goodyear_20p0X7p0-13
```

### 学習オプション
```bash
# GPU指定
python3 tire_pinn/main_pinn_train.py --device cuda --tire-model MRF_18p0X6p0-10

# バッチサイズ変更
python3 tire_pinn/main_pinn_train.py --batch-size 1024

# 複数オプション組み合わせ
python3 tire_pinn/main_pinn_train.py \
    --tire-model Hoosier_20p5X7p0-13_R20 \
    --device cuda \
    --batch-size 2048 \
    --epochs 260

# 学習再開
python3 tire_pinn/main_pinn_train.py --resume output/pinn/checkpoint.pt
```

### 検証
```bash
# 物理制約テスト
python3 tire_pinn/validation/physics_tests.py output/pinn/best_model.pt
```

### C++エクスポート
```bash
# JSONエクスポート（C++シミュレーション用）
python3 tire_pinn/export/cpp_exporter.py \
    output/pinn/best_model.pt \
    cpp_export/pinn/
```

## 📈 学習時間の目安

- **GPU (RTX 3090)**: 2-3時間（260エポック）
- **CPU**: 10-15時間
- **推奨**: CUDA対応GPU

## 🐛 トラブルシューティング

### PyTorchがインストールできない
```bash
# CUDA 11.8の場合
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# CUDA 12.1の場合
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu121

# CPUのみの場合
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
```

### CUDA out of memory
```bash
# バッチサイズを小さくする
python3 tire_pinn/main_pinn_train.py --batch-size 256
```

### 学習が遅い
- コロケーション点の数を減らす（`config/pinn_config.yaml`編集）
- 単調性制約を無効化（`physics.monotonicity.enable: false`）

## 📝 実装済み機能

- ✅ TirePINNモデル（8層、Swish、LayerNorm）
- ✅ TTCデータローダー（DriveBrake + Cornering）
- ✅ 4つの物理制約ロス
- ✅ カリキュラム学習スケジューラー
- ✅ コロケーション点サンプリング
- ✅ 自動物理制約テスト
- ✅ C++エクスポート（15桁精度）
- ✅ 早期停止 + チェックポイント
- ✅ R²スコア計算
- ✅ すべての__init__.py

## 🔧 バグフィックス済み

1. **trainer.py line 129**: データロス計算の修正
2. **FrictionCircleLoss**: 正規化パラメータ設定の追加
3. **すべての__init__.py**: 適切なインポート追加

## 📚 次のステップ

1. **学習開始**: `./tire_pinn/train.sh`
2. **結果確認**: `output/pinn/training_history.json`
3. **物理制約テスト**: `python3 tire_pinn/validation/physics_tests.py ...`
4. **C++統合**: エクスポートしたJSONをC++シミュレータに読み込み

## 📧 サポート

詳細は以下のファイルを参照：
- `README.md` - 全体的な使い方
- `IMPLEMENTATION_STATUS.md` - 実装状況の詳細
- `config/pinn_config.yaml` - すべての設定項目

---

**ステータス**: ✅ すべて実装完了、テスト準備完了
**日付**: 2026-02-25
