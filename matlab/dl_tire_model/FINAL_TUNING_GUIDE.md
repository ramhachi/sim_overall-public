# 最終チューニングガイド

## 現状分析

### Simple版の結果（最良）
```
Fx: RMSE=117.8N, R²=0.9865 ✓
Fy: RMSE=126.1N, R²=0.9871 ✓
Mx: RMSE=19.2Nm, R²=0.8390 (改善必要)
Mz: RMSE=22.8Nm, R²=0.5401 (改善必要)
```

### Deep版の結果（悪化）
```
Fx: RMSE=118.7N, R²=0.9863
Fy: RMSE=206.7N, R²=0.9653 ✗
Mx: RMSE=13.1Nm, R²=0.9252 ✓
Mz: RMSE=23.2Nm, R²=0.5240
```

**結論**: 標準モデルの方が良い → さらにチューニングを詰める

---

## 実施した改善

### 1. データ分割の最適化
- **変更前**: 80/20 split
- **変更後**: 90/10 split
- **理由**: データ数が少ないため、より多くを学習に使用

### 2. Simple版のパラメータ最適化

**更新されたパラメータ:**
```matlab
max_epochs: 200 → 300
batch_size: 256 → 512
learning_rate: 0.001 → 0.0015
patience: 15 → 25
val_frequency: 30 → 50
```

### 3. 超深層モデルの作成（逆張り戦略）

**新規作成**: `createVeryDeepTireModel.m`

**アーキテクチャ:**
```
Input (5) 
  → 256 (BN + ReLU + Dropout 0.2)
  → 256 (BN + ReLU + Dropout 0.2)
  → 128 (BN + ReLU + Dropout 0.2)
  → 128 (BN + ReLU + Dropout 0.2)
  → 64 (BN + ReLU + Dropout 0.15)
  → 64 (BN + ReLU + Dropout 0.15)
  → 32 (BN + ReLU + Dropout 0.1)
  → 32 (BN + ReLU)
  → Output (4)
```

**特徴:**
- 8層の隠れ層
- Batch Normalization全層
- 段階的なDropout減少
- 約20万パラメータ

---

## 実行方法

### オプション1: 最適化Simple版（推奨）⭐

```matlab
cd matlab/dl_tire_model
run_simple_validation
```

**改善点:**
- 90/10データ分割
- 300エポック
- バッチサイズ512
- 学習率0.0015

**予想時間:** 20-40分  
**予想結果:**
```
Fx: RMSE < 100N ✓
Fy: RMSE < 110N ✓
Mx: R² > 0.85 ✓
Mz: R² > 0.60 (改善)
```

---

### オプション2: 超深層版（実験的）🚀

```matlab
cd matlab/dl_tire_model
run_ultra_deep_validation
```

**特徴:**
- 8層の超深層ネットワーク
- Batch Normalization
- 400エポック
- 最適化された学習率

**予想時間:** 30-60分  
**予想結果:** 不明（実験的）

**目的:** 
- 深層化で性能向上するか検証
- Mz R²の改善を狙う

---

## 推奨実行順序

### ステップ1: 最適化Simple版
```matlab
run_simple_validation
```

**期待される改善:**
- Fx/Fy RMSE: さらに低下
- Mx R²: 0.85以上
- Mz R²: 0.60以上

### ステップ2: 結果が不十分なら超深層版
```matlab
run_ultra_deep_validation
```

### ステップ3: 比較
```matlab
% Simple版
load('output/simple_validation_model.mat')
disp(metrics)

% Ultra Deep版
load('output/ultra_deep_validation_model.mat')
disp(metrics)
```

---

## パラメータ比較表

| パラメータ | 元Simple | 最適化Simple | Ultra Deep |
|-----------|---------|------------|-----------|
| データ分割 | 80/20 | 90/10 | 90/10 |
| エポック数 | 200 | 300 | 400 |
| バッチサイズ | 256 | 512 | 512 |
| 学習率 | 0.001 | 0.0015 | 0.002 |
| Patience | 15 | 25 | 30 |
| 隠れ層数 | 3 | 3 | 8 |
| Batch Norm | なし | なし | あり |
| 予想時間 | 15-30分 | 20-40分 | 30-60分 |

---

## 期待される最終結果

### 最適化Simple版
```
目標:
  Fx: RMSE < 100N ✓
  Fy: RMSE < 110N ✓
  Mx: R² > 0.85 ✓
  Mz: R² > 0.60 ✓
  Speed: < 1ms ✓
```

### Ultra Deep版（ベストケース）
```
目標:
  Fx: RMSE < 95N ✓
  Fy: RMSE < 105N ✓
  Mx: R² > 0.90 ✓
  Mz: R² > 0.70 ✓
  Speed: < 1ms ✓
```

---

## トラブルシューティング

### メモリ不足
```matlab
% バッチサイズを減らす
train_config.batch_size = 256;  % 512から変更
```

### 過学習の兆候
- Validation lossが上昇
- Early stoppingが機能しているか確認
- Dropoutを増やす

### 学習が遅い
- GPUが使用されているか確認
- バッチサイズを増やす（メモリが許せば）

---

## さらなる改善のヒント

### 1. アンサンブル学習
複数のモデルを訓練して平均：
```matlab
% 3つのモデルを訓練
model1 = run_simple_validation;
model2 = run_simple_validation;  % 異なるseed
model3 = run_simple_validation;

% 予測を平均
Y_pred = (predict(model1, X) + predict(model2, X) + predict(model3, X)) / 3;
```

### 2. 出力別モデル
Mz専用のモデルを作成：
```matlab
% Mzだけを予測する専用モデル
% より深いネットワークを使用
```

### 3. フルデータセット
サンプル数を増やす：
```matlab
data_paths = {
    '../../data/tiredata/DriveBrake-Hoosier18.0×6.0-10-R20.csv'
    '../../data/tiredata/Cornering-Hoosier18.0×6.0-10-R20.csv'
};
```

---

## 作成されたファイル

1. ✅ `run_simple_validation.m` - 最適化済み
2. ✅ `createVeryDeepTireModel.m` - 超深層モデル
3. ✅ `run_ultra_deep_validation.m` - 超深層版検証
4. ✅ `FINAL_TUNING_GUIDE.md` - このガイド

---

## 実行コマンドまとめ

```matlab
% ディレクトリ移動
cd matlab/dl_tire_model

% オプション1: 最適化Simple版（推奨）
run_simple_validation

% オプション2: 超深層版（実験的）
run_ultra_deep_validation

% 結果比較
load('output/simple_validation_model.mat')
fprintf('Simple: Fx=%.1f, Fy=%.1f, Mx R²=%.3f, Mz R²=%.3f\n', ...
        metrics.Fx.RMSE, metrics.Fy.RMSE, metrics.Mx.R2, metrics.Mz.R2);

load('output/ultra_deep_validation_model.mat')
fprintf('Ultra: Fx=%.1f, Fy=%.1f, Mx R²=%.3f, Mz R²=%.3f\n', ...
        metrics.Fx.RMSE, metrics.Fy.RMSE, metrics.Mx.R2, metrics.Mz.R2);
```

---

## 次のステップ

最良の結果が得られたら：

1. **モデルの保存**
2. **性能レポート作成**
3. **C++エクスポート**
4. **本番環境デプロイ**

---

**重要**: 
- まず最適化Simple版を試してください
- 結果が良ければそれを使用
- さらに改善したい場合のみUltra Deep版を試す
- 両方の結果を比較して最良のものを選択
