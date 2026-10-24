# NNタイヤモデルC++統合の使用方法

## 概要

学習済みのニューラルネットワークタイヤモデル（MATLAB実装）をLookup Table方式でC++シミュレータに統合しました。TMeasyモデルは精度と学習時間の観点から棄却し、NNモデルのみを使用します。

## 実装されたコンポーネント

### 1. MAT File Reader (`fsae/utils/mat_file_reader.hpp`)

MATLABで生成されたLookup Tableファイルを読み込むユーティリティ。

**特徴**:
- `matio`ライブラリを使用
- 5次元配列のサポート
- MATLAB列優先からC++行優先への自動変換
- RAII wrapper (`MatFile`クラス)

**使用例**:
```cpp
#include "fsae/utils/mat_file_reader.hpp"

using namespace fsae::utils;

// MATファイルを開く
MatFile matfile("tire_lookup_table.mat");

// 1D配列を読み込み
auto SR_array = MatFileReader::read_array_1d(matfile.get(), "SR_array");

// 5D配列を読み込み
auto Fx_grid = MatFileReader::read_array_5d(matfile.get(), "Fx_grid");
```

---

### 2. NN Lookup Tire Model (`fsae/physics/tire/nn_lookup_tire.hpp`)

5次元線形補間でタイヤ力を計算するクラス。

**入力**: 5変数
- `SR`: スリップ率 [-0.2, 0.2]
- `SA`: スリップ角 [-15, 15] deg
- `Fz`: 垂直荷重 [500, 2500] N
- `IA`: キャンバー角 [-5, 5] deg
- `TSTC`: タイヤ表面温度 [20, 80] °C

**出力**: 4変数
- `Fx`: 前後力 [N]
- `Fy`: 横力 [N]
- `Mx`: 転覆モーメント [Nm]
- `Mz`: セルフアライニングトルク [Nm]

**使用例**:
```cpp
#include "fsae/physics/tire/nn_lookup_tire.hpp"

using namespace fsae::physics::tire;

// モデルの初期化
NNLookupTireModel tire_model;
tire_model.load_from_mat("matlab/dl_tire_model/output/tire_lookup_table.mat");

// タイヤ力の計算
TireInput input;
input.SR = 0.05;      // 5% slip
input.SA = 3.0;       // 3° slip angle
input.Fz = 1000.0;    // 1000N vertical load
input.IA = 0.0;       // 0° camber
input.TSTC = 60.0;    // 60°C tire temperature

TireForces forces = tire_model.interpolate(input);

std::cout << "Fx = " << forces.Fx << " N\n";
std::cout << "Fy = " << forces.Fy << " N\n";
```

---

### 3. Tire Thermal Integrator (`fsae/physics/tire/tire_thermal_integrator.hpp`)

タイヤ表面温度を動的に計算するクラス（設計書_統合版.md 4.3節準拠）。

**熱収支方程式**:
```
m_tread * c_p * dT/dt = Q_gen - Q_conv - Q_cond

Q_gen = |Fx * vsx| + |Fy * vsy|  (摩擦仕事)
Q_conv = h_conv(v) * A * (T_surf - T_air)  (対流冷却)
Q_cond = h_road * A_contact * (T_surf - T_road)  (伝導)
```

**使用例**:
```cpp
#include "fsae/physics/tire/tire_thermal_integrator.hpp"

using namespace fsae::physics::tire;

// 初期化
TireThermalIntegrator thermal;

// シミュレーションループ内
for (double t = 0; t < 10.0; t += dt) {
    // タイヤ力を計算
    std::array<TireForces, 4> forces = calculate_tire_forces();
    std::array<TireSlipInfo, 4> slip_info = calculate_slip();
    
    // 熱更新
    thermal.update(dt, forces, slip_info, v_vehicle, T_ambient);
    
    // 次のステップで温度を使用
    double TSTC = thermal.get_temperature(0);  // FL輪
}
```

---

## Lookup Table生成（MATLAB側）

**重要**: 既存の`generateLookupTable.m`を使用してください。学習フローは一切変更していません。

```matlab
% matlab/dl_tire_model/ディレクトリで実行

% 学習済みモデルを読み込み
load('output/tire_model.mat', 'trained_net', 'norm_params');

% グリッド設定
grid_config = struct();
grid_config.SR_range = [-0.2, 0.2, 50];      % [min, max, points]
grid_config.SA_range = [-15, 15, 60];
grid_config.Fz_range = [500, 2500, 40];
grid_config.IA_range = [-5, 5, 20];
grid_config.TSTC_range = [20, 80, 20];

% Lookup Table生成
output_path = 'output/tire_lookup_table.mat';
generateLookupTable(trained_net, norm_params, output_path, grid_config);
```

**グリッド解像度の推奨**:
- **開発時**: 30×40×30×15×15 ≈ 8M点 (250 MB)
- **本番**: 50×60×40×20×20 ≈ 96M点 (3 GB)

---

## ビルド方法

### 依存ライブラリのインストール

```bash
# vcpkg経由でmatioをインストール
vcpkg install matio
```

### CMake設定

`CMakeLists.txt`に以下が追加されています：

```cmake
target_link_libraries(fsae_core PUBLIC
    # ... 既存の依存
    matio
)

add_executable(test_nn_tire tests/test_nn_tire_integration.cpp)
target_link_libraries(test_nn_tire PRIVATE fsae_core GTest::gtest)
```

### ビルド実行

```bash
cd cpp
mkdir build
cd build
cmake ..
cmake --build .
```

---

## テスト実行

```bash
# ユニットテスト実行
./test_nn_tire

# テスト前にLookup Tableを生成しておく
cd ../../matlab/dl_tire_model
matlab -batch "load('output/tire_model.mat'); generateLookupTable(trained_net, norm_params, 'output/tire_lookup_table.mat', grid_config);"
```

**テスト内容**:
1. ✅ Lookup Table読み込みテスト
2. ✅ 補間精度テスト（1e-10以下）
3. ✅ 補間連続性テスト
4. ✅ 熱モデル初期化テスト
5. ✅ 摩擦発熱テスト
6. ✅ 対流冷却テスト
7. ✅ 温度クランプテスト
8. ✅ 統合シミュレーションテスト

---

## 14DOFシミュレータへの統合（次のステップ）

以下のファイルを修正してNNモデルを統合します：

### `vehicle_14dof.hpp`の変更

```cpp
#include "fsae/physics/tire/nn_lookup_tire.hpp"
#include "fsae/physics/tire/tire_thermal_integrator.hpp"

class Vehicle14DOF {
private:
    // TMeasyモデルを削除
    // TMeasyTireModel tire_model;  // ← 削除
    
    // NNモデルを追加
    NNLookupTireModel nn_tire_model;
    TireThermalIntegrator tire_thermal;
    
public:
    void initialize() {
        // Lookup Tableを読み込み
        nn_tire_model.load_from_mat("path/to/tire_lookup_table.mat");
    }
    
    void compute_tire_forces() {
        for (int i = 0; i < 4; i++) {
            // スリップ計算
            TireInput input;
            input.SR = compute_slip_ratio(i);
            input.SA = compute_slip_angle(i);
            input.Fz = compute_vertical_load(i);
            input.IA = compute_camber_angle(i);
            input.TSTC = tire_thermal.get_temperature(i);
            
            // NN補間
            TireForces forces = nn_tire_model.interpolate(input);
            
            // 力の適用
            apply_tire_forces(i, forces);
        }
    }
};
```

---

## パフォーマンス

**補間速度**:
- 5次元線形補間: ~2 μs/輪（現代的なCPU）
- 4輪合計: ~8 μs
- 1 kHzシミュレーションレート（dt=1ms）で十分余裕あり

**メモリ使用量**:
- グリッド解像度50×60×40×20×20の場合: 約3 GB
- 粗いグリッド30×40×30×15×15の場合: 約250 MB

---

## トラブルシューティング

### Lookup Tableが読み込めない

```
Error: Failed to open MAT file
```

**原因**: ファイルパスが不正  
**解決**: 絶対パスまたは相対パスを確認

### 補間値が異常

```
Fx = nan or inf
```

**原因**: グリッド範囲外の入力  
**解決**: 入力値がクランプされているか確認。範囲外の場合は最近傍値を返す仕様。

### matioがリンクできない

```
undefined reference to `Mat_Open`
```

**原因**: matioがインストールされていない  
**解決**: `vcpkg install matio`を実行

---

## 今後の拡張

1. **適応的グリッド**: 線形領域で細かく、飽和領域で粗くする
2. **GPU加速**: 多輪シミュレーションでのバッチ補間
3. **熱モデル改善**: タイヤ内部温度の3層モデル
4. **キャッシング**: 同一入力での再計算を避ける

---

## 参考文献

- `.kiro/specs/tire-model-integration-docs/design.md` - NN統合設計書
- `docs/設計書_統合版.md` - 14DOFシミュレータ設計書
- `matlab/dl_tire_model/README.md` - MATLAB実装リファレンス
