# タイヤモデル調整タスク

## 作成日
2026-01-26

## 目的
既存のシミュレーションにおけるタイヤモデルを、tire-model-integration-docsとdocs配下の設計書に基づいて調整する。
既存のNNベースの学習フローは維持し、TMeasyモデルとの統合構造を整備する。

## 参照ドキュメント
1. `.kiro/specs/tire-model-integration-docs/design.md` - NN統合設計書
2. `.kiro/specs/tire-model-integration-docs/requirements.md` - NN統合要件
3. `docs/タイヤフィッティング設計書.md` - TMeasyパラメータフィッティング
4. `docs/設計書_統合版.md` - 14DOFシミュレータ統合設計書

## 現状分析

### 既存実装
- **TMeasyモデル**: `cpp/include/fsae/physics/tire/tmeasy.hpp`
  - 基本的なTMeasy実装は完成
  - 緩和長は固定値
  - 熱モデルなし
  - 荷重依存性は簡易的（線形）

- **NNモデル**: `matlab/dl_tire_model/`
  - 約140,000サンプルのTTCデータで学習済み
  - 5入力（SR, SA, Fz, IA, TSTC）→ 4出力（Fx, Fy, Mx, Mz）
  - エクスポート機能完成
  - **この部分は触らない**

### 課題
1. タイヤ熱モデルが未実装
2. 動的緩和長が未実装
3. 荷重依存性のモデル化が不十分
4. TMeasyとNNモデルの切り替え機構がない

## 調整内容

### Phase 1: タイヤ熱モデルの追加
**ファイル**: `cpp/include/fsae/physics/tire/tire_thermal.hpp` (新規作成)

設計書_統合版.md の4.3節に基づいて、以下を実装:
- タイヤ表面温度の動的計算
- 摩擦仕事による発熱
- 対流・伝導冷却
- 温度依存の摩擦係数

**数式**:
```
m_tread * c_p * dT_surf/dt = Q_gen - Q_conv - Q_cond
Q_gen = |Fx * vsx| + |Fy * vsy|
Q_conv = h_conv * A_tread * (T_surf - T_air)
mu(T) = mu_peak * exp(-(T - T_opt)^2 / (2 * T_width^2))
```

### Phase 2: 動的緩和長の実装
**ファイル**: `cpp/include/fsae/physics/tire/tmeasy.hpp` (更新)

設計書_統合版.md の4.2節に基づいて:
```cpp
// 緩和長の荷重依存性
Scalar compute_relaxation_length(Scalar Fz, Scalar C_alpha, Scalar K_lat) {
    return C_alpha * Fz / K_lat;
}
```

タイヤフィッティング設計書.md の4.2.2節で定義されている:
- コーナリングスティフネスの荷重依存性: `C_y(Fz) = C_y0 + C_y1 * Fz`

### Phase 3: 荷重依存性の改善
**ファイル**: `cpp/include/fsae/physics/tire/tmeasy.hpp` (更新)

タイヤフィッティング設計書.md の4.2節に基づいて:
```cpp
// 摩擦係数の荷重依存性（現在の線形から非線形へ）
Scalar mu_x(Fz) = mu_x_peak * (1 - k_mu_x * (Fz - Fz_nom) / Fz_nom);
Scalar mu_y(Fz) = mu_y_peak * (1 - k_mu_y * (Fz - Fz_nom) / Fz_nom);
```

### Phase 4: タイヤモデル統合インターフェースの作成
**ファイル**: `cpp/include/fsae/physics/tire/tire_model_interface.hpp` (新規作成)

tire-model-integration-docs/design.md の3節に基づいて:
- TMeasyとNNモデルを統一的に扱う抽象インターフェース
- ランタイムでの切り替え機能
- Lookup Table補間機能（NN統合用）

```cpp
class ITireModel {
public:
    virtual TireForces compute(const TireInput& input) = 0;
    virtual void update_state(double dt) = 0;
};

class TMeasyTireModel : public ITireModel { /*...*/ };
class NNLookupTireModel : public ITireModel { /*...*/ };
```

### Phase 5: パラメータ構造の拡張
**ファイル**: `cpp/include/fsae/physics/tire/tire_params.hpp` (新規作成)

タイヤフィッティング設計書.md の13.2節に基づいて、全パラメータを統合:
```cpp
struct TireParams {
    // 基本パラメータ
    double mu_x_peak;
    double mu_y_peak;
    double Fz_nom;
    
    // 荷重依存パラメータ
    double k_mu_x;
    double k_mu_y;
    double C_y0, C_y1;  // 剛性の荷重依存
    
    // 緩和長パラメータ
    double Lx_nom, Ly_nom;
    double K_lat;  // 横剛性
    
    // 熱パラメータ
    double T_opt;      // 最適温度
    double T_width;    // 温度幅
    double m_tread;    // トレッド質量
    double c_p;        // 比熱
    double h_conv;     // 熱伝達係数
};
```

## 実装順序

1. **タイヤ熱モデル** (tire_thermal.hpp) ← 独立しているので先に実装
2. **パラメータ構造** (tire_params.hpp) ← 他の修正のベースとなる
3. **TMeasy改善** (tmeasy.hpp) ← 熱モデルと動的緩和長を統合
4. **統合インターフェース** (tire_model_interface.hpp) ← NNモデルとの切り替え
5. **テスト作成** (test_tire_thermal.cpp, test_tire_integration.cpp)

## 非実施項目（明示的に触らない）

- `matlab/dl_tire_model/` 配下のすべてのファイル（学習フロー）
- NN学習アルゴリズム
- データ前処理パイプライン
- エクスポート機能の改修

## 成功基準

1. ✅ タイヤ表面温度が動的に計算される
2. ✅ 緩和長が荷重に応じて変化する
3. ✅ 荷重依存性がタイヤフィッティング設計書の式に従う
4. ✅ TMeasyとNNモデルを切り替えられる
5. ✅ 既存のシミュレーションコードとの互換性を保つ
6. ✅ MATLABのNN学習フローを一切変更しない

## 次のステップ

各Phaseごとに実装ファイルを作成し、統合テストを実施する。
