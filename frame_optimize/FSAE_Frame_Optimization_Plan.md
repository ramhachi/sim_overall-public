# FSAE フレーム統合最適化プログラム — 計画書

> **Version**: 1.0  
> **Date**: 2026-02-23  
> **規則準拠**: Formula SAE Rules 2026 V1 (10 Sept 2025)  
> **実装言語**: Python (計算エンジン) + MATLAB (可視化・UI)  

---

## 第Ⅰ部：概念設計書

### 1. プロジェクトの目的と位置づけ

本プログラムは、FSAEスペースフレームの設計において**概念設計段階で高速にトポロジー・断面寸法を探索**し、車検適合性を自動検証しつつ、ねじり剛性と軽量化を両立する最適解を見つけることを目的とする。

市販CAEソフトウェアを最適化ループ内に含めない代わりに、**3D梁要素FEM（直接剛性法）を自作**し、1評価あたり0.5秒以内での高速計算を実現する。

#### 1.1 想定する使い方の流れ

```
[ユーザー操作]
  初期ジオメトリ入力（節点・候補パイプ）
  レギュレーション阻止領域・テンプレートの定義
  エンジン/バッテリー等の阻止領域の定義
  素材カタログ・市販パイプカタログの選択
         |
[最適化エンジン]
  Ground Structure法 + 断面最適化の同時実行
  レギュレーション適合チェック（各評価時）
  ねじり剛性 / 重量 / (拡張用)応力 等の計算
         |
[出力]
  MATLAB 3D可視化
  最適トポロジー＋断面寸法
  レギュレーション適合レポート
  パレートフロント図
```

---

### 2. FSAE 2026 レギュレーションの整理と制約への落とし込み

規則書 Section F（Chassis and Structural）から、最適化プログラムに必要な制約を以下のように体系化する。

#### 2.1 管材寸法規定 — F.3.4.1

規則は用途別に4段階の最小要件を定めている。これはハード制約としてプログラムに組み込む。

| サイズ区分 | 最小断面二次モーメント | 最小断面積 | 最小外径/辺長 | 最小肉厚 | 適用箇所 |
|---|---|---|---|---|---|
| **Size A** | 11,320 mm4 | 173 mm2 | 25.0 mm | 2.0 mm | フロントフープ(c), メインフープ(g), ショルダーハーネスバー(k) |
| **Size B** | 8,509 mm4 | 114 mm2 | 25.0 mm | 1.2 mm | フロントBH(a), FBH Support(b), サイドインパクト(e), MHブレース(h), ハーネス取付(j), バッテリー保護(m) |
| **Size C** | 6,695 mm4 | 91 mm2 | 25.0 mm | 1.2 mm | MH Brace Support(i), コンポーネント保護(n), 構造チューブ一般(o) |
| **Size D** | 18,015 mm4 | 126 mm2 | 35.0 mm | 1.2 mm | 曲げ/複数管のUpper Side Impact(f) |

##### 材料物性値（SES計算用）— F.3.4.2

| | ヤング率 E | 降伏応力 Sy | 引張強度 Su |
|---|---|---|---|
| **鉄鋼（非溶接）** | 200 GPa | 305 MPa | 365 MPa |
| **鉄鋼（溶接部）** | 200 GPa | 180 MPa | 300 MPa |
| **Al 6061-T6（非溶接）** | 69 GPa | 240 MPa | 290 MPa |
| **Al 6061-T6（溶接部）** | 69 GPa | 115 MPa | 175 MPa |

##### アルミニウム使用制限 — F.3.2.1 / F.3.5

メインフープ(g)、MHブレーシング(h)、ショルダーハーネスバー(k)の3箇所は**鉄鋼のみ**。それ以外はアルミ使用可（ただしSESでの等価証明が必要）。アルミ最小肉厚は非溶接2.0mm、溶接3.0mm。

#### 2.2 構造トポロジー要件

以下はフレームの位相（何がどこに繋がるか）に関する強制要件であり、最適化で削除できない**保護されたメンバー**となる。

##### 必須構造要素（削除不可）

| 構造要素 | 規則条項 | 制約内容 |
|---|---|---|
| **フロントバルクヘッド** | F.6.1 | Size B以上の閉断面管で構成 |
| **FBH Support** | F.6.2 | 各側3本以上（上・下・対角）でフロントフープまで三角化 |
| **フロントフープ** | F.5.7 | Size A以上、片側最下部から反対側最下部まで連続管 |
| **フロントフープブレース** | F.6.3 | Size B以上、左右各1本、前方に伸ばしFBHまたは三角化節点へ |
| **サイドインパクト（上・下・対角）** | F.6.4 | Size B以上、各側3本以上。Upper SIは地面から高さ290-345mm |
| **メインフープ** | F.5.8 | Size A以上、**鉄鋼のみ**、単一連続管、垂直から10度以内 |
| **MHブレース** | F.5.9 | Size B以上、**鉄鋼のみ**、左右各1本、頂部160mm以内に接続、30度以上 |
| **MHブレースサポート** | F.6.6 | Size C以上、各側2本（上下）で三角化 |
| **ショルダーハーネスバー** | F.6.5 | Size A以上、**鉄鋼のみ**、単一連続管、MH左右に接続 |

##### ジオメトリ制約

| 制約 | 規則条項 | 定量値 |
|---|---|---|
| MH内幅 | F.5.8.4 | 最下部接続点で380mm以上（内寸） |
| MHブレース角度 | F.5.9.5 | MHとブレースのなす角30度以上 |
| フロントフープ傾斜 | F.5.7.6 | Upper SI上方で鉛直から20度以内 |
| 曲げ最小半径 | F.5.2.1 | 管中心線で3 x OD以上 |
| ヘルメットクリアランス | F.5.6.4 | MH頂部-FH頂部直線から50mm以上 |

#### 2.3 コクピットテンプレート制約 — T.1.1, T.1.2

規則で2種類のテンプレートが定義されている。これらは**幾何学的制約**として最適化に組み込む。

- **コクピット開口テンプレート**: FHとMHの間で、SI上面から25mm下まで水平挿入可能
- **内部断面テンプレート**: ペダル後方100mmからステアリングコラム後方まで通過可能

#### 2.4 設計阻止領域の体系

プログラムで設定可能な阻止領域は以下の4カテゴリとする。

| カテゴリ | 例 | 表現方法 |
|---|---|---|
| **レギュレーション領域** | コクピットテンプレート通過空間、ヘルメットクリアランス | 凸多面体/円柱の和集合 |
| **搭載物領域** | エンジン、バッテリーパック、燃料タンク | AABB（軸平行直方体）またはOBB |
| **機構領域** | サスペンションストローク範囲、ステアリングラック可動域 | パラメトリック掃引体積 |
| **ユーザー定義領域** | エルゴノミクス、配管経路、メンテナンスアクセス | 任意凸多面体 |

パイプが阻止領域を貫通した場合、**侵入体積に比例するペナルティ**を目的関数に加算する。

---

### 3. 最適化の数理的枠組み

#### 3.1 設計変数

本プログラムは2段階の設計変数を同時に扱う。

**トポロジー変数（離散）**: Ground Structure法に基づき、あらかじめ候補パイプを大量に定義しておき、各パイプの存在/非存在を0/1の密度変数 rho で表現する。連続緩和 rho in [0, 1] を用い、最終段階でSIMP法的に rho を 0 or 1 へ射影する。

**断面変数（離散）**: 各パイプグループの外径D・肉厚tは連続変数ではなく、**市販パイプカタログからの離散選択**とする。これはカタログインデックスの整数変数として扱い、遺伝的アルゴリズム（GA）で探索する。

#### 3.2 目的関数

単目的化した関数 f(x) を最小化する。

```
f(x) = W_mass * f_mass(x)
     - W_stiff * f_stiff(x)
     + SUM( P_constraint_i(x) )
```

##### 質量項 f_mass

```
f_mass = m(x) / m_ref
```

ここで m_ref はベースライン設計の質量。**質量は最も重い重みを持つ**（W_mass > W_stiff）ことで軽量化を優先する。

##### 剛性項 f_stiff — 閾値型報酬関数

剛性については「ある値以上でないとペナルティ、それ以上で報酬」を以下のように実装する。

```
K < K_min の場合:
  f_stiff = +gamma * ((K_min - K) / K_min)^2         ... 大きなペナルティ

K_min <= K < K_target の場合:
  f_stiff = -alpha * (K - K_min)/(K_target - K_min)   ... 小さい報酬

K_target <= K の場合:
  f_stiff = -alpha - beta * (K - K_target)/(K_max - K_target)  ... 報酬（逓減）
```

ここで K_min は最低許容剛性、K_target は目標剛性、K_max は飽和上限。gamma >> alpha により、K_min 未満で急激なペナルティを与える。

**設計方針**: W_mass > W_stiff * alpha とすることで、剛性が十分なら質量削減が優先される。

##### 制約ペナルティ P

| ペナルティ | 条件 | 強度 |
|---|---|---|
| 阻止領域侵入 | パイプが阻止領域と干渉 | 侵入長に比例、係数大 |
| 最小断面未達 | 規則の Size A/B/C/D 未達 | 不足量の二乗、係数極大 |
| 三角化不備 | 必要な三角化ができていない | 固定ペナルティ |
| テンプレート不通過 | コクピットテンプレートが通らない | 侵入量に比例 |
| ヘルメットクリアランス不足 | 50mm未満 | 不足量に比例 |
| 座屈不成立 | オイラー座屈荷重が設計荷重未満 | 比率に比例 |

#### 3.3 解析エンジンの出力と拡張性

解析結果は**構造化コンテナ**として返し、後から目的関数を追加できるようにする。

```python
class AnalysisResult:
    """解析結果コンテナ — 全ての物理量を保持"""
    # --- 基本 ---
    mass: float                    # 総質量 [kg]
    torsional_rigidity: float      # ねじり剛性 [Nm/deg]
    
    # --- 変位・応力場（全自由度） ---
    displacement: np.ndarray       # 全節点変位ベクトル
    element_forces: dict           # 要素内力（軸力、曲げ、せん断、ねじり）
    element_stresses: dict         # 要素応力（von Mises等）
    
    # --- 座屈 ---
    euler_buckling_factors: dict   # 各要素のオイラー座屈安全率
    
    # --- 固有振動 ---
    natural_frequencies: np.ndarray  # 固有振動数 [Hz]（オプション）
    mode_shapes: np.ndarray          # 固有モード形状（オプション）
    
    # --- レギュレーション ---
    regulation_check: dict         # 各規則条項の合否と余裕
    
    # --- 拡張用スロット ---
    custom: dict                   # ユーザー定義の追加出力
```

この設計により、将来的に以下のような目的関数の追加が容易になる:

- 横曲げ剛性
- 1次固有振動数の最大化
- サスペンション取付点の応力最小化
- 荷重ケース別の最大変位制限
- エネルギー吸収量（衝突解析）
- コスト関数（素材費 + 加工費の推定）

---

### 4. 市販パイプカタログと素材データベース

#### 4.1 パイプカタログの構造

パイプの径・肉厚は納入先の在庫に依存するため、**離散カタログ**として管理する。

```python
@dataclass
class PipeCatalogEntry:
    id: str                  # 例: "STL-R-25.4x1.65"
    shape: str               # 'round' or 'square'
    material_id: str         # 素材DBへの参照
    D_outer: float           # 外径 [mm]
    t: float                 # 肉厚 [mm]
    available: bool          # 現在在庫あり
    cost_per_meter: float    # 単価 [円/m]（オプション）
    lead_time_days: int      # 納期 [日]（オプション）
    
    def size_class(self) -> str:
        """F.3.4.1 の Size A/B/C/D のどれを満たすか自動判定"""
        ...
```

#### 4.2 素材データベース

素材は最低4種（AISI 4130, AISI 1020, Al 6061-T6, Al 7075-T6）を用意し、ユーザーが追加可能とする。各素材には規則上の制限がメタデータとして含まれる。

特に重要な規則上の制約:
- メインフープ(g)、MHブレース(h)、ショルダーハーネスバー(k)は**鉄鋼のみ**（F.3.2.1）
- アルミの非溶接最小肉厚: 2.0mm、溶接最小肉厚: 3.0mm（F.3.5.3）
- SES計算ではF.3.4.2の規定値を使用（実材料の値ではなく規則上の値）

---

### 5. 梁FEMの精度と補正

#### 5.1 誤差要因と対策

| 誤差要因 | 梁FEM vs シェルFEM誤差 | 対策 | 補正後誤差 |
|---|---|---|---|
| 継手の局所変形 | +15-30% 過大評価 | CIDECT経験式による補正係数 | 5-15% |
| コープカット断面欠損 | +10-25% 過大評価 | 有効長補正 + 継手係数の再調整 | 5-12% |
| せん断変形の無視 | +5-10% | Timoshenko梁要素 | < 3% |
| 断面のovalization | +3-8% | 補正不可（保守的方向） | 3-8% |
| **総合** | **+20-40%** | **全補正適用** | **10-20%** |

#### 5.2 キャリブレーション戦略

1. シェルFEM（市販CAE）で3-5ケースの基準解を作成
2. 同じモデルを梁FEMで解き、継手補正係数をフィッティング
3. フィッティング結果を設定ファイルに保存
4. 最適化で得られた上位5案を改めてシェルFEMで検証

---

## 第II部：実装設計書

### 6. システムアーキテクチャ

```
+---------------------------------------------------+
|                  MATLAB (Front-End)                |
|  +------------+  +------------+  +------------+   |
|  | 3D Viewer  |  | Pareto Plot|  | Config GUI |   |
|  | (frame +   |  | (mass vs   |  | (禁止領域   |   |
|  |  exclusion |  |  stiffness)|  |  パイプDB)  |   |
|  |  zones)    |  |            |  |            |   |
|  +-----+------+  +-----+------+  +-----+------+  |
|        |               |               |          |
|  +-----+---------------+---------------+------+   |
|  |           MATLAB-Python Bridge              |   |
|  |     (MAT-file I/O or TCP socket)           |   |
|  +---------------------+----------------------+   |
+-------------------------|--------------------------+
                          |
+-------------------------|--------------------------+
|                         v  Python (Back-End)       |
|  +-----------------------------------------------+ |
|  |         OptimizationEngine                    | |
|  |  +----------+  +-----------+  +--------+     | |
|  |  |  GA/DE   |->|  Beam FEM |->|Regulat.|     | |
|  |  | Optimizer|  |  Solver   |  |Checker |     | |
|  |  +----------+  +-----------+  +--------+     | |
|  +-----------------------------------------------+ |
|  +-------------+ +--------------+ +----------+    |
|  | PipeCatalog | |MaterialDB    | |Exclusion |    |
|  |   (離散)    | |(鉄/Al/CrMo)  | | ZoneDB  |    |
|  +-------------+ +--------------+ +----------+    |
+----------------------------------------------------+
```

### 7. Python モジュール構成

```
fsae_optimizer/
  __init__.py
  config.py                 # グローバル設定・定数

  data/
    materials.json          # 素材データベース
    pipe_catalog.json       # 市販パイプカタログ（離散）
    regulation_params.json  # 規則数値（Size A/B/C/D等）

  model/
    __init__.py
    node.py                 # 節点クラス
    element.py              # 要素クラス（管材属性含む）
    frame.py                # フレームモデル（節点+要素の集合体）
    pipe_catalog.py         # パイプカタログマネージャ
    material.py             # 素材クラス
    exclusion_zone.py       # 阻止領域（AABB, OBB, 凸多面体, 円柱）

  fem/
    __init__.py
    beam3d.py               # 3D梁要素（Timoshenko）
    assembly.py             # 全体剛性行列の組立
    solver.py               # 連立方程式ソルバー
    postprocess.py          # 応力・内力の後処理
    buckling.py             # オイラー座屈チェック
    modal.py                # 固有値解析（オプション）
    joint_correction.py     # 継手柔性・コープカット補正

  regulation/
    __init__.py
    tube_size_check.py      # F.3.4.1 管材サイズチェック
    topology_check.py       # 必須構造の存在チェック
    geometry_check.py       # 角度・距離・クリアランスチェック
    template_check.py       # T.1.1/T.1.2 コクピットテンプレート
    ses_export.py           # SES用データ出力

  optimization/
    __init__.py
    design_variable.py      # 設計変数の定義（トポロジー+断面）
    objective.py            # 目的関数（質量・剛性・ペナルティ統合）
    ground_structure.py     # Ground Structure法：候補パイプ生成
    ga_optimizer.py         # 遺伝的アルゴリズム（離散断面対応）
    de_optimizer.py         # Differential Evolution（連続緩和用）
    result.py               # AnalysisResult コンテナ

  io/
    __init__.py
    csv_import.py           # CADからの節点座標CSV読込
    mat_export.py           # MATLAB用.matファイル出力
    json_io.py              # 設定・結果のJSON入出力
    paraview_export.py      # VTK出力（オプション）

  matlab/                   # MATLAB側スクリプト群
    fsae_viewer.m           # メイン3D可視化
    plot_frame.m            # フレーム描画（パイプ径反映）
    plot_exclusion.m        # 阻止領域の半透明描画
    plot_pareto.m           # パレートフロント
    plot_convergence.m      # 収束履歴
    plot_regulation.m       # 規則適合状況のヒートマップ
    load_results.m          # Python結果の読込
    template_check_viz.m    # テンプレート通過可否の可視化
    animate_optimization.m  # 最適化過程のアニメーション
```

---

### 8. 主要クラス・関数の詳細設計

#### 8.1 Ground Structure（候補パイプ生成）

```python
class GroundStructure:
    """
    与えられた節点群に対し、接続候補パイプを自動生成する。
    
    戦略:
    1. 全節点ペアから候補を列挙
    2. 最大長制約で足切り
    3. 阻止領域との干渉が大きい候補を除外
    4. 必須構造メンバーは削除不可フラグ付与
    """
    
    def generate(self, nodes, max_length, exclusion_zones,
                 required_members):
        ...
    
    def add_node(self, position):
        """最適化中にノードを追加（adaptive refinement）"""
        ...
```

#### 8.2 離散パイプ選択

```python
class DiscretePipeSelector:
    """GAの遺伝子としてカタログインデックスを直接使用する"""
    
    def __init__(self, catalog):
        self.catalog = catalog
    
    def nearest_available(self, D_target, t_target, 
                          material_id, size_class_min):
        """目標値に最も近い市販パイプを返す"""
        ...
    
    def encode_for_ga(self, element_groups):
        """GA用: 各グループのカタログインデックスを遺伝子に変換"""
        ...
    
    def decode_from_ga(self, gene):
        """GA用: 遺伝子からパイプ割当を復元"""
        ...
```

#### 8.3 阻止領域

```python
class ExclusionZone:
    """阻止領域の基底クラス"""
    def contains_point(self, point) -> bool: ...
    def intersects_segment(self, p1, p2) -> float: ...
    def penalty(self, p1, p2, pipe_radius) -> float: ...

class AABBExclusionZone(ExclusionZone):
    """軸平行直方体（エンジン、バッテリー等）"""
    ...

class CylinderExclusionZone(ExclusionZone):
    """円柱（テンプレート通過空間、排気管クリアランス等）"""
    ...

class ConvexHullExclusionZone(ExclusionZone):
    """凸多面体（任意形状）"""
    ...
```

#### 8.4 目的関数（閾値型剛性報酬）

```python
class ObjectiveFunction:
    """
    f(x) = W_mass * f_mass(x) - W_stiff * f_stiff(x) + penalties
    
    設計方針:
    - W_mass > W_stiff により質量を最優先
    - 剛性は閾値型: K_min未満で巨大ペナルティ
    - 解析結果を拡張可能な形で保持
    """
    
    def __init__(self, config):
        self.W_mass = config.W_mass          # 例: 1.0
        self.W_stiff = config.W_stiff        # 例: 0.5 (< W_mass)
        self.K_min = config.K_min            # 最低許容剛性 [Nm/deg]
        self.K_target = config.K_target      # 目標剛性
        self.gamma_stiff = config.gamma      # ペナルティ係数（大）
        self.extra_objectives = []           # 拡張用
    
    def stiffness_reward(self, K):
        """閾値型剛性報酬/ペナルティ"""
        if K < self.K_min:
            deficit = (self.K_min - K) / self.K_min
            return self.gamma_stiff * deficit ** 2  # 巨大ペナルティ
        elif K < self.K_target:
            progress = (K - self.K_min) / (self.K_target - self.K_min)
            return -self.alpha * progress * 0.5     # 小さい報酬
        else:
            return -self.alpha                       # フル報酬
    
    def add_objective(self, name, func, weight):
        """目的関数の動的追加"""
        self.extra_objectives.append((name, func, weight))
    
    def evaluate(self, result):
        """統合目的関数値を計算"""
        f = self.W_mass * (result.mass / self.mass_ref)
        f += self.stiffness_reward(result.torsional_rigidity)
        for name, func, w in self.extra_objectives:
            f += w * func(result)
        f += sum(result.regulation_check.get('penalties', []))
        return f
```

---

### 9. MATLAB可視化の設計

#### 9.1 可視化出力一覧

| 可視化 | ファイル | 内容 |
|---|---|---|
| **3Dフレーム** | plot_frame.m | パイプを実径の円柱で描画。サイズ区分別に色分け |
| **阻止領域** | plot_exclusion.m | 半透明の箱/円柱/凸包で描画。侵入パイプを赤色ハイライト |
| **パレートフロント** | plot_pareto.m | 質量 vs ねじり剛性の散布図。現在ベスト解をマーク |
| **収束履歴** | plot_convergence.m | 世代数 vs 目的関数値、質量、剛性の3段プロット |
| **規則適合マップ** | plot_regulation.m | 各パイプのサイズ区分適合を緑/黄/赤で表示 |
| **テンプレート** | template_check_viz.m | コクピット断面にテンプレートを重ねて描画 |
| **アニメーション** | animate_optimization.m | 最適化過程をフレーム毎にアニメーション表示 |
| **変形図** | plot_deformation.m | ねじり荷重時の変形モード（スケール付） |
| **応力分布** | plot_stress.m | 各要素のvon Mises応力をカラーマップ表示 |

#### 9.2 MATLAB-Python間のデータ連携

**推奨方式: MAT-fileによるバッチ連携**

PythonのsciPy.io.savematでMAT形式に保存し、MATLAB側でload()で読み込む。リアルタイム可視化が必要な場合はTCPソケットで中間結果をストリーミングする。

---

### 10. 最適化以外で実装可能な付加機能

レギュレーションを詳細に読み込んだ結果、最適化エンジン以外にも以下の機能が有用と判断した。

#### 10.1 SES自動生成ツール

規則 F.2.1 で要求されるStructural Equivalency Spreadsheetのデータを自動生成する。計算済みの断面特性、座屈荷重、等価性計算結果をExcelテンプレートに自動入力する。

#### 10.2 溶接長・溶接コスト見積もり

フレーム全パイプの端部交差曲線（フィッシュマウス）長さを幾何学的に計算し、溶接長の総計と概算コストを出力する。コスト・製造性イベント（S.3、最大124点）のスコア向上に直結する。

#### 10.3 ドライバーテンプレート検証（F.5.6.5）

95パーセンタイル男性テンプレート（phi200mm-490mm-phi200mm-280mm-phi300mm）をフレーム内に配置し、ヘルメットクリアランス50mmの自動検証をMATLABで3D可視化する。

#### 10.4 衝突エネルギー吸収の簡易評価

インパクトアテネータ（F.8）の要件（平均減速20g以下、ピーク40g以下、吸収エネルギー7350J以上）に対し、フロント構造の剛性と変形からエネルギー吸収量を概算する。

#### 10.5 チルトテスト予測

車検のチルトテスト（IN.12、60度傾斜）でのCG位置と転倒リスクを、フレーム重量分布から簡易推定する。

#### 10.6 荷重ケースライブラリ

| 荷重ケース | 内容 | 用途 |
|---|---|---|
| ねじり | リア固定、フロントトルク | メイン目的関数 |
| 横曲げ | 片側コーナリング2G | サイドインパクト評価 |
| 縦曲げ | 前後バンプ3G | FBHサポート評価 |
| 前面衝突 | 20G減速 | IA/AIP評価 |
| ロールオーバー | MH頂部に側方荷重 | MH/MHブレース評価 |
| ブレーキ | 1.5G減速 | FBH-MH間の評価 |

#### 10.7 デザインイベント支援データ出力

デザインイベント（S.4、最大150点）に向け、最適化の根拠データ（パレートフロント、設計トレードオフ、感度解析結果）を自動的にレポート形式で出力する機能。審査員への説明資料の基礎データとなる。

---

### 11. 実装ロードマップ

#### Phase 1: 基盤構築（2週間）

| タスク | 詳細 | 優先度 |
|---|---|---|
| 素材・パイプDB | JSON定義、カタログマネージャ | 高 |
| 3D梁FEMソルバー | Timoshenko梁要素、直接剛性法、スパース解法 | 高 |
| 継手補正 | CIDECT式 + コープカット有効長補正 | 高 |
| ねじり剛性計算 | リア固定・フロントトルク法 | 高 |
| 規則チェッカー | Size A/B/C/D判定、必須構造チェック | 高 |

#### Phase 2: 最適化エンジン（2週間）

| タスク | 詳細 | 優先度 |
|---|---|---|
| Ground Structure生成 | 候補パイプ列挙、阻止領域フィルタ | 高 |
| 離散GA | カタログインデックス遺伝子、エリート保存 | 高 |
| 目的関数 | 質量 + 閾値型剛性報酬 + ペナルティ | 高 |
| 阻止領域 | AABB, 円柱, 凸包、干渉判定 | 高 |
| テンプレートチェック | 2D断面のMinkowski和による通過判定 | 中 |

#### Phase 3: MATLAB可視化（1週間）

| タスク | 詳細 | 優先度 |
|---|---|---|
| 3Dフレーム描画 | surf()による円柱パイプ、色分け | 高 |
| 阻止領域描画 | patch()半透明、alphamap | 高 |
| パレートフロント | scatter()、interactive datatip | 中 |
| 収束履歴 | subplot、リアルタイム更新対応 | 中 |
| アニメーション | VideoWriter、GIF出力 | 低 |

#### Phase 4: 検証と拡張（1週間）

| タスク | 詳細 | 優先度 |
|---|---|---|
| シェルFEM比較 | 3-5ケースのキャリブレーション | 高 |
| SES出力 | 規則準拠ドキュメント自動生成 | 中 |
| 固有値解析 | 質量行列 + 固有値ソルバー | 中 |
| コスト関数 | 素材費 + 溶接長 x 単価 | 低 |
| 疲労寿命推定 | 応力範囲からSN曲線参照 | 低 |

---

## 付録

### A. FSAE 2026 管材要件クイックリファレンス

```
F.3.2.1 管材用途別要件一覧
--------------------------------------------------------------
用途                | サイズ | Al可? | 主な制約
--------------------------------------------------------------
a. Front Bulkhead   | B      | Yes   | F.6.1
b. FBH Support      | C      | Yes   | F.6.2, 各側3本+三角化
c. Front Hoop       | A      | Yes   | F.5.7, 連続管
d. FH Bracing       | B      | Yes   | F.6.3, 直線
e. Side Impact      | B      | Yes   | F.6.4, 各側3本以上
f. Bent Upper SI    | D      | Yes   | F.6.4.4
g. Main Hoop        | A      | NO    | F.5.8, 鉄鋼のみ, 単一連続管
h. MH Bracing       | B      | NO    | F.5.9, 鉄鋼のみ, 直線
i. MH Brace Supp.   | C      | Yes   | F.6.6, 各側2本(上下)
j. Harness Attach   | B      | Yes   | T.2.4
k. Shoulder Bar     | A      | NO    | F.6.5, 鉄鋼のみ, 単一連続管
l. SH Bar Brace     | C      | Yes   | F.6.5.2
m. Battery Protect  | B      | Yes   | F.11.2
n. Component Prot.  | C      | Yes   | F.5.16
o. Structural       | C      | Yes   | 一般構造チューブ
--------------------------------------------------------------
```

### B. 代表的な市販パイプカタログ（初期データ例）

```
鉄鋼 丸パイプ（AISI 4130 CrMo / AISI 1020）:
  phi 19.05 x 0.89mm (3/4" x 0.035")    <- 非構造
  phi 22.22 x 0.89mm (7/8" x 0.035")    <- 非構造
  phi 25.40 x 1.25mm (1" x 0.049")      <- Size C
  phi 25.40 x 1.65mm (1" x 0.065")      <- Size B
  phi 25.40 x 2.10mm (1" x 0.083")      <- Size A適合可能
  phi 25.40 x 2.41mm (1" x 0.095")      <- Size A
  phi 28.58 x 1.65mm (1-1/8" x 0.065")
  phi 31.75 x 1.25mm (1-1/4" x 0.049")  <- Size D適合可能
  phi 31.75 x 1.65mm (1-1/4" x 0.065")  <- Size D
  phi 34.93 x 1.65mm (1-3/8" x 0.065")  <- Size D
  phi 38.10 x 1.65mm (1-1/2" x 0.065")

鉄鋼 角パイプ:
  25.40 x 25.40 x 1.65mm (1" x 1" x 0.065")
  25.40 x 25.40 x 2.41mm (1" x 1" x 0.095")
  31.75 x 31.75 x 1.65mm (1-1/4" x 1-1/4" x 0.065")

Al 6061-T6 丸パイプ:
  phi 25.40 x 2.00mm
  phi 31.75 x 2.00mm
  phi 38.10 x 2.00mm
  phi 25.40 x 3.00mm  <- 溶接部用
  phi 31.75 x 3.00mm  <- 溶接部用
```

### C. 計算時間バジェット

| 処理 | 推定時間 | 備考 |
|---|---|---|
| 断面特性計算 | < 0.1 ms | 全要素分 |
| 全体剛性行列組立 | 1-3 ms | 100要素、スパースCOO |
| 連立方程式求解 | 2-8 ms | 480 DOF、scipy spsolve |
| 後処理（応力等） | < 1 ms | |
| 規則チェック | < 1 ms | 幾何判定のみ |
| 阻止領域干渉判定 | < 1 ms | AABB高速判定 |
| **1評価合計** | **< 15 ms** | **目標500msに対して余裕33倍** |
| GA 1世代（pop=50） | 約750 ms | 50評価 |
| 最適化全体（100世代） | 約75 秒 | 5000評価 |

### D. 大会配点と本プログラムの寄与

| イベント | 配点 | 本プログラムの寄与 |
|---|---|---|
| Design Event | 150点 | 最適化根拠データ、トレードオフ解析の提示 |
| Cost and Manufacturing | 124点 | 溶接長推定、部品点数の最適化 |
| Acceleration | 100点 | 軽量化による加速性能向上 |
| Skid Pad | 75点 | 重心高低減、ねじり剛性によるハンドリング |
| Autocross | 125点 | 同上 |
| Endurance | 275点 | 軽量化+剛性バランスによる総合性能 |
| Efficiency | 100点 | 軽量化による消費エネルギー削減 |
| **合計** | **949点** | フレーム設計は全イベントに間接的に影響 |
