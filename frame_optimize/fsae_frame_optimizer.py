"""
FSAE Space Frame Optimization Loop
===================================
3D梁要素FEM（直接剛性法）によるねじり剛性・重量の多目的最適化

特徴:
- Euler-Bernoulli / Timoshenko 3D梁要素 (12 DOF/要素)
- 溶接継手柔性の経験的補正
- scipy.optimize による多目的最適化
- 1評価あたり < 0.05秒（100要素規模）

精度目標: 市販CAE（シェル要素）比 ±20%以内
"""

import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
from scipy.optimize import minimize, differential_evolution
from dataclasses import dataclass, field
from typing import List, Tuple, Optional, Dict
import time


# =============================================================================
# 1. データ構造
# =============================================================================

@dataclass
class TubeMaterial:
    """管材の材料特性"""
    name: str
    E: float          # ヤング率 [Pa]
    G: float          # せん断弾性係数 [Pa]
    rho: float        # 密度 [kg/m^3]
    yield_stress: float  # 降伏応力 [Pa]


@dataclass
class TubeSection:
    """管材の断面特性
    
    shape: 'round' or 'square'
    D_outer: 外径 [m] (丸パイプ) or 外辺長 [m] (角パイプ)
    t: 肉厚 [m]
    """
    shape: str          # 'round' or 'square'
    D_outer: float      # 外径 or 外辺長 [m]
    t: float            # 肉厚 [m]
    
    @property
    def D_inner(self) -> float:
        return self.D_outer - 2 * self.t
    
    @property
    def A(self) -> float:
        """断面積 [m^2]"""
        if self.shape == 'round':
            return np.pi / 4 * (self.D_outer**2 - self.D_inner**2)
        else:  # square
            return self.D_outer**2 - self.D_inner**2
    
    @property
    def Iy(self) -> float:
        """断面二次モーメント (y軸) [m^4]"""
        if self.shape == 'round':
            return np.pi / 64 * (self.D_outer**4 - self.D_inner**4)
        else:
            return (self.D_outer**4 - self.D_inner**4) / 12
    
    @property
    def Iz(self) -> float:
        """断面二次モーメント (z軸) [m^4]"""
        return self.Iy  # 対称断面
    
    @property
    def J(self) -> float:
        """ねじり定数 (Saint-Venant) [m^4]"""
        if self.shape == 'round':
            return np.pi / 32 * (self.D_outer**4 - self.D_inner**4)
        else:
            # 薄肉閉断面の近似 (Bredt's formula)
            b = self.D_outer - self.t  # 中心線辺長
            Am = b**2                   # 中心線囲い面積
            s = 4 * b                   # 中心線周長
            return 4 * Am**2 * self.t / s
    
    @property
    def Asy(self) -> float:
        """せん断有効面積 (y方向) [m^2] - Timoshenko梁用"""
        if self.shape == 'round':
            # 薄肉円筒の近似
            return self.A / 2.0
        else:
            return self.A * 5 / 6
    
    @property
    def Asz(self) -> float:
        """せん断有効面積 (z方向) [m^2]"""
        return self.Asy  # 対称断面


@dataclass
class Node:
    """節点"""
    id: int
    x: float
    y: float
    z: float
    
    @property
    def coords(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])


@dataclass
class Element:
    """3D梁要素"""
    id: int
    node_i: int       # 始点節点ID
    node_j: int       # 終点節点ID
    section: TubeSection
    material: TubeMaterial
    group: str = ""    # 最適化用グループ名 (同グループ=同断面)
    joint_factor: float = 1.0  # 継手柔性補正係数 (0.0-1.0, 1.0=剛結)


@dataclass 
class BoundaryCondition:
    """境界条件"""
    node_id: int
    dof: int          # 0-5: ux, uy, uz, rx, ry, rz
    value: float = 0.0


@dataclass
class Load:
    """外力"""
    node_id: int
    dof: int          # 0-5: Fx, Fy, Fz, Mx, My, Mz
    value: float


# =============================================================================
# 2. 3D梁要素剛性行列
# =============================================================================

def beam3d_local_stiffness(L: float, E: float, G: float, 
                            A: float, Iy: float, Iz: float, J: float,
                            Asy: float = None, Asz: float = None,
                            use_timoshenko: bool = True) -> np.ndarray:
    """
    3D梁要素のローカル剛性行列 (12x12)
    
    DOF順: [ux, uy, uz, rx, ry, rz] for node_i, then node_j
    
    use_timoshenko=True: Timoshenko梁（せん断変形考慮）
    use_timoshenko=False: Euler-Bernoulli梁
    """
    K = np.zeros((12, 12))
    
    # せん断変形パラメータ
    if use_timoshenko and Asy is not None and Asz is not None:
        phi_y = 12 * E * Iz / (G * Asy * L**2) if Asy > 0 else 0
        phi_z = 12 * E * Iy / (G * Asz * L**2) if Asz > 0 else 0
    else:
        phi_y = 0
        phi_z = 0
    
    # 軸力項
    ea_l = E * A / L
    K[0, 0] = ea_l;   K[0, 6] = -ea_l
    K[6, 0] = -ea_l;  K[6, 6] = ea_l
    
    # ねじり項
    gj_l = G * J / L
    K[3, 3] = gj_l;   K[3, 9] = -gj_l
    K[9, 3] = -gj_l;  K[9, 9] = gj_l
    
    # y方向曲げ (Iz使用)
    c1 = 12 * E * Iz / (L**3 * (1 + phi_y))
    c2 = 6 * E * Iz / (L**2 * (1 + phi_y))
    c3 = (4 + phi_y) * E * Iz / (L * (1 + phi_y))
    c4 = (2 - phi_y) * E * Iz / (L * (1 + phi_y))
    
    K[1, 1] = c1;    K[1, 5] = c2;    K[1, 7] = -c1;   K[1, 11] = c2
    K[5, 1] = c2;    K[5, 5] = c3;    K[5, 7] = -c2;   K[5, 11] = c4
    K[7, 1] = -c1;   K[7, 5] = -c2;   K[7, 7] = c1;    K[7, 11] = -c2
    K[11, 1] = c2;   K[11, 5] = c4;   K[11, 7] = -c2;  K[11, 11] = c3
    
    # z方向曲げ (Iy使用)
    d1 = 12 * E * Iy / (L**3 * (1 + phi_z))
    d2 = 6 * E * Iy / (L**2 * (1 + phi_z))
    d3 = (4 + phi_z) * E * Iy / (L * (1 + phi_z))
    d4 = (2 - phi_z) * E * Iy / (L * (1 + phi_z))
    
    K[2, 2] = d1;    K[2, 4] = -d2;   K[2, 8] = -d1;   K[2, 10] = -d2
    K[4, 2] = -d2;   K[4, 4] = d3;    K[4, 8] = d2;    K[4, 10] = d4
    K[8, 2] = -d1;   K[8, 4] = d2;    K[8, 8] = d1;    K[8, 10] = d2
    K[10, 2] = -d2;  K[10, 4] = d4;   K[10, 8] = d2;   K[10, 10] = d3
    
    return K


def rotation_matrix_3d(node_i: np.ndarray, node_j: np.ndarray, 
                        v_ref: np.ndarray = None) -> np.ndarray:
    """
    要素のローカル→グローバル座標変換行列 (12x12)
    
    x軸: 要素軸方向 (node_i → node_j)
    v_ref: 参照ベクトル（y軸方向の定義用、デフォルトはグローバルY or Z）
    """
    dx = node_j - node_i
    L = np.linalg.norm(dx)
    ex = dx / L  # ローカルx軸
    
    # 参照ベクトルの選定
    if v_ref is None:
        # 要素がほぼ鉛直の場合はグローバルXを参照
        if abs(ex[1]) > 0.95:
            v_ref = np.array([1.0, 0.0, 0.0])
        else:
            v_ref = np.array([0.0, 1.0, 0.0])
    
    ez = np.cross(ex, v_ref)
    ez = ez / np.linalg.norm(ez)
    ey = np.cross(ez, ex)
    
    # 3x3回転行列
    R3 = np.array([ex, ey, ez])
    
    # 12x12ブロック対角
    T = np.zeros((12, 12))
    for i in range(4):
        T[3*i:3*i+3, 3*i:3*i+3] = R3
    
    return T


# =============================================================================
# 3. FEMソルバー
# =============================================================================

class BeamFEMSolver:
    """3D梁要素FEMソルバー（直接剛性法）"""
    
    def __init__(self, use_timoshenko: bool = True):
        self.nodes: Dict[int, Node] = {}
        self.elements: List[Element] = []
        self.bcs: List[BoundaryCondition] = []
        self.loads: List[Load] = []
        self.use_timoshenko = use_timoshenko
        
        self._K_global = None
        self._ndof = 0
    
    def add_node(self, node: Node):
        self.nodes[node.id] = node
    
    def add_element(self, elem: Element):
        self.elements.append(elem)
    
    def add_bc(self, bc: BoundaryCondition):
        self.bcs.append(bc)
    
    def add_load(self, load: Load):
        self.loads.append(load)
    
    def clear_loads(self):
        self.loads = []
    
    def _node_dof_start(self, node_id: int) -> int:
        """節点IDからグローバルDOFの開始インデックスを返す"""
        sorted_ids = sorted(self.nodes.keys())
        idx = sorted_ids.index(node_id)
        return idx * 6
    
    def assemble(self) -> Tuple[sparse.csr_matrix, np.ndarray]:
        """全体剛性行列と荷重ベクトルの組立"""
        n_nodes = len(self.nodes)
        self._ndof = n_nodes * 6
        
        # COO形式で組立（高速）
        rows = []
        cols = []
        vals = []
        
        for elem in self.elements:
            ni = self.nodes[elem.node_i]
            nj = self.nodes[elem.node_j]
            
            L = np.linalg.norm(nj.coords - ni.coords)
            sec = elem.section
            mat = elem.material
            
            # ローカル剛性行列
            K_local = beam3d_local_stiffness(
                L, mat.E, mat.G,
                sec.A, sec.Iy, sec.Iz, sec.J,
                sec.Asy, sec.Asz,
                self.use_timoshenko
            )
            
            # 継手柔性補正: 曲げ・ねじり項のみ低減
            if elem.joint_factor < 1.0:
                jf = elem.joint_factor
                # 軸力項(0,6)はそのまま、曲げ・ねじり項を低減
                mask = np.ones((12, 12))
                axial_dofs = [0, 6]
                for ii in range(12):
                    for jj in range(12):
                        if ii in axial_dofs and jj in axial_dofs:
                            continue
                        mask[ii, jj] = jf
                K_local = K_local * mask
            
            # 座標変換
            T = rotation_matrix_3d(ni.coords, nj.coords)
            K_global_elem = T.T @ K_local @ T
            
            # グローバル組立
            dof_i = self._node_dof_start(elem.node_i)
            dof_j = self._node_dof_start(elem.node_j)
            dof_map = list(range(dof_i, dof_i+6)) + list(range(dof_j, dof_j+6))
            
            for ii in range(12):
                for jj in range(12):
                    if abs(K_global_elem[ii, jj]) > 1e-20:
                        rows.append(dof_map[ii])
                        cols.append(dof_map[jj])
                        vals.append(K_global_elem[ii, jj])
        
        K = sparse.csr_matrix(
            (vals, (rows, cols)), 
            shape=(self._ndof, self._ndof)
        )
        
        # 荷重ベクトル
        F = np.zeros(self._ndof)
        for load in self.loads:
            dof_start = self._node_dof_start(load.node_id)
            F[dof_start + load.dof] += load.value
        
        return K, F
    
    def solve(self) -> np.ndarray:
        """連立方程式を解く → 変位ベクトル"""
        K, F = self.assemble()
        
        # 境界条件の適用（ペナルティ法 — 高速）
        penalty = 1e20 * K.diagonal().max()
        K_mod = K.tolil()
        for bc in self.bcs:
            dof = self._node_dof_start(bc.node_id) + bc.dof
            K_mod[dof, dof] += penalty
            F[dof] += penalty * bc.value
        
        K_mod = K_mod.tocsr()
        
        # 解法
        U = spsolve(K_mod, F)
        return U
    
    def total_mass(self) -> float:
        """フレーム総重量 [kg]"""
        mass = 0.0
        for elem in self.elements:
            ni = self.nodes[elem.node_i]
            nj = self.nodes[elem.node_j]
            L = np.linalg.norm(nj.coords - ni.coords)
            mass += elem.material.rho * elem.section.A * L
        return mass
    
    def torsional_rigidity(self, 
                           front_nodes: List[int],
                           rear_nodes: List[int],
                           torque: float = 1000.0) -> float:
        """
        ねじり剛性 [Nm/deg] の計算
        
        rear_nodes: 固定端（全DOF拘束）
        front_nodes: トルク入力端（カップルモーメント）
        """
        # 境界条件をリセット
        original_bcs = self.bcs.copy()
        original_loads = self.loads.copy()
        self.bcs = []
        self.loads = []
        
        # 後方固定
        for nid in rear_nodes:
            for dof in range(6):
                self.add_bc(BoundaryCondition(nid, dof, 0.0))
        
        # 前方にトルク入力
        # フロントノード群の重心を計算
        front_coords = np.array([self.nodes[nid].coords for nid in front_nodes])
        centroid = front_coords.mean(axis=0)
        
        # 各フロントノードにカップルフォースを適用
        # 簡易: 最も離れた2点にFz方向の力でカップル
        if len(front_nodes) >= 2:
            # Y方向に最も離れた2点を使用
            y_vals = front_coords[:, 1]
            i_top = np.argmax(y_vals)
            i_bot = np.argmin(y_vals)
            arm = abs(y_vals[i_top] - y_vals[i_bot])
            
            if arm > 1e-6:
                F_couple = torque / arm
                # Z方向の力ペア（X軸まわりトルク）
                self.add_load(Load(front_nodes[i_top], 2, F_couple))   # +Fz
                self.add_load(Load(front_nodes[i_bot], 2, -F_couple))  # -Fz
            else:
                # アームが小さい場合はモーメント直接
                for nid in front_nodes:
                    self.add_load(Load(nid, 3, torque / len(front_nodes)))
        else:
            self.add_load(Load(front_nodes[0], 3, torque))
        
        # 解析
        U = self.solve()
        
        # フロントノードのねじり角を計算
        thetas = []
        for nid in front_nodes:
            dof_start = self._node_dof_start(nid)
            # X軸まわりの回転
            uy = U[dof_start + 1]
            uz = U[dof_start + 2]
            r = self.nodes[nid].coords - centroid
            # 変位からの回転角近似
            if abs(r[1]) > 1e-6:
                theta_from_z = uz / abs(r[1])
            elif abs(r[2]) > 1e-6:
                theta_from_y = uy / abs(r[2])
            else:
                theta_from_z = 0
            
            rx = U[dof_start + 3]  # 直接の回転DOF
            thetas.append(rx)
        
        avg_theta = np.mean(np.abs(thetas))
        theta_deg = np.degrees(avg_theta)
        
        # 剛性 [Nm/deg]
        if theta_deg > 1e-12:
            K_torsion = torque / theta_deg
        else:
            K_torsion = float('inf')
        
        # 境界条件を元に戻す
        self.bcs = original_bcs
        self.loads = original_loads
        
        return K_torsion


# =============================================================================
# 4. 継手柔性補正
# =============================================================================

def joint_flexibility_factor(D_chord: float, t_chord: float,
                              D_brace: float, t_brace: float,
                              theta_deg: float = 90.0) -> float:
    """
    溶接継手の柔性補正係数 (0.0 - 1.0)
    
    CIDECT（国際管構造委員会）の経験式に基づく簡易補正
    
    Parameters:
        D_chord: 主管外径 [m]
        t_chord: 主管肉厚 [m]
        D_brace: 支管外径 [m]  
        t_brace: 支管肉厚 [m]
        theta_deg: 接合角度 [deg]
    
    Returns:
        factor: 1.0 = 完全剛結, 小さいほど柔
    
    FSAEフレームでの典型値: 0.7 - 0.9
    """
    beta = D_brace / D_chord     # 径比
    gamma = D_chord / (2 * t_chord)  # スレンダネス比
    theta = np.radians(theta_deg)
    
    # 簡易補正: gamma が大きい（薄肉）ほど柔らかい
    # beta が小さい（支管が細い）ほど局所変形が大きい
    # 典型的なFSAE管材 (25.4mm x 1.65mm ~ 31.75mm x 1.65mm) で
    # gamma ≈ 7.7 ~ 9.6, beta ≈ 0.7 ~ 1.0
    
    # 経験的補正（文献値からのフィッティング）
    factor = 1.0 - 0.15 * (gamma / 10.0) * (1.0 - beta**0.5) / np.sin(theta)
    factor = np.clip(factor, 0.5, 1.0)
    
    return factor


# =============================================================================
# 5. 最適化
# =============================================================================

@dataclass
class DesignVariable:
    """設計変数（管材グループ）"""
    group_name: str
    shape: str             # 'round' or 'square'
    D_min: float           # 外径下限 [m]
    D_max: float           # 外径上限 [m]
    t_min: float           # 肉厚下限 [m]
    t_max: float           # 肉厚上限 [m]


class FSAEFrameOptimizer:
    """
    FSAE フレーム多目的最適化
    
    目的関数: 重量最小化 + ねじり剛性最大化
    → 単目的化: f = w_mass * (mass/mass_ref) - w_stiff * (K/K_ref)
    
    制約:
    - 管材の市販サイズ制約（オプション）
    - 最小肉厚（FSAE規則）
    - 目標ねじり剛性の下限
    """
    
    def __init__(self, solver: BeamFEMSolver,
                 front_nodes: List[int],
                 rear_nodes: List[int],
                 design_vars: List[DesignVariable],
                 semantic_map=None,
                 mass_ref: Optional[float] = None,
                 K_ref: Optional[float] = None):
        self.solver = solver
        self.front_nodes = front_nodes
        self.rear_nodes = rear_nodes
        self.design_vars = design_vars
        self.semantic_map = semantic_map
        
        # 重み
        self.w_mass = 0.5
        self.w_stiff = 0.5
        
        # 正規化参照値。未指定時は初期設計（_get_x0）から決定的に計算する。
        self.mass_ref = mass_ref
        self.K_ref = K_ref
        
        # 制約
        self.K_min = None  # ねじり剛性下限 [Nm/deg]
        self.enable_compliance_penalty = True
        self.prefer_compliant_result = True
        self.compliance_penalty_scale = 100.0
        
        # 評価回数カウンター
        self.n_eval = 0
        self.eval_history = []

    def _has_compliance_context(self) -> bool:
        """レギュレーションチェック用の semantic_map が利用可能か判定"""
        return getattr(self, 'semantic_map', None) is not None

    def _compute_compliance_penalty(self, checks: List) -> float:
        """不適合件数に基づく目的関数ペナルティ

        目的関数本体は 0-1 オーダーのため、規則違反は十分大きい係数で罰する。
        固定ジオメトリ由来の違反は探索全体に対して定数項となり、
        設計変数で改善可能な違反だけが相対評価に効く。
        """
        if not checks:
            return 0.0

        severity_weights = {
            'critical': 1.0,
            'warning': 0.2,
            'info': 0.05,
        }
        failed_weight = sum(
            severity_weights.get(check.severity, 0.1)
            for check in checks
            if not check.passed
        )
        return self.compliance_penalty_scale * failed_weight
    
    def _apply_design(self, x: np.ndarray):
        """設計変数をソルバーの要素に反映"""
        idx = 0
        for dv in self.design_vars:
            D = x[idx]
            t = x[idx + 1]
            sec = TubeSection(shape=dv.shape, D_outer=D, t=t)
            
            for elem in self.solver.elements:
                if elem.group == dv.group_name:
                    elem.section = sec
            idx += 2
    
    def _get_bounds(self) -> List[Tuple[float, float]]:
        """設計変数の上下限"""
        bounds = []
        for dv in self.design_vars:
            bounds.append((dv.D_min, dv.D_max))
            bounds.append((dv.t_min, dv.t_max))
        return bounds
    
    def _get_x0(self) -> np.ndarray:
        """初期設計変数"""
        x0 = []
        for dv in self.design_vars:
            x0.append((dv.D_min + dv.D_max) / 2)
            x0.append((dv.t_min + dv.t_max) / 2)
        return np.array(x0)
    
    def _initialize_references(self):
        """固定された初期設計から正規化基準を決める（評価順・seed非依存）。"""
        if self.mass_ref is not None and self.K_ref is not None:
            return
        x_ref = self._get_x0()
        self._apply_design(x_ref)
        mass_ref = self.solver.total_mass()
        K_ref = self.solver.torsional_rigidity(self.front_nodes, self.rear_nodes)
        if not (np.isfinite(mass_ref) and mass_ref > 0):
            raise ValueError(f"Invalid reference mass: {mass_ref}")
        if not (np.isfinite(K_ref) and K_ref > 0):
            raise ValueError(f"Invalid reference torsional stiffness: {K_ref}")
        if self.mass_ref is None:
            self.mass_ref = float(mass_ref)
        if self.K_ref is None:
            self.K_ref = float(K_ref)

    def evaluate(self, x: np.ndarray) -> Dict:
        """1回の評価"""
        self._apply_design(x)
        
        mass = self.solver.total_mass()
        K_torsion = self.solver.torsional_rigidity(
            self.front_nodes, self.rear_nodes
        )
        
        self.n_eval += 1
        result = {'mass': mass, 'K_torsion': K_torsion, 'x': x.copy()}
        self.eval_history.append(result)
        
        return result

    def evaluate_with_compliance(self, x: np.ndarray) -> Dict:
        """レギュレーション適合チェック付き評価

        evaluate() の結果に FSAE 2026 ルール適合情報を付加する。
        semantic_map 属性が設定されている場合のみチェックを実行する。
        """
        from fsae_regulation_checker import FSAERegulationChecker

        result = self.evaluate(x)
        if self._has_compliance_context():
            checker = FSAERegulationChecker(self.solver, self.semantic_map)
            checks = checker.check_all()
            result['compliance'] = checks
            result['compliance_pass'] = all(c.passed for c in checks)
            result['compliance_failed'] = sum(1 for c in checks if not c.passed)
        else:
            result['compliance'] = []
            result['compliance_pass'] = True
            result['compliance_failed'] = 0
        return result
    
    def objective(self, x: np.ndarray) -> float:
        """
        単目的化された目的関数（最小化）
        
        f = w_mass * (mass/mass_ref) - w_stiff * (K/K_ref)
        """
        self._initialize_references()
        if self._has_compliance_context():
            result = self.evaluate_with_compliance(x)
        else:
            result = self.evaluate(x)
        mass = result['mass']
        K = result['K_torsion']
        
        # 正規化（基準は初期設計から決定的に設定済み）
        f_mass = mass / self.mass_ref
        f_stiff = K / self.K_ref
        
        # ペナルティ（ねじり剛性下限）
        penalty = 0.0
        if self.K_min is not None and K < self.K_min:
            penalty = 100.0 * ((self.K_min - K) / self.K_min)**2

        compliance_penalty = 0.0
        if self._has_compliance_context() and self.enable_compliance_penalty:
            compliance_penalty = self._compute_compliance_penalty(
                result.get('compliance', [])
            )
            penalty += compliance_penalty
        
        obj = self.w_mass * f_mass - self.w_stiff * f_stiff + penalty
        result['compliance_penalty'] = compliance_penalty
        result['penalty'] = penalty
        result['objective'] = obj
        return obj
    
    def optimize(self, method: str = 'differential_evolution',
                 max_iter: int = 100,
                 seed: int = 42) -> Dict:
        """
        最適化の実行
        
        method: 'differential_evolution' or 'nelder-mead'
        """
        bounds = self._get_bounds()
        x0 = self._get_x0()
        
        self.n_eval = 0
        self.eval_history = []
        
        t_start = time.time()
        
        if method == 'differential_evolution':
            result = differential_evolution(
                self.objective,
                bounds=bounds,
                maxiter=max_iter,
                seed=seed,
                tol=1e-4,
                popsize=10,
                mutation=(0.5, 1.0),
                recombination=0.7,
                workers=1  # シングルスレッド（0.5秒制約のため）
            )
        elif method == 'nelder-mead':
            result = minimize(
                self.objective,
                x0=x0,
                method='Nelder-Mead',
                options={'maxiter': max_iter, 'xatol': 1e-5, 'fatol': 1e-6}
            )
        else:
            raise ValueError(f"Unknown method: {method}")
        
        t_elapsed = time.time() - t_start
        
        # 最適化器が返した解を評価
        if self._has_compliance_context():
            best = self.evaluate_with_compliance(result.x)
        else:
            best = self.evaluate(result.x)

        # レギュレーション適合が履歴中に存在する場合は、その中で最良解を優先
        selected_from_history = False
        if self._has_compliance_context() and self.prefer_compliant_result:
            compliant_history = [
                entry for entry in self.eval_history
                if entry.get('compliance_pass', False)
            ]
            if compliant_history:
                best_history = min(
                    compliant_history,
                    key=lambda entry: entry.get('objective', float('inf'))
                )
                if not np.allclose(best_history['x'], result.x):
                    selected_from_history = True
                best = self.evaluate_with_compliance(best_history['x'])
                result_x = best_history['x']
            else:
                result_x = result.x
        else:
            result_x = result.x
        
        return {
            'x_opt': result_x,
            'mass': best['mass'],
            'K_torsion': best['K_torsion'],
            'n_eval': self.n_eval,
            'time_sec': t_elapsed,
            'converged': result.success if hasattr(result, 'success') else True,
            'history': self.eval_history,
            'compliance': best.get('compliance', []),
            'compliance_pass': best.get('compliance_pass', True),
            'compliance_failed': best.get('compliance_failed', 0),
            'compliance_penalty': best.get('compliance_penalty', 0.0),
            'selected_from_history': selected_from_history,
        }


# =============================================================================
# 6. デモ: 簡易FSAEフレーム
# =============================================================================

def create_demo_fsae_frame() -> Tuple[BeamFEMSolver, List[int], List[int]]:
    """
    簡易FSAEフレーム（主要メンバーのみ）のデモモデル
    
    座標系: X=前後, Y=左右, Z=上下
    寸法: 概略的なFSAE車両サイズ

    Returns:
        (solver, front_nodes, rear_nodes) のタプル。
        FrameSemanticMap も必要な場合は create_demo_fsae_frame_with_map() を使用。
    """
    solver = BeamFEMSolver(use_timoshenko=True)
    
    # --- 材料 ---
    steel_4130 = TubeMaterial(
        name="AISI 4130",
        E=205e9,       # 205 GPa
        G=80e9,        # 80 GPa
        rho=7850,      # 7850 kg/m^3
        yield_stress=460e6  # 460 MPa (normalized)
    )
    
    # --- 節点定義 [m] ---
    # フロントバルクヘッド
    nodes_data = [
        (1,  0.000,  0.250,  0.100),   # Front-BH top-left
        (2,  0.000, -0.250,  0.100),   # Front-BH top-right
        (3,  0.000,  0.250, -0.100),   # Front-BH bot-left
        (4,  0.000, -0.250, -0.100),   # Front-BH bot-right
        # フロントフープ
        (5,  0.500,  0.300,  0.250),   # Front-hoop top-left
        (6,  0.500, -0.300,  0.250),   # Front-hoop top-right
        (7,  0.500,  0.300, -0.150),   # Front-hoop bot-left
        (8,  0.500, -0.300, -0.150),   # Front-hoop bot-right
        # メインフープ
        (9,  1.200,  0.300,  0.500),   # Main-hoop top-left
        (10, 1.200, -0.300,  0.500),   # Main-hoop top-right
        (11, 1.200,  0.300, -0.150),   # Main-hoop bot-left
        (12, 1.200, -0.300, -0.150),   # Main-hoop bot-right
        # リア
        (13, 2.000,  0.200,  0.250),   # Rear top-left
        (14, 2.000, -0.200,  0.250),   # Rear top-right
        (15, 2.000,  0.200, -0.100),   # Rear bot-left
        (16, 2.000, -0.200, -0.100),   # Rear bot-right
    ]
    
    for nid, x, y, z in nodes_data:
        solver.add_node(Node(nid, x, y, z))
    
    # --- 断面 ---
    # FSAE規則:
    #   - ロールフープ系: Size A 適合が必要
    #   - ブレース系: Size B 適合を最適化で満たす
    sec_main = TubeSection('round', 0.02540, 0.00165)    # 1" x 0.065"
    sec_roll = TubeSection('round', 0.03175, 0.00211)    # 1.25" x 0.083" (Size A 適合)
    sec_brace = TubeSection('round', 0.02540, 0.00120)   # 1" x 0.049"
    
    # --- 要素定義 ---
    elements_data = [
        # フロントバルクヘッド (枠)
        (1, 1, 2, sec_main, 'front_bh'),
        (2, 3, 4, sec_main, 'front_bh'),
        (3, 1, 3, sec_main, 'front_bh'),
        (4, 2, 4, sec_main, 'front_bh'),
        
        # フロント→フープ (上側レール)
        (5, 1, 5, sec_main, 'upper_rail'),
        (6, 2, 6, sec_main, 'upper_rail'),
        # フロント→フープ (下側レール)
        (7, 3, 7, sec_main, 'lower_rail'),
        (8, 4, 8, sec_main, 'lower_rail'),
        
        # フロントフープ
        (9,  5, 6, sec_roll, 'front_hoop'),
        (10, 7, 8, sec_roll, 'front_hoop'),
        (11, 5, 7, sec_roll, 'front_hoop'),
        (12, 6, 8, sec_roll, 'front_hoop'),
        
        # サイドレール (フロントフープ→メインフープ)
        (13, 5, 9,  sec_main, 'upper_rail'),
        (14, 6, 10, sec_main, 'upper_rail'),
        (15, 7, 11, sec_main, 'lower_rail'),
        (16, 8, 12, sec_main, 'lower_rail'),
        
        # メインフープ
        (17, 9,  10, sec_roll, 'main_hoop'),
        (18, 11, 12, sec_roll, 'main_hoop'),
        (19, 9,  11, sec_roll, 'main_hoop'),
        (20, 10, 12, sec_roll, 'main_hoop'),
        
        # メインフープ→リア
        (21, 9,  13, sec_main, 'upper_rail'),
        (22, 10, 14, sec_main, 'upper_rail'),
        (23, 11, 15, sec_main, 'lower_rail'),
        (24, 12, 16, sec_main, 'lower_rail'),
        
        # リア枠
        (25, 13, 14, sec_main, 'rear_frame'),
        (26, 15, 16, sec_main, 'rear_frame'),
        (27, 13, 15, sec_main, 'rear_frame'),
        (28, 14, 16, sec_main, 'rear_frame'),
        
        # 対角ブレース（ねじり剛性向上）
        (29, 5, 8,  sec_brace, 'diagonal'),  # フロントフープ X
        (30, 9, 12, sec_brace, 'diagonal'),  # メインフープ X
        (31, 5, 12, sec_brace, 'side_brace'),  # 左サイド対角
        (32, 6, 11, sec_brace, 'side_brace'),  # 右サイド対角
        (33, 1, 4,  sec_brace, 'diagonal'),  # フロントBH X
        (34, 13, 16, sec_brace, 'diagonal'),  # リア X
    ]
    
    elem_id = 0
    for data in elements_data:
        eid, ni, nj, sec, group = data
        jf = joint_flexibility_factor(
            sec.D_outer, sec.t, sec.D_outer, sec.t, 90.0
        )
        elem = Element(eid, ni, nj, sec, steel_4130, group, jf)
        solver.add_element(elem)
        elem_id += 1
    
    front_nodes = [1, 2, 3, 4]
    rear_nodes = [13, 14, 15, 16]
    
    return solver, front_nodes, rear_nodes


def create_demo_fsae_frame_with_map():
    """create_demo_fsae_frame() に FrameSemanticMap を付加して返す。

    Returns:
        (solver, front_nodes, rear_nodes, semantic_map) のタプル。
    """
    from fsae_regulation_checker import FrameSemanticMap

    solver, front_nodes, rear_nodes = create_demo_fsae_frame()
    semantic_map = FrameSemanticMap(
        # デモフレームのデフォルト節点マッピング
        main_hoop_top_left=9,
        main_hoop_top_right=10,
        main_hoop_bot_left=11,
        main_hoop_bot_right=12,
        front_hoop_top_left=5,
        front_hoop_top_right=6,
        front_hoop_bot_left=7,
        front_hoop_bot_right=8,
        front_bh_top_left=1,
        front_bh_top_right=2,
        front_bh_bot_left=3,
        front_bh_bot_right=4,
        rear_top_left=13,
        rear_top_right=14,
        rear_bot_left=15,
        rear_bot_right=16,
        brace_top_left=9,
        brace_top_right=10,
        brace_bot_left=13,
        brace_bot_right=14,
        front_axle_x=0.0,
        rear_axle_x=2.0,
        track_front=1.2,
        track_rear=1.15,
        side_impact_height=0.310,
        ia_length=0.300,
        ia_height=0.150,
        ia_width=0.250,
    )
    return solver, front_nodes, rear_nodes, semantic_map


def run_demo():
    """デモの実行"""
    print("=" * 70)
    print("FSAE Frame Optimization - Demo")
    print("=" * 70)
    
    # --- 1. ベースライン評価 ---
    print("\n[1] ベースライン評価...")
    solver, front_nodes, rear_nodes, semantic_map = create_demo_fsae_frame_with_map()
    
    t0 = time.time()
    mass = solver.total_mass()
    K = solver.torsional_rigidity(front_nodes, rear_nodes, torque=1000.0)
    t_eval = time.time() - t0
    
    print(f"  総重量:     {mass:.3f} kg")
    print(f"  ねじり剛性: {K:.1f} Nm/deg")
    print(f"  評価時間:   {t_eval*1000:.1f} ms")
    
    # --- 2. 最適化 ---
    print("\n[2] 最適化実行...")
    
    # 設計変数: 4グループ
    design_vars = [
        DesignVariable('upper_rail', 'round', 0.020, 0.035, 0.0012, 0.0025),
        DesignVariable('lower_rail', 'round', 0.020, 0.035, 0.0012, 0.0025),
        DesignVariable('diagonal',   'round', 0.0254, 0.035, 0.0012, 0.0025),
        DesignVariable('side_brace', 'round', 0.0254, 0.035, 0.0012, 0.0025),
    ]
    
    optimizer = FSAEFrameOptimizer(
        solver, front_nodes, rear_nodes, design_vars, semantic_map=semantic_map
    )
    optimizer.w_mass = 0.6
    optimizer.w_stiff = 0.4
    optimizer.K_min = K * 0.8  # 現状の80%以上を確保
    
    result = optimizer.optimize(method='differential_evolution', max_iter=50, seed=42)
    
    print(f"\n[結果]")
    print(f"  最適重量:     {result['mass']:.3f} kg "
          f"(ベースライン比 {result['mass']/mass*100:.1f}%)")
    print(f"  ねじり剛性:   {result['K_torsion']:.1f} Nm/deg "
          f"(ベースライン比 {result['K_torsion']/K*100:.1f}%)")
    print(f"  評価回数:     {result['n_eval']}")
    print(f"  総計算時間:   {result['time_sec']:.2f} 秒")
    print(f"  1評価あたり:  {result['time_sec']/result['n_eval']*1000:.1f} ms")
    print(f"  レギュ適合:   {'PASS' if result['compliance_pass'] else 'FAIL'}")
    print(f"  違反件数:     {result['compliance_failed']}")
    
    # 最適断面の表示
    print(f"\n[最適断面]")
    idx = 0
    for dv in design_vars:
        D = result['x_opt'][idx] * 1000  # mm
        t = result['x_opt'][idx+1] * 1000  # mm
        print(f"  {dv.group_name:15s}: φ{D:.1f} x t{t:.2f} mm")
        idx += 2
    
    print("\n" + "=" * 70)
    return result


if __name__ == "__main__":
    run_demo()
