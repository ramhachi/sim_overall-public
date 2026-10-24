"""
FSAE 2026 フレームレギュレーション適合チェッカー
================================================
FSAE 2026ルールに基づくフレーム形状・管材仕様の適合性検証モジュール。

対象ルール:
- F.3.2: 管材仕様 (Size A/B/C/D)
- F.5.7/F.5.8: フープ角度制限
- F.5.9: ブレース要件
- F.6.4: サイドインパクト高さ
- F.8.4/F.8.8: インパクトアッテネータ
- V.1.2/V.1.3: ホイールベース・トレッド比
"""

import math
import sys
import os
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

# fsae_frame_optimizer と同じディレクトリからインポート
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from fsae_frame_optimizer import (
    BeamFEMSolver,
    Element,
    Node,
    TubeMaterial,
    TubeSection,
)

# =============================================================================
# FSAE 2026 管材仕様 (F.3.2)
# =============================================================================
TUBE_SPECS: Dict[str, Dict[str, float]] = {
    "Size_A": {
        "I_min": 11320e-12,   # [m⁴] 断面二次モーメント
        "A_min": 173e-6,      # [m²] 断面積
        "OD_min": 0.025,      # [m]  最小外径
        "t_min": 0.002,       # [m]  最小肉厚
    },
    "Size_B": {
        "I_min": 8509e-12,
        "A_min": 114e-6,
        "OD_min": 0.025,
        "t_min": 0.0012,
    },
    "Size_C": {
        "I_min": 6695e-12,
        "A_min": 91e-6,
        "OD_min": 0.025,
        "t_min": 0.0012,
    },
    "Size_D": {
        "I_min": 18015e-12,
        "A_min": 126e-6,
        "OD_min": 0.035,
        "t_min": 0.0012,
    },
}

# 要素グループ → 要求管材仕様のデフォルトマッピング
# V2 のグループ名を正とし、V1 の名前はエイリアスとして残す。
DEFAULT_GROUP_SPEC_MAP: Dict[str, str] = {
    # V1/V2 common
    "main_hoop": "Size_A",
    "front_hoop": "Size_A",
    "side_impact": "Size_B",
    "side_impact_bent": "Size_D",
    "shoulder_bar": "Size_A",
    "shoulder": "Size_A",
    "mh_bracing": "Size_B",
    "mh_brace_support": "Size_C",
    "front_bulkhead": "Size_B",
    "fbh_support": "Size_C",
    "front_susp_mount": "Size_C",
    "fh_bracing": "Size_B",
    "component_protection": "Size_C",
    "diff_mount": "Size_C",
    "rear_box": "Size_C",
    "secondary_brace": "Size_C",
    "structural": "Size_B",
    # V1 aliases retained for the demo/legacy frame
    "upper_rail": "Size_B",
    "lower_rail": "Size_B",
    "front_bh": "Size_B",
    "rear_frame": "Size_C",
    "diagonal": "Size_B",
    "diagonal_brace": "Size_C",
    "side_brace": "Size_B",
}

V1_REQUIRED_GROUPS = {
    "front_hoop",
    "main_hoop",
    "upper_rail",
    "lower_rail",
    "front_bh",
    "rear_frame",
}

V2_REQUIRED_GROUPS = {
    "front_hoop",
    "main_hoop",
    "side_impact",
    "front_bulkhead",
    "shoulder_bar",
    "mh_bracing",
    "fbh_support",
    "mh_brace_support",
    "component_protection",
    "diff_mount",
    "rear_box",
    "secondary_brace",
}

V2_MARKER_GROUPS = {
    "front_bulkhead",
    "shoulder_bar",
    "mh_bracing",
    "fbh_support",
    "mh_brace_support",
    "rear_box",
    "secondary_brace",
    "component_protection",
    "diff_mount",
    "front_susp_mount",
}


# =============================================================================
# データ構造
# =============================================================================
@dataclass
class RegulationCheckResult:
    """個別レギュレーションチェック結果"""

    rule_id: str          # 例: "F.5.8.3"
    category: str         # "frame", "suspension", "safety", "powertrain", "aero"
    description: str      # ルール説明
    passed: bool
    value: str            # 実測値
    required: str         # 要求値
    severity: str         # "critical", "warning", "info"


@dataclass
class FrameSemanticMap:
    """フレーム節点IDとセマンティックロールのマッピング

    節点IDを意味的役割に対応付ける。create_demo_fsae_frame() のデフォルト
    レイアウトに基づくデフォルト値を持つ。
    """

    # メインフープ節点
    main_hoop_top_left: int = 9
    main_hoop_top_right: int = 10
    main_hoop_bot_left: int = 11
    main_hoop_bot_right: int = 12

    # フロントフープ節点
    front_hoop_top_left: int = 5
    front_hoop_top_right: int = 6
    front_hoop_bot_left: int = 7
    front_hoop_bot_right: int = 8

    # フロントバルクヘッド節点
    front_bh_top_left: int = 1
    front_bh_top_right: int = 2
    front_bh_bot_left: int = 3
    front_bh_bot_right: int = 4

    # リア節点（ブレース接続点を兼ねる）
    rear_top_left: int = 13
    rear_top_right: int = 14
    rear_bot_left: int = 15
    rear_bot_right: int = 16

    # ブレース接続: メインフープ上での接続点
    # デフォルトではメインフープ頂部に直接接続
    brace_top_left: int = 9
    brace_top_right: int = 10
    brace_bot_left: int = 13   # ブレースがリアへ向かう接続先
    brace_bot_right: int = 14

    # ホイールベース参照節点 (前軸x, 後軸x)
    front_axle_x: float = 0.0    # [m] フロントアクスルX座標
    rear_axle_x: float = 2.0     # [m] リアアクスルX座標

    # トレック幅参照値 [m]
    track_front: float = 1.2
    track_rear: float = 1.15

    # サイドインパクト参照高さ [m] (フレーム最低点からの相対値)
    side_impact_height: float = 0.310

    # インパクトアッテネータ寸法 [m]
    ia_length: float = 0.300
    ia_height: float = 0.150
    ia_width: float = 0.250

    # 要素グループ → 管材仕様マッピング
    group_spec_map: Dict[str, str] = field(default_factory=lambda: dict(DEFAULT_GROUP_SPEC_MAP))


# =============================================================================
# メインチェッカークラス
# =============================================================================
class FSAERegulationChecker:
    """FSAE 2026 フレームレギュレーション適合チェッカー

    BeamFEMSolver と FrameSemanticMap を用いてフレーム形状・管材仕様の
    FSAE 2026ルール適合性を検証する。
    """

    def __init__(self, solver: BeamFEMSolver, semantic_map: FrameSemanticMap):
        self.solver = solver
        self.smap = semantic_map

    # -----------------------------------------------------------------
    # ユーティリティ
    # -----------------------------------------------------------------
    def _get_node(self, node_id: int) -> Node:
        return self.solver.nodes[node_id]

    def _calc_angle_from_vertical(self, bot_node_id: int, top_node_id: int) -> float:
        """2節点間のベクトルとZ軸正方向（鉛直上方）のなす角 [deg]"""
        bot = self._get_node(bot_node_id)
        top = self._get_node(top_node_id)
        vec = np.array([top.x - bot.x, top.y - bot.y, top.z - bot.z])
        vertical = np.array([0.0, 0.0, 1.0])
        cos_angle = np.dot(vec, vertical) / (np.linalg.norm(vec) * np.linalg.norm(vertical))
        return float(np.degrees(np.arccos(np.clip(cos_angle, -1.0, 1.0))))

    def _calc_angle_between_vectors(
        self, v1: np.ndarray, v2: np.ndarray
    ) -> float:
        """2ベクトル間の角度 [deg]"""
        cos_a = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
        return float(np.degrees(np.arccos(np.clip(cos_a, -1.0, 1.0))))

    def _lowest_frame_z(self) -> float:
        """フレーム最低点のZ座標"""
        return min(n.z for n in self.solver.nodes.values())

    # -----------------------------------------------------------------
    # 管材仕様チェック (F.3.2)
    # -----------------------------------------------------------------
    def check_tube_specs(self) -> List[RegulationCheckResult]:
        """F.3.2: 要素グループ毎に管材仕様を検証"""
        results: List[RegulationCheckResult] = []
        # グループ毎にまとめてチェック
        checked_groups: Dict[str, List[Element]] = {}
        for elem in self.solver.elements:
            if elem.group not in checked_groups:
                checked_groups[elem.group] = []
            checked_groups[elem.group].append(elem)

        for group_name, elements in checked_groups.items():
            spec_name = self.smap.group_spec_map.get(group_name)
            if spec_name is None:
                results.append(
                    RegulationCheckResult(
                        rule_id="F.3.2",
                        category="frame",
                        description=f"{group_name}: 管材仕様が未定義のgroup",
                        passed=False,
                        value=f"{len(elements)} elements",
                        required="group_spec_map entry",
                        severity="critical",
                    )
                )
                continue
            spec = TUBE_SPECS[spec_name]
            n_elem = len(elements)
            min_od = min(e.section.D_outer for e in elements)
            min_t = min(e.section.t for e in elements)
            min_a = min(e.section.A for e in elements)
            min_i = min(min(e.section.Iy, e.section.Iz) for e in elements)

            def add_metric(metric: str, value: float, required: float, unit: str, scale: float,
                           tolerance: float) -> None:
                passed = value >= required - tolerance
                results.append(
                    RegulationCheckResult(
                        rule_id="F.3.2",
                        category="frame",
                        description=(
                            f"{group_name} ({spec_name}) {metric} "
                            f"(min of {n_elem} elements)"
                        ),
                        passed=passed,
                        value=f"{value * scale:.3f} {unit}",
                        required=f">= {required * scale:.3f} {unit}",
                        severity="critical",
                    )
                )

            add_metric("外径 OD", min_od, spec["OD_min"], "mm", 1000.0, 1e-9)
            add_metric("肉厚 t", min_t, spec["t_min"], "mm", 1000.0, 1e-9)
            add_metric("断面積 A", min_a, spec["A_min"], "mm^2", 1e6, 1e-12)
            add_metric("断面二次モーメント I", min_i, spec["I_min"], "mm^4", 1e12, 1e-18)

            material_names = {e.material.name for e in elements}
            if len(material_names) > 1:
                results.append(
                    RegulationCheckResult(
                        rule_id="F.3.2",
                        category="frame",
                        description=f"{group_name}: 同一group内で材料が混在",
                        passed=False,
                        value=", ".join(sorted(material_names)),
                        required="single material per group",
                        severity="critical",
                    )
                )

        return results

    # -----------------------------------------------------------------
    # 角度チェック
    # -----------------------------------------------------------------
    def check_main_hoop_angle(self) -> RegulationCheckResult:
        """F.5.8.3: メインフープ鉛直からの角度 ≤ 10°"""
        sm = self.smap
        angle_left = self._calc_angle_from_vertical(
            sm.main_hoop_bot_left, sm.main_hoop_top_left
        )
        angle_right = self._calc_angle_from_vertical(
            sm.main_hoop_bot_right, sm.main_hoop_top_right
        )
        max_angle = max(angle_left, angle_right)
        passed = max_angle <= 10.0 + 1e-9
        return RegulationCheckResult(
            rule_id="F.5.8.3",
            category="frame",
            description="メインフープ鉛直角度 ≤ 10°",
            passed=passed,
            value=f"{max_angle:.2f}°",
            required="≤ 10°",
            severity="critical",
        )

    def check_front_hoop_angle(self) -> RegulationCheckResult:
        """F.5.7.6: フロントフープ鉛直からの角度 ≤ 20°"""
        sm = self.smap
        angle_left = self._calc_angle_from_vertical(
            sm.front_hoop_bot_left, sm.front_hoop_top_left
        )
        angle_right = self._calc_angle_from_vertical(
            sm.front_hoop_bot_right, sm.front_hoop_top_right
        )
        max_angle = max(angle_left, angle_right)
        passed = max_angle <= 20.0 + 1e-9
        return RegulationCheckResult(
            rule_id="F.5.7.6",
            category="frame",
            description="フロントフープ鉛直角度 ≤ 20°",
            passed=passed,
            value=f"{max_angle:.2f}°",
            required="≤ 20°",
            severity="critical",
        )

    def check_brace_angle(self) -> RegulationCheckResult:
        """F.5.9.5: ブレースとメインフープの挟角 ≥ 30°"""
        sm = self.smap
        # メインフープ鉛直ベクトル（左側）
        mh_bot = self._get_node(sm.main_hoop_bot_left)
        mh_top = self._get_node(sm.main_hoop_top_left)
        hoop_vec = np.array([
            mh_top.x - mh_bot.x,
            mh_top.y - mh_bot.y,
            mh_top.z - mh_bot.z,
        ])
        # ブレースベクトル（メインフープ頂部 → リア接続点）
        brace_top = self._get_node(sm.brace_top_left)
        brace_bot = self._get_node(sm.brace_bot_left)
        brace_vec = np.array([
            brace_bot.x - brace_top.x,
            brace_bot.y - brace_top.y,
            brace_bot.z - brace_top.z,
        ])
        angle = self._calc_angle_between_vectors(hoop_vec, brace_vec)
        passed = angle >= 30.0 - 1e-9
        return RegulationCheckResult(
            rule_id="F.5.9.5",
            category="frame",
            description="ブレース挟角 ≥ 30°",
            passed=passed,
            value=f"{angle:.2f}°",
            required="≥ 30°",
            severity="critical",
        )

    def check_brace_attachment_height(self) -> RegulationCheckResult:
        """F.5.9.4: ブレース接続点はメインフープ頂部から160mm以内"""
        sm = self.smap
        top_z = self._get_node(sm.main_hoop_top_left).z
        attach_z = self._get_node(sm.brace_top_left).z
        drop_mm = (top_z - attach_z) * 1000.0
        passed = drop_mm <= 160.0 + 1e-6
        return RegulationCheckResult(
            rule_id="F.5.9.4",
            category="frame",
            description="ブレース接続点 ≤ 160 mm below メインフープ頂部",
            passed=passed,
            value=f"{drop_mm:.1f} mm",
            required="≤ 160 mm",
            severity="critical",
        )

    def check_main_hoop_width(self) -> RegulationCheckResult:
        """F.5.8.4: メインフープ底部の内寸幅 ≥ 380mm"""
        sm = self.smap
        bot_l = self._get_node(sm.main_hoop_bot_left)
        bot_r = self._get_node(sm.main_hoop_bot_right)
        outer_width = abs(bot_l.y - bot_r.y)
        # 内寸 = 外寸 - チューブ外径（両側）。混在時は最大径で保守的に評価。
        tube_od = max(
            (e.section.D_outer for e in self.solver.elements if e.group == "main_hoop"),
            default=0.0,
        )
        inner_width = outer_width - tube_od
        inner_width_mm = inner_width * 1000.0
        passed = inner_width_mm >= 380.0 - 1e-6
        return RegulationCheckResult(
            rule_id="F.5.8.4",
            category="frame",
            description="メインフープ底部内寸幅 ≥ 380 mm",
            passed=passed,
            value=f"{inner_width_mm:.1f} mm",
            required="≥ 380 mm",
            severity="critical",
        )

    # -----------------------------------------------------------------
    # 車両寸法チェック
    # -----------------------------------------------------------------
    def check_wheelbase(self) -> RegulationCheckResult:
        """V.1.2: ホイールベース ≥ 1525mm"""
        wb_m = abs(self.smap.rear_axle_x - self.smap.front_axle_x)
        wb_mm = wb_m * 1000.0
        passed = wb_mm >= 1525.0 - 1e-6
        return RegulationCheckResult(
            rule_id="V.1.2",
            category="frame",
            description="ホイールベース ≥ 1525 mm",
            passed=passed,
            value=f"{wb_mm:.0f} mm",
            required="≥ 1525 mm",
            severity="critical",
        )

    def check_track_ratio(self) -> RegulationCheckResult:
        """V.1.3.2: トレッド比 (小/大) ≥ 75%"""
        t_f = self.smap.track_front
        t_r = self.smap.track_rear
        ratio = min(t_f, t_r) / max(t_f, t_r) if max(t_f, t_r) > 0 else 0
        ratio_pct = ratio * 100.0
        passed = ratio_pct >= 75.0 - 1e-6
        return RegulationCheckResult(
            rule_id="V.1.3.2",
            category="frame",
            description="トレッド比 (小/大) ≥ 75%",
            passed=passed,
            value=f"{ratio_pct:.1f}%",
            required="≥ 75%",
            severity="critical",
        )

    def check_side_impact_height(self) -> RegulationCheckResult:
        """F.6.4.4: アッパーサイドインパクト高さ 290-345mm (フレーム最低点基準)"""
        lowest_z = self._lowest_frame_z()
        side_elements = [
            e for e in self.solver.elements
            if e.group in ("side_impact", "side_impact_bent")
        ]
        if side_elements:
            node_ids = {e.node_i for e in side_elements} | {e.node_j for e in side_elements}
            upper_z = max(self._get_node(nid).z for nid in node_ids)
            height_from_lowest = (upper_z - lowest_z) * 1000.0
            source = "side_impact nodes"
        else:
            # Legacy/demo frames may not define a side_impact group. The
            # semantic map value is a frame-lowest-relative height in metres.
            height_from_lowest = self.smap.side_impact_height * 1000.0
            source = "semantic map"
        passed = 290.0 - 1e-6 <= height_from_lowest <= 345.0 + 1e-6
        return RegulationCheckResult(
            rule_id="F.6.4.4",
            category="frame",
            description=f"アッパーサイドインパクト高さ 290-345 mm ({source})",
            passed=passed,
            value=f"{height_from_lowest:.1f} mm",
            required="290-345 mm",
            severity="critical",
        )

    # -----------------------------------------------------------------
    # インパクトアッテネータ (F.8.4 / F.8.8)
    # -----------------------------------------------------------------
    def check_impact_attenuator_dimensions(self) -> RegulationCheckResult:
        """F.8.4.2: IA寸法 — 長さ≥200mm, 高さ≥100mm, 幅≥200mm"""
        sm = self.smap
        l_ok = sm.ia_length >= 0.200 - 1e-9
        h_ok = sm.ia_height >= 0.100 - 1e-9
        w_ok = sm.ia_width >= 0.200 - 1e-9
        passed = l_ok and h_ok and w_ok
        value_str = (
            f"L={sm.ia_length*1000:.0f}, "
            f"H={sm.ia_height*1000:.0f}, "
            f"W={sm.ia_width*1000:.0f} mm"
        )
        return RegulationCheckResult(
            rule_id="F.8.4.2",
            category="safety",
            description="IA寸法: L≥200, H≥100, W≥200 mm",
            passed=passed,
            value=value_str,
            required="L≥200, H≥100, W≥200 mm",
            severity="critical",
        )

    def check_impact_attenuator_energy(self) -> RegulationCheckResult:
        """F.8.8.2: IA エネルギー吸収 ≥ 7350J (300kg @ 7m/s)"""
        # エネルギー吸収は設計仕様パラメータ — ここでは寸法ベースの簡易推定
        # 0.5 * 300 * 7^2 = 7350 J
        required_energy = 7350.0  # [J]
        # 簡易推定: IA体積 × 想定吸収エネルギー密度 (アルミハニカム ~4 MJ/m³)
        volume = self.smap.ia_length * self.smap.ia_height * self.smap.ia_width
        energy_density = 4.0e6  # [J/m³] アルミハニカムの典型値
        estimated_energy = volume * energy_density
        passed = estimated_energy >= required_energy
        return RegulationCheckResult(
            rule_id="F.8.8.2",
            category="safety",
            description="IA エネルギー吸収 ≥ 7350 J",
            passed=passed,
            value=f"{estimated_energy:.0f} J (推定)",
            required=f"≥ {required_energy:.0f} J",
            severity="critical",
        )

    # -----------------------------------------------------------------
    # 材料チェック
    # -----------------------------------------------------------------
    def check_main_hoop_material(self) -> RegulationCheckResult:
        """F.3.2.1.g: メインフープは鉄鋼のみ（アルミ不可）"""
        is_steel = True
        mat_name = "N/A"
        for elem in self.solver.elements:
            if elem.group == "main_hoop":
                mat_name = elem.material.name
                name_lower = mat_name.lower()
                if "aluminum" in name_lower or "aluminium" in name_lower or "al " in name_lower:
                    is_steel = False
        return RegulationCheckResult(
            rule_id="F.3.2.1.g",
            category="frame",
            description="メインフープ: 鉄鋼材のみ",
            passed=is_steel,
            value=mat_name,
            required="Steel only (no aluminum)",
            severity="critical",
        )

    def check_bracing_material(self) -> RegulationCheckResult:
        """F.3.2.1.h: ブレースは Size B 鉄鋼 (アルミの場合 Size A 等価必要)"""
        passed = True
        details = []
        for elem in self.solver.elements:
            if elem.group in ("diagonal", "side_brace"):
                name_lower = elem.material.name.lower()
                is_aluminum = (
                    "aluminum" in name_lower
                    or "aluminium" in name_lower
                    or "al " in name_lower
                )
                if is_aluminum:
                    # アルミの場合 Size A 等価が必要
                    spec = TUBE_SPECS["Size_A"]
                    if elem.section.A < spec["A_min"] or min(elem.section.Iy, elem.section.Iz) < spec["I_min"]:
                        passed = False
                        details.append(f"Elem {elem.id}: Al bracing below Size A equiv")
                else:
                    # 鉄鋼の場合 Size B が必要
                    spec = TUBE_SPECS["Size_B"]
                    if elem.section.A < spec["A_min"] - 1e-12 or min(elem.section.Iy, elem.section.Iz) < spec["I_min"] - 1e-18:
                        passed = False
                        details.append(f"Elem {elem.id}: Steel bracing below Size B")

        value_str = "OK" if passed else "; ".join(details)
        return RegulationCheckResult(
            rule_id="F.3.2.1.h",
            category="frame",
            description="ブレース材料: Size B steel (or Al with Size A equiv)",
            passed=passed,
            value=value_str,
            required="Size B steel or Size A aluminum equivalent",
            severity="critical",
        )

    def check_required_groups(self) -> RegulationCheckResult:
        """必須構造グループの存在を検査する。"""
        present = {e.group for e in self.solver.elements}
        is_v2 = bool(present & V2_MARKER_GROUPS)
        required = V2_REQUIRED_GROUPS if is_v2 else V1_REQUIRED_GROUPS
        missing = sorted(required - present)
        return RegulationCheckResult(
            rule_id="F.3.1",
            category="frame",
            description="必須構造グループの存在",
            passed=not missing,
            value="all present" if not missing else "missing: " + ", ".join(missing),
            required=", ".join(sorted(required)),
            severity="critical",
        )

    # -----------------------------------------------------------------
    # 全チェック実行
    # -----------------------------------------------------------------
    def check_all(self) -> List[RegulationCheckResult]:
        """全レギュレーションチェックを実行"""
        results: List[RegulationCheckResult] = []

        # 必須構造
        results.append(self.check_required_groups())

        # 管材仕様
        results.extend(self.check_tube_specs())

        # 形状チェック
        results.append(self.check_main_hoop_angle())
        results.append(self.check_front_hoop_angle())
        results.append(self.check_brace_angle())
        results.append(self.check_brace_attachment_height())
        results.append(self.check_main_hoop_width())

        # 車両寸法
        results.append(self.check_wheelbase())
        results.append(self.check_track_ratio())

        # サイドインパクト
        results.append(self.check_side_impact_height())

        # インパクトアッテネータ
        results.append(self.check_impact_attenuator_dimensions())
        results.append(self.check_impact_attenuator_energy())

        # 材料
        results.append(self.check_main_hoop_material())
        results.append(self.check_bracing_material())

        return results

    # -----------------------------------------------------------------
    # レポート生成
    # -----------------------------------------------------------------
    def generate_report(self, results: List[RegulationCheckResult]) -> str:
        """人間可読なコンプライアンスレポートを生成"""
        lines = [
            "=" * 70,
            "FSAE 2026 Frame Regulation Compliance Report",
            "=" * 70,
        ]

        passed_count = sum(1 for r in results if r.passed)
        total = len(results)
        lines.append(f"\n総合: {passed_count}/{total} 項目適合")
        lines.append(f"結果: {'✅ ALL PASS' if passed_count == total else '❌ VIOLATIONS FOUND'}")
        lines.append("")

        # カテゴリ別にグループ化
        categories: Dict[str, List[RegulationCheckResult]] = {}
        for r in results:
            categories.setdefault(r.category, []).append(r)

        for cat, cat_results in categories.items():
            lines.append(f"--- {cat.upper()} ---")
            for r in cat_results:
                status = "✅" if r.passed else "❌"
                lines.append(
                    f"  {status} [{r.rule_id}] {r.description}"
                )
                lines.append(
                    f"     実測: {r.value}  |  要求: {r.required}"
                )
            lines.append("")

        lines.append("=" * 70)
        return "\n".join(lines)

    def generate_report_dict(self, results: List[RegulationCheckResult]) -> dict:
        """ダッシュボードAPI用のJSON直列化可能レポートを生成"""
        checks = []
        for r in results:
            checks.append(
                {
                    "id": r.rule_id,
                    "category": r.category,
                    "rule": r.description,
                    "passed": r.passed,
                    "value": r.value,
                    "required": r.required,
                    "severity": r.severity,
                }
            )
        return {
            "overall": all(r.passed for r in results),
            "total": len(results),
            "passed": sum(1 for r in results if r.passed),
            "failed": sum(1 for r in results if not r.passed),
            "checks": checks,
        }


# =============================================================================
# スタンドアロン実行
# =============================================================================
if __name__ == "__main__":
    from fsae_frame_optimizer import create_demo_fsae_frame

    solver, front_nodes, rear_nodes = create_demo_fsae_frame()
    smap = FrameSemanticMap()
    checker = FSAERegulationChecker(solver, smap)
    results = checker.check_all()
    print(checker.generate_report(results))
