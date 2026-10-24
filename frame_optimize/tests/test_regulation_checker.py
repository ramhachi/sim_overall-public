"""
FSAE 2026 レギュレーションチェッカー テスト
==========================================
管材仕様・角度計算・全チェック・レポート生成の検証。
"""

import math
import os
import sys

import numpy as np
import pytest

# frame_optimize ディレクトリをパスに追加
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from fsae_frame_optimizer import (
    BeamFEMSolver,
    Element,
    Node,
    TubeMaterial,
    TubeSection,
    create_demo_fsae_frame,
)
from fsae_regulation_checker import (
    DEFAULT_GROUP_SPEC_MAP,
    TUBE_SPECS,
    FrameSemanticMap,
    FSAERegulationChecker,
    RegulationCheckResult,
)


# =============================================================================
# フィクスチャ
# =============================================================================
@pytest.fixture
def demo_frame():
    """デモフレーム (solver, front_nodes, rear_nodes) を返す"""
    return create_demo_fsae_frame()


@pytest.fixture
def default_smap():
    """デフォルト FrameSemanticMap を返す"""
    return FrameSemanticMap()


@pytest.fixture
def checker(demo_frame, default_smap):
    """デモフレームとデフォルトマップからチェッカーを構築"""
    solver, _, _ = demo_frame
    return FSAERegulationChecker(solver, default_smap)


# =============================================================================
# 管材仕様チェック
# =============================================================================
class TestTubeSpecs:
    """F.3.2 管材仕様チェックのテスト"""

    def test_tube_spec_returns_results(self, checker):
        results = checker.check_tube_specs()
        assert len(results) > 0
        assert all(isinstance(r, RegulationCheckResult) for r in results)

    def test_tube_spec_all_have_rule_id(self, checker):
        results = checker.check_tube_specs()
        for r in results:
            assert r.rule_id == "F.3.2"

    def test_main_hoop_roll_tube_od_pass(self, checker):
        """メインフープ (31.75mm) は Size A OD (25mm) をクリア"""
        results = checker.check_tube_specs()
        main_hoop_od = [
            r for r in results
            if "main_hoop" in r.description and "外径" in r.description
        ]
        assert len(main_hoop_od) > 0
        for r in main_hoop_od:
            assert r.passed, f"Main hoop OD should pass: {r.value}"

    def test_brace_tube_od_pass(self, checker):
        """ブレース (25.4mm) は Size B OD (25mm) をクリア"""
        results = checker.check_tube_specs()
        brace_od = [
            r for r in results
            if "diagonal" in r.description and "外径" in r.description
        ]
        assert len(brace_od) > 0
        for r in brace_od:
            assert r.passed, f"Brace OD should pass: {r.value}"

    def test_tube_spec_fail_thin_wall(self):
        """肉厚不足の場合にFAILすることを検証"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        solver.add_node(Node(2, 1, 0, 0))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        # Size A 要件: t ≥ 2.0mm だが 1.0mm を設定
        thin_sec = TubeSection("round", 0.025, 0.001)
        solver.add_element(Element(1, 1, 2, thin_sec, mat, "main_hoop"))

        smap = FrameSemanticMap()
        checker = FSAERegulationChecker(solver, smap)
        results = checker.check_tube_specs()
        t_results = [r for r in results if "肉厚" in r.description and "main_hoop" in r.description]
        assert len(t_results) > 0
        assert not t_results[0].passed, "Thin wall should fail"

    def test_tube_spec_fail_small_od(self):
        """外径不足の場合にFAILすることを検証"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        solver.add_node(Node(2, 1, 0, 0))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        small_sec = TubeSection("round", 0.020, 0.002)  # OD 20mm < 25mm required
        solver.add_element(Element(1, 1, 2, small_sec, mat, "main_hoop"))

        smap = FrameSemanticMap()
        checker = FSAERegulationChecker(solver, smap)
        results = checker.check_tube_specs()
        od_results = [r for r in results if "外径" in r.description and "main_hoop" in r.description]
        assert len(od_results) > 0
        assert not od_results[0].passed, "Small OD should fail"

    def test_mixed_group_checks_every_element(self):
        """同じgroup内の2番目以降の違反も検出する"""
        solver = BeamFEMSolver()
        for i, z in enumerate([0.0, 1.0, 2.0, 3.0], start=1):
            solver.add_node(Node(i, z, 0, 0))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        good = TubeSection("round", 0.03175, 0.00211)
        thin = TubeSection("round", 0.0254, 0.00165)
        solver.add_element(Element(1, 1, 2, good, mat, "main_hoop"))
        solver.add_element(Element(2, 3, 4, thin, mat, "main_hoop"))
        c = FSAERegulationChecker(solver, FrameSemanticMap())
        results = c.check_tube_specs()
        t_results = [r for r in results if "肉厚" in r.description and "main_hoop" in r.description]
        assert t_results and not t_results[0].passed

    def test_unknown_group_fails_closed(self):
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        solver.add_node(Node(2, 1, 0, 0))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        solver.add_element(Element(1, 1, 2, TubeSection("round", 0.03175, 0.00211), mat, "mystery"))
        c = FSAERegulationChecker(solver, FrameSemanticMap())
        results = c.check_tube_specs()
        assert results and not results[0].passed

    def test_v2_groups_are_mapped(self):
        for name in ("side_impact", "shoulder_bar", "mh_bracing", "front_bulkhead", "rear_box"):
            assert name in DEFAULT_GROUP_SPEC_MAP


# =============================================================================
# 角度計算テスト
# =============================================================================
class TestAngleCalculations:
    """角度計算の検証（既知のジオメトリを使用）"""

    def test_vertical_vector_angle_zero(self, checker):
        """完全に鉛直なベクトルは0°"""
        # メインフープ: 節点11(z=-0.15) → 節点9(z=0.5), 同じ x,y
        angle = checker._calc_angle_from_vertical(11, 9)
        assert angle == pytest.approx(0.0, abs=0.1), f"Vertical member should be ~0°, got {angle}"

    def test_tilted_vector_angle(self):
        """45°傾斜ベクトルの角度検証"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0.0, 0.0, 0.0))
        solver.add_node(Node(2, 1.0, 0.0, 1.0))  # 45° from vertical in XZ plane
        smap = FrameSemanticMap()
        c = FSAERegulationChecker(solver, smap)
        angle = c._calc_angle_from_vertical(1, 2)
        assert angle == pytest.approx(45.0, abs=0.1)

    def test_horizontal_vector_angle_90(self):
        """水平ベクトルは90°"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0.0, 0.0, 0.0))
        solver.add_node(Node(2, 1.0, 0.0, 0.0))
        smap = FrameSemanticMap()
        c = FSAERegulationChecker(solver, smap)
        angle = c._calc_angle_from_vertical(1, 2)
        assert angle == pytest.approx(90.0, abs=0.1)

    def test_main_hoop_angle_pass(self, checker):
        """デモフレームのメインフープは鉛直 → ≤10°"""
        result = checker.check_main_hoop_angle()
        assert result.passed
        assert result.rule_id == "F.5.8.3"

    def test_front_hoop_angle_pass(self, checker):
        """デモフレームのフロントフープは鉛直 → ≤20°"""
        result = checker.check_front_hoop_angle()
        assert result.passed
        assert result.rule_id == "F.5.7.6"

    def test_main_hoop_angle_fail(self):
        """メインフープが10°超傾斜した場合にFAIL"""
        solver = BeamFEMSolver()
        # 20°傾斜したメインフープ
        tilt = math.tan(math.radians(20))
        solver.add_node(Node(11, 0.0, 0.3, -0.15))
        solver.add_node(Node(9, tilt * 0.65, 0.3, 0.5))  # 20° tilt
        solver.add_node(Node(12, 0.0, -0.3, -0.15))
        solver.add_node(Node(10, tilt * 0.65, -0.3, 0.5))
        smap = FrameSemanticMap()
        c = FSAERegulationChecker(solver, smap)
        result = c.check_main_hoop_angle()
        assert not result.passed, "20° tilt should exceed 10° limit"

    def test_brace_angle_pass(self, checker):
        """デモフレームのブレース角度 ≥ 30°"""
        result = checker.check_brace_angle()
        assert result.passed
        assert result.rule_id == "F.5.9.5"


# =============================================================================
# 形状チェック
# =============================================================================
class TestGeometryChecks:
    """形状チェック (幅、高さ、寸法) の検証"""

    def test_brace_attachment_pass(self, checker):
        """デモフレーム: ブレースはメインフープ頂部に直接接続 → 0mm drop"""
        result = checker.check_brace_attachment_height()
        assert result.passed
        assert "0.0 mm" in result.value

    def test_main_hoop_width_pass(self, checker):
        """デモフレーム: 内寸幅 ~568mm ≥ 380mm"""
        result = checker.check_main_hoop_width()
        assert result.passed
        assert result.rule_id == "F.5.8.4"

    def test_main_hoop_width_fail(self):
        """メインフープ幅が狭い場合にFAIL"""
        solver = BeamFEMSolver()
        # Y=±0.1m → 外寸200mm → 内寸 ~168mm < 380mm
        solver.add_node(Node(11, 1.2, 0.1, -0.15))
        solver.add_node(Node(12, 1.2, -0.1, -0.15))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        sec = TubeSection("round", 0.03175, 0.00165)
        solver.add_element(Element(1, 11, 12, sec, mat, "main_hoop"))
        smap = FrameSemanticMap()
        c = FSAERegulationChecker(solver, smap)
        result = c.check_main_hoop_width()
        assert not result.passed, "Narrow hoop should fail width check"

    def test_wheelbase_pass(self, checker):
        """デモフレーム: WB=2000mm ≥ 1525mm"""
        result = checker.check_wheelbase()
        assert result.passed
        assert "2000" in result.value

    def test_wheelbase_fail(self):
        """短いホイールベースはFAIL"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        smap = FrameSemanticMap(front_axle_x=0.0, rear_axle_x=1.0)  # 1000mm
        c = FSAERegulationChecker(solver, smap)
        result = c.check_wheelbase()
        assert not result.passed, "1000mm wheelbase should fail"

    def test_track_ratio_pass(self, checker):
        """デモフレーム: 1150/1200 = 95.8% ≥ 75%"""
        result = checker.check_track_ratio()
        assert result.passed

    def test_track_ratio_fail(self):
        """極端なトレッド差はFAIL"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        smap = FrameSemanticMap(track_front=1.5, track_rear=1.0)  # 66.7%
        c = FSAERegulationChecker(solver, smap)
        result = c.check_track_ratio()
        assert not result.passed, "66.7% ratio should fail"

    def test_side_impact_height_pass(self, checker):
        """デモフレーム: 310mm → 290-345mm 範囲内"""
        result = checker.check_side_impact_height()
        assert result.passed

    def test_side_impact_height_fail_low(self):
        """高さ不足はFAIL"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        smap = FrameSemanticMap(side_impact_height=0.200)  # 200mm < 290mm
        c = FSAERegulationChecker(solver, smap)
        result = c.check_side_impact_height()
        assert not result.passed

    def test_side_impact_height_fail_high(self):
        """高すぎてもFAIL"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        smap = FrameSemanticMap(side_impact_height=0.400)  # 400mm > 345mm
        c = FSAERegulationChecker(solver, smap)
        result = c.check_side_impact_height()
        assert not result.passed

    def test_side_impact_height_uses_actual_nodes(self):
        """side_impact group がある場合は実節点Zを使う"""
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0.0))
        solver.add_node(Node(2, 1, 0, 0.0))
        solver.add_node(Node(3, 1, 0, 0.400))
        mat = TubeMaterial("Steel", 205e9, 80e9, 7850, 460e6)
        sec = TubeSection("round", 0.03175, 0.00211)
        solver.add_element(Element(1, 1, 2, sec, mat, "side_impact"))
        solver.add_element(Element(2, 2, 3, sec, mat, "side_impact"))
        c = FSAERegulationChecker(solver, FrameSemanticMap())
        result = c.check_side_impact_height()
        assert not result.passed
        assert "400.0" in result.value

    def test_missing_required_group_fails(self):
        solver, _, _ = create_demo_fsae_frame()
        solver.elements = [e for e in solver.elements if e.group != "main_hoop"]
        c = FSAERegulationChecker(solver, FrameSemanticMap())
        assert not c.check_required_groups().passed


# =============================================================================
# インパクトアッテネータ
# =============================================================================
class TestImpactAttenuator:
    def test_ia_dimensions_pass(self, checker):
        result = checker.check_impact_attenuator_dimensions()
        assert result.passed
        assert result.rule_id == "F.8.4.2"

    def test_ia_dimensions_fail_short(self):
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        smap = FrameSemanticMap(ia_length=0.100, ia_height=0.150, ia_width=0.250)
        c = FSAERegulationChecker(solver, smap)
        result = c.check_impact_attenuator_dimensions()
        assert not result.passed

    def test_ia_energy_pass(self, checker):
        result = checker.check_impact_attenuator_energy()
        assert result.passed
        assert result.rule_id == "F.8.8.2"

    def test_ia_energy_fail_small_volume(self):
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        # 非常に小さいIA → エネルギー吸収不足
        smap = FrameSemanticMap(ia_length=0.050, ia_height=0.050, ia_width=0.050)
        c = FSAERegulationChecker(solver, smap)
        result = c.check_impact_attenuator_energy()
        assert not result.passed


# =============================================================================
# 材料チェック
# =============================================================================
class TestMaterialChecks:
    def test_main_hoop_steel_pass(self, checker):
        result = checker.check_main_hoop_material()
        assert result.passed
        assert "4130" in result.value

    def test_main_hoop_aluminum_fail(self):
        solver = BeamFEMSolver()
        solver.add_node(Node(1, 0, 0, 0))
        solver.add_node(Node(2, 1, 0, 0))
        al_mat = TubeMaterial("Aluminum 6061-T6", 70e9, 26e9, 2700, 276e6)
        sec = TubeSection("round", 0.03175, 0.00165)
        solver.add_element(Element(1, 1, 2, sec, al_mat, "main_hoop"))
        smap = FrameSemanticMap()
        c = FSAERegulationChecker(solver, smap)
        result = c.check_main_hoop_material()
        assert not result.passed, "Aluminum main hoop should fail"

    def test_bracing_material_pass(self, checker):
        """デモフレームのブレースは鉄鋼だが断面積がSize B未満 → FAIL"""
        result = checker.check_bracing_material()
        # デモフレームの sec_brace (25.4mm × 1.20mm) は A≈91mm² < Size B の 114mm²
        assert not result.passed, "Demo brace section is below Size B area requirement"


# =============================================================================
# check_all 統合テスト
# =============================================================================
class TestCheckAll:
    def test_check_all_returns_list(self, checker):
        results = checker.check_all()
        assert isinstance(results, list)
        assert len(results) > 10  # 管材仕様だけで多数チェック

    def test_check_all_result_types(self, checker):
        results = checker.check_all()
        for r in results:
            assert isinstance(r, RegulationCheckResult)
            assert r.rule_id
            assert r.category
            assert r.severity in ("critical", "warning", "info")

    def test_check_all_categories(self, checker):
        results = checker.check_all()
        categories = {r.category for r in results}
        assert "frame" in categories
        assert "safety" in categories

    def test_demo_frame_compliance(self, checker):
        """デモフレームの既知の適合・不適合をチェック"""
        results = checker.check_all()
        result_map = {}
        for r in results:
            key = f"{r.rule_id}:{r.description}"
            result_map[key] = r
        # メインフープ角度は合格（垂直メンバー）
        angle_results = [r for r in results if r.rule_id == "F.5.8.3"]
        assert len(angle_results) == 1
        assert angle_results[0].passed


# =============================================================================
# レポート生成
# =============================================================================
class TestReportGeneration:
    def test_generate_report_string(self, checker):
        results = checker.check_all()
        report = checker.generate_report(results)
        assert isinstance(report, str)
        assert "FSAE 2026" in report
        assert "=" * 70 in report

    def test_generate_report_contains_results(self, checker):
        results = checker.check_all()
        report = checker.generate_report(results)
        # ルールIDが含まれること
        assert "F.5.8.3" in report
        assert "F.5.7.6" in report
        assert "V.1.2" in report

    def test_generate_report_dict(self, checker):
        results = checker.check_all()
        report_dict = checker.generate_report_dict(results)
        assert isinstance(report_dict, dict)
        assert "overall" in report_dict
        assert "checks" in report_dict
        assert "total" in report_dict
        assert "passed" in report_dict
        assert "failed" in report_dict
        assert isinstance(report_dict["checks"], list)
        assert report_dict["total"] == len(results)
        assert report_dict["passed"] + report_dict["failed"] == report_dict["total"]

    def test_report_dict_check_format(self, checker):
        results = checker.check_all()
        report_dict = checker.generate_report_dict(results)
        for check in report_dict["checks"]:
            assert "id" in check
            assert "category" in check
            assert "rule" in check
            assert "passed" in check
            assert "value" in check
            assert "required" in check
            assert "severity" in check


# =============================================================================
# evaluate_with_compliance 統合テスト
# =============================================================================
class TestEvaluateWithCompliance:
    def test_evaluate_with_compliance(self):
        """FSAEFrameOptimizer.evaluate_with_compliance() の動作確認"""
        from fsae_frame_optimizer import (
            DesignVariable,
            FSAEFrameOptimizer,
            create_demo_fsae_frame,
        )

        solver, front_nodes, rear_nodes = create_demo_fsae_frame()
        design_vars = [
            DesignVariable("upper_rail", "round", 0.020, 0.035, 0.0012, 0.0025),
        ]
        optimizer = FSAEFrameOptimizer(solver, front_nodes, rear_nodes, design_vars)
        optimizer.semantic_map = FrameSemanticMap()

        x = np.array([0.0254, 0.00165])
        result = optimizer.evaluate_with_compliance(x)

        assert "compliance" in result
        assert "compliance_pass" in result
        assert isinstance(result["compliance"], list)
        assert isinstance(result["compliance_pass"], bool)

    def test_evaluate_without_semantic_map(self):
        """semantic_map未設定時はコンプライアンスチェックをスキップ"""
        from fsae_frame_optimizer import (
            DesignVariable,
            FSAEFrameOptimizer,
            create_demo_fsae_frame,
        )

        solver, front_nodes, rear_nodes = create_demo_fsae_frame()
        design_vars = [
            DesignVariable("upper_rail", "round", 0.020, 0.035, 0.0012, 0.0025),
        ]
        optimizer = FSAEFrameOptimizer(solver, front_nodes, rear_nodes, design_vars)

        x = np.array([0.0254, 0.00165])
        result = optimizer.evaluate_with_compliance(x)

        assert result["compliance"] == []
        assert result["compliance_pass"] is True

    def test_objective_penalizes_noncompliance_in_loop(self):
        """objective() が最適化ループ中に不適合へ自動ペナルティを与えることを確認"""
        from fsae_frame_optimizer import (
            DesignVariable,
            FSAEFrameOptimizer,
            create_demo_fsae_frame_with_map,
        )

        solver, front_nodes, rear_nodes, smap = create_demo_fsae_frame_with_map()
        design_vars = [
            DesignVariable("diagonal", "round", 0.0254, 0.035, 0.0012, 0.0025),
            DesignVariable("side_brace", "round", 0.0254, 0.035, 0.0012, 0.0025),
        ]
        optimizer = FSAEFrameOptimizer(
            solver, front_nodes, rear_nodes, design_vars, semantic_map=smap
        )
        optimizer.w_mass = 0.0
        optimizer.w_stiff = 0.0
        optimizer.K_min = None

        # 薄いブレース → Size B不適合
        x_fail = np.array([0.0254, 0.0012, 0.0254, 0.0012])
        # 太いブレース → Size B適合
        x_pass = np.array([0.03175, 0.00165, 0.03175, 0.00165])

        obj_fail = optimizer.objective(x_fail)
        obj_pass = optimizer.objective(x_pass)

        assert obj_pass < obj_fail

    def test_objective_is_independent_of_evaluation_order(self):
        """正規化基準が探索順・seedに依存しない"""
        from fsae_frame_optimizer import (
            DesignVariable,
            FSAEFrameOptimizer,
            create_demo_fsae_frame,
        )

        solver, front_nodes, rear_nodes = create_demo_fsae_frame()
        dvs = [DesignVariable("diagonal", "round", 0.0254, 0.035, 0.0012, 0.0025)]
        x1 = np.array([0.028, 0.0014])
        x2 = np.array([0.033, 0.0020])

        opt_a = FSAEFrameOptimizer(solver, front_nodes, rear_nodes, dvs)
        a1 = opt_a.objective(x1)
        a2 = opt_a.objective(x2)
        opt_b = FSAEFrameOptimizer(solver, front_nodes, rear_nodes, dvs)
        b2 = opt_b.objective(x2)
        b1 = opt_b.objective(x1)
        assert a1 == pytest.approx(b1)
        assert a2 == pytest.approx(b2)

    def test_optimize_returns_compliance_summary(self):
        """optimize() が最終結果に適合判定を自動付与することを確認"""
        from fsae_frame_optimizer import (
            DesignVariable,
            FSAEFrameOptimizer,
            create_demo_fsae_frame_with_map,
        )

        solver, front_nodes, rear_nodes, smap = create_demo_fsae_frame_with_map()
        design_vars = [
            DesignVariable("diagonal", "round", 0.0254, 0.035, 0.0012, 0.0025),
        ]
        optimizer = FSAEFrameOptimizer(
            solver, front_nodes, rear_nodes, design_vars, semantic_map=smap
        )

        result = optimizer.optimize(method="nelder-mead", max_iter=2)

        assert "compliance" in result
        assert "compliance_pass" in result
        assert "compliance_failed" in result
        assert "selected_from_history" in result
        assert isinstance(result["compliance"], list)
        assert isinstance(result["compliance_pass"], bool)


# =============================================================================
# create_demo_fsae_frame_with_map テスト
# =============================================================================
class TestCreateDemoWithMap:
    def test_returns_four_items(self):
        from fsae_frame_optimizer import create_demo_fsae_frame_with_map

        result = create_demo_fsae_frame_with_map()
        assert len(result) == 4

    def test_semantic_map_type(self):
        from fsae_frame_optimizer import create_demo_fsae_frame_with_map

        _, _, _, smap = create_demo_fsae_frame_with_map()
        assert isinstance(smap, FrameSemanticMap)

    def test_integration_with_checker(self):
        from fsae_frame_optimizer import create_demo_fsae_frame_with_map

        solver, _, _, smap = create_demo_fsae_frame_with_map()
        checker = FSAERegulationChecker(solver, smap)
        results = checker.check_all()
        assert len(results) > 0
