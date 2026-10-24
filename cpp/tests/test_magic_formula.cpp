// tests/test_magic_formula.cpp
#include <gtest/gtest.h>
#include "fsae/physics/tire/magic_formula.hpp"
#include <cmath>

using namespace fsae::physics::tire;

class MagicFormulaTest : public ::testing::Test {
protected:
    MagicFormulaParams params;

    void SetUp() override {
        // Hoosier 18x6-10 R20 相当のパラメータ
        params.Fz0 = -890.0;
        params.R0 = 0.2286;

        // Fy0 (典型的なレーシングタイヤ)
        params.pCy1 = 1.3;
        params.pDy1 = 2.0;
        params.pDy2 = -0.1;
        params.pDy3 = 0.0;
        params.pKy1 = -15.0;
        params.pKy2 = 2.0;
        params.pKy4 = 2.0;
        params.pEy1 = -0.5;
        params.pEy2 = 0.0;
        params.pEy3 = 0.0;
        params.pEy4 = 0.0;
        params.pHy1 = 0.0;
        params.pHy2 = 0.0;
        params.pVy1 = 0.0;
        params.pVy2 = 0.0;

        // Fx0
        params.pCx1 = 1.6;
        params.pDx1 = 1.5;
        params.pDx2 = -0.1;
        params.pDx3 = 0.0;
        params.pKx1 = 20.0;
        params.pKx2 = 0.0;
        params.pKx3 = 0.0;
        params.pEx1 = -0.3;
        params.pEx2 = 0.0;
        params.pEx3 = 0.0;
        params.pEx4 = 0.0;
        params.pHx1 = 0.0;
        params.pHx2 = 0.0;
        params.pVx1 = 0.0;
        params.pVx2 = 0.0;

        // Gxa
        params.rBx1 = 12.0;
        params.rBx2 = 8.0;
        params.rCx1 = 1.0;
        params.rHx1 = 0.0;
        params.rEx1 = 0.0;

        // Gyk
        params.rBy1 = 8.0;
        params.rBy2 = 5.0;
        params.rBy3 = 0.0;
        params.rCy1 = 1.0;
        params.rHy1 = 0.0;

        // Mz
        params.qBz1 = 10.0;
        params.qBz2 = 0.0;
        params.qCz1 = 1.3;
        params.qDz1 = 0.13;
        params.qDz2 = 0.0;
        params.qEz1 = -1.0;
        params.qHz1 = 0.0;
        params.qHz2 = 0.0;
        params.qBz9 = 8.0;
        params.qDz6 = -0.03;
        params.qDz8 = -0.01;
        params.qDz9 = 0.0;
    }
};

TEST_F(MagicFormulaTest, ZeroSlipProducesNearZeroForce) {
    // SR=0, SA=0, gamma=0 → Fx≈0, Fy≈0
    double Fz = -890.0;  // TTC convention
    auto forces = compute_magic_formula(params, 0.0, 0.0, Fz, 0.0);

    EXPECT_NEAR(forces.Fx, 0.0, 1.0);
    EXPECT_NEAR(forces.Fy, 0.0, 1.0);
}

TEST_F(MagicFormulaTest, PureLateralForceSCurve) {
    // SA sweep → Fy の S字カーブ
    double Fz = -890.0;

    double Fy_neg = compute_mf_Fy0(params, -0.15, Fz, 0.0);  // α = -8.6°
    double Fy_zero = compute_mf_Fy0(params, 0.0, Fz, 0.0);
    double Fy_pos = compute_mf_Fy0(params, 0.15, Fz, 0.0);   // α = +8.6°

    // S字: 正αで正Fy or 負Fy（符号規約依存だが、反対方向であること）
    EXPECT_NE(0.0, Fy_pos);
    EXPECT_NE(0.0, Fy_neg);
    // 反対符号であること
    EXPECT_LT(Fy_neg * Fy_pos, 0.0);
    // ゼロ付近
    EXPECT_NEAR(Fy_zero, 0.0, 5.0);
}

TEST_F(MagicFormulaTest, PureLateralForcePeakValue) {
    // ピーク横力が物理的に妥当な範囲 (μ*Fz 程度)
    double Fz = -890.0;
    double peak_Fy = 0.0;

    for (double alpha = -0.3; alpha <= 0.3; alpha += 0.01) {
        double Fy = std::abs(compute_mf_Fy0(params, alpha, Fz, 0.0));
        peak_Fy = std::max(peak_Fy, Fy);
    }

    // |peak_Fy| ≈ μ * |Fz| = 2.0 * 890 = 1780N (±50%の余裕)
    EXPECT_GT(peak_Fy, 500.0);
    EXPECT_LT(peak_Fy, 3000.0);
}

TEST_F(MagicFormulaTest, PureLongitudinalForcePeakValue) {
    // ピーク縦力
    double Fz = -890.0;
    double peak_Fx = 0.0;

    for (double kappa = -0.3; kappa <= 0.3; kappa += 0.01) {
        double Fx = std::abs(compute_mf_Fx0(params, kappa, Fz, 0.0));
        peak_Fx = std::max(peak_Fx, Fx);
    }

    // |peak_Fx| ≈ μ * |Fz| = 1.5 * 890 = 1335N (±50%の余裕)
    EXPECT_GT(peak_Fx, 400.0);
    EXPECT_LT(peak_Fx, 3000.0);
}

TEST_F(MagicFormulaTest, VehicleConventionMapsPositiveDriveSlipToPositiveFx) {
    double Fz = -890.0;

    auto raw_drive = compute_magic_formula(params, 0.10, 0.0, Fz, 0.0);
    auto vehicle_drive = magic_formula_to_vehicle_convention(raw_drive);

    EXPECT_LT(raw_drive.Fx, 0.0);
    EXPECT_GT(vehicle_drive.Fx, 0.0);
    EXPECT_DOUBLE_EQ(vehicle_drive.Fy, raw_drive.Fy);
    EXPECT_DOUBLE_EQ(vehicle_drive.Mz, raw_drive.Mz);
}

TEST_F(MagicFormulaTest, CombinedSlipGxaRange) {
    // Gxa ∈ [0, 1] (正規化係数)
    double Fz = -890.0;

    // κ=0 で α をスイープ
    for (double alpha = -0.3; alpha <= 0.3; alpha += 0.05) {
        double Gxa = compute_mf_Gxa(params, alpha, 0.0);
        EXPECT_GE(Gxa, 0.0);
        EXPECT_LE(Gxa, 1.01);  // 数値誤差マージン
    }

    // α=0 で Gxa=1
    double Gxa_zero = compute_mf_Gxa(params, 0.0, 0.0);
    EXPECT_NEAR(Gxa_zero, 1.0, 1e-6);

    // |α| が大きいほど Gxa は小さくなる
    double Gxa_small = compute_mf_Gxa(params, 0.05, 0.0);
    double Gxa_large = compute_mf_Gxa(params, 0.20, 0.0);
    EXPECT_GT(Gxa_small, Gxa_large);
}

TEST_F(MagicFormulaTest, CombinedSlipGykRange) {
    // Gyk ∈ [0, 1]

    // α=0.1 で κ をスイープ
    for (double kappa = -0.3; kappa <= 0.3; kappa += 0.05) {
        double Gyk = compute_mf_Gyk(params, 0.1, kappa);
        EXPECT_GE(Gyk, 0.0);
        EXPECT_LE(Gyk, 1.01);
    }

    // κ=0 で Gyk=1
    double Gyk_zero = compute_mf_Gyk(params, 0.1, 0.0);
    EXPECT_NEAR(Gyk_zero, 1.0, 1e-6);
}

TEST_F(MagicFormulaTest, SelfAligningTorqueSmallAtZeroSlip) {
    double Fz = -890.0;

    // SA=0 で Mz は小さい（残留トルク Mzr の影響で完全にゼロにはならない）
    auto forces = compute_magic_formula(params, 0.0, 0.0, Fz, 0.0);
    EXPECT_NEAR(forces.Mz, 0.0, 15.0);  // 15Nm 以内（Mzr残留分を許容）
}

TEST_F(MagicFormulaTest, LoadDependence) {
    // 荷重増加 → 力の絶対値増加
    double alpha = 0.1;  // ~5.7°

    double Fy_low = std::abs(compute_mf_Fy0(params, alpha, -500.0, 0.0));
    double Fy_mid = std::abs(compute_mf_Fy0(params, alpha, -890.0, 0.0));
    double Fy_high = std::abs(compute_mf_Fy0(params, alpha, -1400.0, 0.0));

    EXPECT_LT(Fy_low, Fy_mid);
    EXPECT_LT(Fy_mid, Fy_high);
}

TEST_F(MagicFormulaTest, AllOutputsFinite) {
    // 広範囲の入力で NaN/Inf が出ないことを確認
    double Fz_values[] = {-200.0, -500.0, -890.0, -1200.0, -1500.0};
    double alpha_values[] = {-0.3, -0.1, 0.0, 0.1, 0.3};
    double kappa_values[] = {-0.3, -0.1, 0.0, 0.1, 0.3};
    double gamma_values[] = {-0.05, 0.0, 0.05};

    for (double Fz : Fz_values) {
        for (double alpha : alpha_values) {
            for (double kappa : kappa_values) {
                for (double gamma : gamma_values) {
                    auto forces = compute_magic_formula(params, kappa, alpha, Fz, gamma);
                    EXPECT_TRUE(std::isfinite(forces.Fx))
                        << "Fx not finite at kappa=" << kappa
                        << " alpha=" << alpha << " Fz=" << Fz;
                    EXPECT_TRUE(std::isfinite(forces.Fy))
                        << "Fy not finite at kappa=" << kappa
                        << " alpha=" << alpha << " Fz=" << Fz;
                    EXPECT_TRUE(std::isfinite(forces.Mz))
                        << "Mz not finite at kappa=" << kappa
                        << " alpha=" << alpha << " Fz=" << Fz;
                }
            }
        }
    }
}

TEST_F(MagicFormulaTest, TopLevelCombinedReduction) {
    // 複合スリップで純スリップより力が減少することを確認
    double Fz = -890.0;
    double alpha = 0.1;
    double kappa = 0.1;

    double Fx_pure = std::abs(compute_mf_Fx0(params, kappa, Fz, 0.0));
    double Fy_pure = std::abs(compute_mf_Fy0(params, alpha, Fz, 0.0));

    auto combined = compute_magic_formula(params, kappa, alpha, Fz, 0.0);

    EXPECT_LT(std::abs(combined.Fx), Fx_pure * 1.01);
    EXPECT_LT(std::abs(combined.Fy), Fy_pure * 1.01);
}
