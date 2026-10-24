#include <gtest/gtest.h>

#include "torsion_rigidity.hpp"

#include <Eigen/Dense>
#include <vector>

using frame_optimize::FemLoad;
using frame_optimize::compute_distributed_couple_torsional_rigidity;

TEST(FrameTorsionMetricTest, DistributedCoupleUsesTotalAppliedForce) {
    Eigen::MatrixXd nodes(4, 3);
    nodes << 0.0, -0.5, 0.0,
             1.0, -0.5, 0.0,
             0.0,  0.5, 0.0,
             1.0,  0.5, 0.0;

    Eigen::VectorXd U = Eigen::VectorXd::Zero(24);
    U(2) = -0.01;
    U(8) = -0.01;
    U(14) = 0.01;
    U(20) = 0.01;

    const std::vector<FemLoad> loads = {
        {0, 2, -500.0},
        {1, 2, -500.0},
        {2, 2, 500.0},
        {3, 2, 500.0},
    };

    const double kt = compute_distributed_couple_torsional_rigidity(nodes, U, loads);
    const double expected_twist_deg = 0.02 * 180.0 / 3.14159265358979323846;
    const double expected_kt = 1000.0 / expected_twist_deg;

    EXPECT_NEAR(kt, expected_kt, 1e-9);
}

