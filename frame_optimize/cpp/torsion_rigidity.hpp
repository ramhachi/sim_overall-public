#pragma once

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <vector>

namespace frame_optimize {

struct FemLoad {
    int node = 0;  // 0-indexed
    int dof = 0;   // 0-indexed
    double value = 0.0;
};

inline double compute_direct_rx_torsional_rigidity(const Eigen::VectorXd& U,
                                                   const std::vector<FemLoad>& loads) {
    constexpr double pi = 3.14159265358979323846;
    for (const auto& load : loads) {
        if (load.dof != 3) {
            continue;
        }
        const int global_dof = 6 * load.node + load.dof;
        const double angle_rad = U(global_dof);
        if (std::abs(angle_rad) > 1e-10) {
            return std::abs(load.value) / (std::abs(angle_rad) * 180.0 / pi);
        }
    }
    return 0.0;
}

inline double compute_distributed_couple_torsional_rigidity(const Eigen::MatrixXd& nodes,
                                                            const Eigen::VectorXd& U,
                                                            const std::vector<FemLoad>& loads) {
    constexpr double pi = 3.14159265358979323846;

    double pos_force = 0.0;
    double neg_force = 0.0;
    double pos_y = 0.0;
    double neg_y = 0.0;
    double pos_uz = 0.0;
    double neg_uz = 0.0;

    for (const auto& load : loads) {
        if (load.dof != 2 || std::abs(load.value) <= 0.0) {
            continue;
        }
        const double weight = std::abs(load.value);
        const double y = nodes(load.node, 1);
        const double uz = U(6 * load.node + 2);
        if (load.value > 0.0) {
            pos_force += weight;
            pos_y += weight * y;
            pos_uz += weight * uz;
        } else {
            neg_force += weight;
            neg_y += weight * y;
            neg_uz += weight * uz;
        }
    }

    if (pos_force <= 0.0 || neg_force <= 0.0) {
        return 0.0;
    }

    pos_y /= pos_force;
    neg_y /= neg_force;
    pos_uz /= pos_force;
    neg_uz /= neg_force;

    const double arm = std::abs(pos_y - neg_y);
    if (arm <= 1e-6) {
        return 0.0;
    }

    const double effective_force = std::min(pos_force, neg_force);
    const double torque = effective_force * arm;
    const double twist_rad = std::abs(pos_uz - neg_uz) / arm;
    const double twist_deg = twist_rad * 180.0 / pi;
    if (twist_deg <= 1e-12) {
        return 0.0;
    }

    return torque / twist_deg;
}

inline double compute_torsional_rigidity(const Eigen::MatrixXd& nodes,
                                         const Eigen::VectorXd& U,
                                         const std::vector<FemLoad>& loads) {
    const double direct = compute_direct_rx_torsional_rigidity(U, loads);
    if (direct > 0.0) {
        return direct;
    }
    return compute_distributed_couple_torsional_rigidity(nodes, U, loads);
}

}  // namespace frame_optimize
