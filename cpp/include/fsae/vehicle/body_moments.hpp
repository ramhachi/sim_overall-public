#pragma once
/// @file body_moments.hpp
/// @brief Rigid-body moment aggregation for the 14DOF vehicle.
///
/// SAE J670e frame: X forward, Y right, Z down. The contact-patch lateral
/// force acts below the CG, so its roll moment is negative for positive Fy:
///   Mx = (r x F)x = -r_z * Fy
/// The suspension force and roll/pitch arms are supplied by the caller so the
/// vehicle model and the regression tests use the same sign convention.

#include <array>
#include <cmath>
#include <Eigen/Dense>

namespace fsae::vehicle {

inline Eigen::Vector3d compute_contact_patch_velocity(
    const Eigen::Vector3d& body_velocity,
    const Eigen::Vector3d& body_angular_rate,
    const Eigen::Vector3d& contact_position) {
    return body_velocity + body_angular_rate.cross(contact_position);
}

/// Gyroscopic moment of one spinning wheel on its carrier (#13).
/// Forward rolling (omega > 0) spins about -Y in SAE (Z down), so the wheel's
/// angular momentum is h = -Iw*omega*y_hat with y_hat = (-sin(delta), cos(delta), 0).
/// Moment exerted on the carrier rotating with body rate Omega: M = h x Omega.
/// Right turn (r > 0): Mx < 0 (rolls the body outward). Body roll rate p > 0 on a
/// front wheel: Mz > 0 (steers right, as a leaning bike).
inline Eigen::Vector3d compute_wheel_gyro_moment(
    double Iw, double omega, double delta, const Eigen::Vector3d& body_rate) {
    const Eigen::Vector3d h = -Iw * omega * Eigen::Vector3d(-std::sin(delta), std::cos(delta), 0.0);
    return h.cross(body_rate);
}

struct BodyMomentContributions {
    double Mx = 0.0;
    double My = 0.0;
    double Mz = 0.0;
};

inline BodyMomentContributions compute_body_moment_contributions(
    const std::array<Eigen::Vector3d, 4>& wheel_pos,
    const std::array<double, 4>& Fx_wheel,
    const std::array<double, 4>& Fy_wheel,
    const std::array<double, 4>& F_susp,
    const std::array<double, 4>& roll_moment_arm,
    const std::array<double, 4>& pitch_moment_arm,
    const std::array<double, 4>& tire_Mz) {
    BodyMomentContributions out;
    for (int i = 0; i < 4; ++i) {
        // Suspension force is upward-positive while Z is down.
        out.Mx -= wheel_pos[i].y() * F_susp[i];
        out.My += wheel_pos[i].x() * F_susp[i];

        // Lateral force at the contact patch rolls the body away from the turn.
        out.Mx -= Fy_wheel[i] * roll_moment_arm[i];

        // Longitudinal force at the contact patch pitches the body.
        out.My += Fx_wheel[i] * pitch_moment_arm[i];

        // Tire free Mz is transmitted through the upright/suspension to the body.
        out.Mz += wheel_pos[i].x() * Fy_wheel[i]
                - wheel_pos[i].y() * Fx_wheel[i]
                + tire_Mz[i];
    }
    return out;
}

} // namespace fsae::vehicle
