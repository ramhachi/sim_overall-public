#pragma once
/// @file velocity_profile.hpp
/// @brief 速度プロファイル生成（摩擦円制約対応版）
///
/// 総合ガイド §15.3, §17 に基づく実装
/// トラック上の各位置での目標車速を計算

#include "../track/track_model.hpp"
#include "../physics/aero/aero_map_4d.hpp"
#include "../driver/steering_actuator_controller.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <optional>
#include <limits>

namespace fsae::laptime {

/// 速度プロファイルポイント
struct VelocityPoint {
    double s;        // 弧長 [m]
    double v;        // 速度 [m/s]
    double ax;       // 縦加速度 [m/s²]
    double ay;       // 横加速度 [m/s²]
    double kappa;    // 曲率 [1/m] (計算用)
    double steering_speed_cap = std::numeric_limits<double>::infinity(); // 操舵到達性由来の速度上限 [m/s]
};

/// 車両性能パラメータ
struct VehiclePerformance {
    // 従来の固定制限
    double max_lat_accel = 1.5 * 9.81;
    bool enforce_max_lat_accel = false;
    double max_lon_accel = 1.0 * 9.81;
    double max_lon_decel = 1.8 * 9.81;
    double max_speed = 30.0;
    double min_speed = 5.0;
    double safety_factor = 0.90;    // 摩擦円の90%を使用（PIDの追従余裕）

    // 物理パラメータ
    bool use_physics_model = true;
    double mass = 250.0;
    double mu_lat = 1.6;            // タイヤ横摩擦係数
    double mu_lon = 1.5;            // タイヤ縦摩擦係数
    double cl_a = 2.8;              // 空力ダウンフォース [m²] (CL * A)
    double cd_a = 0.95;             // 空力ドラッグ [m²] (CD * A)
    double rho = 1.225;             // 空気密度
    bool use_load_sensitive_lateral_limit = false;
    double tire_nominal_load = 1500.0;       // [N]
    double tire_load_sensitivity = -0.0001;  // [1/N]
    double front_static_load_fraction = 0.50;
    double front_aero_balance = 0.50;
    double track_front = 1.20;               // [m]
    double track_rear = 1.20;                // [m]
    double cg_height = 0.28;                 // [m]
    double front_lateral_transfer_fraction = 0.50;
    bool use_aero_map = false;
    physics::aero::AeroMap4D<double> aero_map;
    double aero_nominal_front_ride_height_mm = 40.0;
    double aero_nominal_rear_ride_height_mm = 55.0;
    double aero_nominal_roll_rad = 0.0;
    double aero_nominal_yaw_rad = 0.0;
    bool aero_drs_active = false;

    // 発進は ax_max(v) の低速値（GGV ではトラクション制限込み）で表す。旧 warmup ランプは削除（Issue #24）。

    // Steering actuator feasibility limits. These are disabled by default so
    // angle-steering and legacy callers keep the original friction-only profile.
    bool use_steering_actuator_limits = false;
    double wheelbase = 1.55;                         // [m]
    double max_tire_steer_angle = 0.55;              // [rad]
    double max_tire_steer_rate = 3.0;                // [rad/s]
    double steering_preview_distance = 12.0;         // [m], used when preview_time <= 0
    double steering_preview_time = 0.0;               // [s]
    double steering_min_preview_distance = 3.0;       // [m]
    double steering_max_preview_distance = 25.0;      // [m]
    double steering_deadband = 0.02;                 // [rad]
    double steering_speed_safety_factor = 0.85;      // [-]
    double steering_inertia = 0.02;                  // [kg*m^2]
    double steering_friction_torque = 0.3;           // [Nm]
    double steering_driver_torque_max = 25.0;        // [Nm]
    double steering_nominal_authority = (25.0 - 0.3) / 0.02; // [Nm/(kg*m^2)]
    double steering_min_authority_scale = 0.75;      // [-]
    double steering_response_omega_n = 30.0;         // [rad/s]
    double steering_response_zeta = 0.70;            // [-]
    double steering_understeer_gradient = 0.002;     // [rad/(m/s^2)]

    // Optional planned-line curvature. When set, velocity limits and steering
    // actuator feasibility are computed from this curvature rather than the
    // centerline curvature while preserving centerline s as the profile index.
    std::vector<std::pair<double, double>> curvature_override;
};

/// 速度プロファイル生成器
class VelocityProfileGenerator {
public:
    explicit VelocityProfileGenerator(const VehiclePerformance& perf)
        : perf_(perf) {}

    /// @param v_start 非周回コースの始点速度（前進 pass の境界条件）。周回コースでは無視（周期解）
    /// @param v_end   非周回コースの終端速度上限（後退 pass の境界条件）。未指定は自由（減速しない）
    std::vector<VelocityPoint> generate(const track::CurvatureBasedTrack& track, double ds = 1.0,
                                        std::optional<double> v_start = std::nullopt,
                                        std::optional<double> v_end = std::nullopt) {
        // 1. 旋回限界速度の計算（純粋な横方向限界）
        auto profile = compute_corner_limited_speed(track, ds);
        if (profile.empty()) return profile;

        // 2. 操舵アクチュエータの状態付き到達性を反映
        apply_steering_state_speed_caps(profile);

        // 3/4. 前方積分（加速限界）→ 後方積分（減速限界）。境界条件は pass の中で与える（Issue #29）
        if (track.is_closed_loop() && profile.size() > 2) {
            // 周回: 2 周分を前進→後退し、各点で 1 周目と 2 周目の像の min を取る。
            // 1 周目は始点側の前進境界が、2 周目は終点側の後退境界が楽観的なので、min で周期解になる。
            const std::size_t n = profile.size();
            const double lap = profile.back().s;
            std::vector<VelocityPoint> two = profile;
            for (std::size_t i = 1; i < n; ++i) {
                two.push_back(profile[i]);
                two.back().s += lap;
            }
            two = backward_integration(forward_integration(std::move(two)));
            for (std::size_t i = 0; i < n; ++i) {
                profile[i].v = std::min(two[i].v, two[i + n - 1].v);
            }
        } else {
            if (v_start) profile.front().v = std::min(*v_start, profile.front().v);
            if (v_end) profile.back().v = std::min(*v_end, profile.back().v);
            profile = backward_integration(forward_integration(std::move(profile)));
        }

        // 5. 加速度の最終更新
        update_accelerations(profile);

        return profile;
    }

private:
    /// 旋回限界速度を計算
    std::vector<VelocityPoint> compute_corner_limited_speed(
        const track::CurvatureBasedTrack& track, double ds)
    {
        std::vector<VelocityPoint> profile;
        double s = 0.0;
        double s_max = track.length();

        while (s <= s_max) {
            VelocityPoint vp;
            vp.s = s;
            vp.kappa = curvature_at(track, s);
            double kappa_abs = std::abs(vp.kappa);

            if (kappa_abs > 1e-6) {
                double v_limit = perf_.max_speed;
                if (perf_.use_aero_map || perf_.use_load_sensitive_lateral_limit) {
                    // 空力荷重が速度依存になるため簡易固定点反復で解く
                    for (int iter = 0; iter < 6; ++iter) {
                        double ay_max = get_max_lat_accel(v_limit);
                        v_limit = std::sqrt(std::max(ay_max / kappa_abs, 0.0));
                    }
                } else {
                    // 空力を考慮した旋回限界: v^2 * kappa <= mu * (g + 0.5/m * rho * cl_a * v^2)
                    double aero_const = 0.5 * perf_.rho * perf_.cl_a / perf_.mass;
                    double denom = kappa_abs / perf_.mu_lat - aero_const;

                    double v_sq;
                    if (denom > 1e-6) {
                        v_sq = 9.81 / denom;
                    } else {
                        v_sq = perf_.max_speed * perf_.max_speed;
                    }
                    v_limit = std::sqrt(std::max(0.0, v_sq));
                }

                const double fixed_accel_limit = get_max_lat_accel(v_limit);
                v_limit = std::min(v_limit, std::sqrt(std::max(fixed_accel_limit / kappa_abs, 0.0)));
                vp.v = std::min(v_limit * perf_.safety_factor, perf_.max_speed);
            } else {
                vp.v = perf_.max_speed;
            }

            const double steering_cap =
                compute_steering_actuator_speed_cap(track, s, vp.kappa, vp.v);
            vp.steering_speed_cap = steering_cap;
            if (std::isfinite(steering_cap)) {
                vp.v = std::min(vp.v, steering_cap);
            }

            if (std::abs(vp.kappa) > 1e-6) {
                vp.v = std::max(vp.v, perf_.min_speed);  // 下限はコーナーのみ（直線の静止発進を妨げない）
            }
            profile.push_back(vp);
            s += ds;
        }
        return profile;
    }

    double required_tire_angle(double kappa, double speed) const {
        const double target_ay = std::max(speed, 0.0) * std::max(speed, 0.0) * kappa;
        return std::clamp(
            -(perf_.wheelbase * kappa + perf_.steering_understeer_gradient * target_ay),
            -perf_.max_tire_steer_angle,
            perf_.max_tire_steer_angle);
    }

    double effective_tire_steer_rate() const {
        const double authority_scale = compute_steering_authority_scale();
        return perf_.max_tire_steer_rate * perf_.steering_speed_safety_factor * authority_scale;
    }

    void apply_steering_state_speed_caps(std::vector<VelocityPoint>& profile) const {
        if (!perf_.use_steering_actuator_limits ||
            profile.size() < 2 ||
            perf_.max_tire_steer_rate <= 1e-6) {
            return;
        }

        const double effective_rate = effective_tire_steer_rate();
        if (effective_rate <= 1e-6) {
            for (auto& point : profile) {
                point.steering_speed_cap = std::min(point.steering_speed_cap, perf_.min_speed);
                point.v = std::min(point.v, perf_.min_speed);
            }
            return;
        }

        const auto prediction_config = make_steering_prediction_config(effective_rate);
        driver::SteeringActuatorPredictionState steering_state{
            required_tire_angle(profile.front().kappa, profile.front().v),
            0.0,
        };
        for (size_t i = 1; i < profile.size(); ++i) {
            auto& prev = profile[i - 1];
            auto& curr = profile[i];
            const double ds = std::max(curr.s - prev.s, 1e-6);
            const double target_tire_angle = required_tire_angle(curr.kappa, curr.v);
            const double gap = std::max(
                0.0,
                std::abs(target_tire_angle - steering_state.tire_angle) - perf_.steering_deadband);
            if (gap > 1e-9) {
                const double reach_time = driver::estimate_steering_actuator_reach_time(
                    prediction_config,
                    steering_state,
                    target_tire_angle,
                    perf_.steering_deadband,
                    1.0);
                const double local_cap = std::isfinite(reach_time) && reach_time > 1e-9
                    ? ds / reach_time
                    : perf_.min_speed;
                curr.steering_speed_cap = std::min(curr.steering_speed_cap, local_cap);
                curr.v = std::min(curr.v, curr.steering_speed_cap);
            }

            const double v_for_dt = std::max(std::min(prev.v, curr.v), perf_.min_speed);
            const double dt = ds / v_for_dt;
            steering_state = driver::predict_steering_actuator_response(
                prediction_config,
                steering_state,
                target_tire_angle,
                dt);
        }
    }

    /// 前方積分（加速限界 + 摩擦円）
    std::vector<VelocityPoint> forward_integration(std::vector<VelocityPoint> profile) {
        for (size_t i = 1; i < profile.size(); ++i) {
            auto& prev = profile[i - 1];
            auto& curr = profile[i];
            double ds = curr.s - prev.s;

            // 現在の速度での横加速度
            double ay = prev.v * prev.v * prev.kappa;
            double ay_max = get_max_lat_accel(prev.v);

            // 摩擦円制約に基づく利用可能な縦加速度
            double lat_ratio = std::clamp(std::abs(ay) / ay_max, 0.0, 1.0);
            double ax_avail = perf_.max_lon_accel * std::sqrt(1.0 - lat_ratio * lat_ratio);

            // 空力ドラッグによる損失
            double drag_accel = get_drag_accel(prev.v);
            double ax_max = std::max(0.0, ax_avail - drag_accel);

            // 積分
            double v_accel_sq = prev.v * prev.v + 2.0 * ax_max * ds;
            double v_accel = std::sqrt(std::max(0.0, v_accel_sq));

            curr.v = std::min(curr.v, v_accel);
        }
        return profile;
    }

    /// 後方積分（減速限界 + 摩擦円）
    std::vector<VelocityPoint> backward_integration(std::vector<VelocityPoint> profile) {
        for (int i = static_cast<int>(profile.size()) - 2; i >= 0; --i) {
            auto& curr = profile[i];
            auto& next = profile[i + 1];
            double ds = next.s - curr.s;

            // 次の地点での横加速度
            double ay = next.v * next.v * next.kappa;
            double ay_max = get_max_lat_accel(next.v);

            // 摩擦円制約に基づく利用可能な減速度
            double lat_ratio = std::clamp(std::abs(ay) / ay_max, 0.0, 1.0);
            double ax_avail = perf_.max_lon_decel * std::sqrt(1.0 - lat_ratio * lat_ratio);

            // 空力ドラッグによる補助（減速を助ける）
            double drag_accel = get_drag_accel(next.v);
            double decel_max = ax_avail + drag_accel;

            // 積分
            double v_brake_sq = next.v * next.v + 2.0 * decel_max * ds;
            double v_brake = std::sqrt(std::max(0.0, v_brake_sq));

            curr.v = std::min(curr.v, v_brake);
        }
        return profile;
    }

    /// 加速度データの更新
    void update_accelerations(std::vector<VelocityPoint>& profile) {
        for (size_t i = 0; i < profile.size(); ++i) {
            auto& vp = profile[i];
            vp.ay = vp.v * vp.v * vp.kappa;
            
            if (i < profile.size() - 1) {
                double ds = profile[i+1].s - vp.s;
                if (ds > 1e-6) {
                    vp.ax = (profile[i+1].v * profile[i+1].v - vp.v * vp.v) / (2.0 * ds);
                }
            } else if (i > 0) {
                vp.ax = profile[i-1].ax;
            }
        }
    }

    /// 速度に応じた最大横加速度を取得（空力考慮）
    double get_max_lat_accel(double v) const {
        const double downforce = get_downforce(v);
        const double fz_total = perf_.mass * 9.81 + downforce;
        double ay_max = (perf_.mu_lat * fz_total) / perf_.mass;
        if (perf_.use_load_sensitive_lateral_limit) {
            ay_max = get_load_sensitive_lat_accel(downforce, ay_max);
        }
        if (perf_.enforce_max_lat_accel && perf_.max_lat_accel > 0.0) {
            ay_max = std::min(ay_max, perf_.max_lat_accel);
        }
        return ay_max;
    }

    double get_downforce(double v) const {
        double downforce = 0.5 * perf_.rho * perf_.cl_a * v * v;
        if (perf_.use_aero_map) {
            auto aero = perf_.aero_map.compute_forces(
                v,
                perf_.aero_nominal_front_ride_height_mm,
                perf_.aero_nominal_rear_ride_height_mm,
                perf_.aero_nominal_roll_rad,
                perf_.aero_nominal_yaw_rad,
                perf_.aero_drs_active
            );
            downforce = std::max(-aero.Fz, 0.0);
        }
        return downforce;
    }

    double get_load_sensitive_lat_accel(double downforce, double initial_ay) const {
        const double static_weight = perf_.mass * 9.81;
        const double front_static_fraction =
            std::clamp(perf_.front_static_load_fraction, 0.05, 0.95);
        const double front_aero_balance =
            std::clamp(perf_.front_aero_balance, 0.05, 0.95);
        const double front_total =
            static_weight * front_static_fraction + downforce * front_aero_balance;
        const double rear_total = static_weight + downforce - front_total;
        const double transfer_fraction =
            std::clamp(perf_.front_lateral_transfer_fraction, 0.05, 0.95);

        double ay = initial_ay;
        for (int iter = 0; iter < 6; ++iter) {
            const double roll_moment = perf_.mass * std::abs(ay) * std::max(perf_.cg_height, 0.0);
            const double front_transfer = std::min(
                roll_moment * transfer_fraction / std::max(perf_.track_front, 0.1),
                std::max(front_total * 0.5 - 1.0, 0.0));
            const double rear_transfer = std::min(
                roll_moment * (1.0 - transfer_fraction) / std::max(perf_.track_rear, 0.1),
                std::max(rear_total * 0.5 - 1.0, 0.0));

            const double fy_capacity =
                tire_lateral_capacity(front_total * 0.5 + front_transfer) +
                tire_lateral_capacity(front_total * 0.5 - front_transfer) +
                tire_lateral_capacity(rear_total * 0.5 + rear_transfer) +
                tire_lateral_capacity(rear_total * 0.5 - rear_transfer);
            ay = fy_capacity / std::max(perf_.mass, 1e-6);
        }
        return ay;
    }

    double tire_lateral_capacity(double fz) const {
        fz = std::max(fz, 1.0);
        const double mu = std::max(
            perf_.mu_lat + perf_.tire_load_sensitivity * (fz - perf_.tire_nominal_load),
            0.1);
        return mu * fz;
    }

    double get_drag_accel(double v) const {
        if (perf_.use_aero_map) {
            auto aero = perf_.aero_map.compute_forces(
                v,
                perf_.aero_nominal_front_ride_height_mm,
                perf_.aero_nominal_rear_ride_height_mm,
                perf_.aero_nominal_roll_rad,
                perf_.aero_nominal_yaw_rad,
                perf_.aero_drs_active
            );
            return std::max(-aero.Fx, 0.0) / perf_.mass;
        }
        return (0.5 * perf_.rho * perf_.cd_a * v * v) / perf_.mass;
    }

    double wrap_track_s(const track::CurvatureBasedTrack& track, double s) const {
        const double length = track.length();
        if (!track.is_closed_loop() || length <= 1e-6) {
            return std::clamp(s, 0.0, length);
        }
        while (s < 0.0) {
            s += length;
        }
        while (s > length) {
            s -= length;
        }
        return s;
    }

    double compute_steering_authority_scale() const {
        const double available_torque = std::max(
            perf_.steering_driver_torque_max - std::max(perf_.steering_friction_torque, 0.0), 1e-3);
        const double authority = available_torque / std::max(perf_.steering_inertia, 1e-6);
        return std::clamp(
            authority / std::max(perf_.steering_nominal_authority, 1e-6),
            perf_.steering_min_authority_scale,
            1.0);
    }

    driver::SteeringActuatorConfig make_steering_prediction_config(
        double effective_tire_steer_rate) const
    {
        driver::SteeringActuatorConfig config;
        config.max_tire_angle = perf_.max_tire_steer_angle;
        config.max_tire_steer_rate = effective_tire_steer_rate;
        config.inner_loop_omega_n = perf_.steering_response_omega_n;
        config.inner_loop_zeta = perf_.steering_response_zeta;
        return config;
    }

    double compute_steering_preview_distance(double reference_speed) const {
        if (perf_.steering_preview_time <= 1e-9) {
            return std::max(perf_.steering_preview_distance, 0.0);
        }
        const double min_distance = std::max(perf_.steering_min_preview_distance, 0.0);
        const double max_distance =
            std::max(perf_.steering_max_preview_distance, min_distance);
        return std::clamp(
            perf_.steering_preview_time * std::max(reference_speed, 0.0),
            min_distance,
            max_distance);
    }

    double compute_steering_actuator_speed_cap(
        const track::CurvatureBasedTrack& track,
        double s,
        double current_kappa,
        double reference_speed) const
    {
        const double preview_distance =
            compute_steering_preview_distance(reference_speed);
        if (!perf_.use_steering_actuator_limits ||
            preview_distance <= 1e-6 ||
            perf_.max_tire_steer_rate <= 1e-6 ||
            perf_.wheelbase <= 1e-6) {
            return std::numeric_limits<double>::infinity();
        }

        const double authority_scale = compute_steering_authority_scale();
        const double effective_tire_rate =
            perf_.max_tire_steer_rate * perf_.steering_speed_safety_factor * authority_scale;
        if (effective_tire_rate <= 1e-6) {
            return perf_.min_speed;
        }

        const double current_tire_angle = required_tire_angle(current_kappa, reference_speed);
        const auto prediction_config = make_steering_prediction_config(effective_tire_rate);
        double cap = std::numeric_limits<double>::infinity();
        constexpr double kStep = 1.0; // [m]

        for (double ds = kStep; ds <= preview_distance; ds += kStep) {
            const double sample_s = wrap_track_s(track, s + ds);
            const double required =
                required_tire_angle(curvature_at(track, sample_s), reference_speed);
            const double angle_gap = std::max(
                0.0,
                std::abs(required - current_tire_angle) - perf_.steering_deadband);
            if (angle_gap <= 1e-9) {
                continue;
            }

            const double reach_time = driver::estimate_steering_actuator_reach_time(
                prediction_config,
                driver::SteeringActuatorPredictionState{current_tire_angle, 0.0},
                required,
                perf_.steering_deadband,
                1.0);
            const double local_cap = std::isfinite(reach_time) && reach_time > 1e-9
                ? ds / reach_time
                : perf_.min_speed;
            cap = std::min(cap, local_cap);
        }

        if (!std::isfinite(cap)) {
            return cap;
        }
        return std::clamp(cap, perf_.min_speed, perf_.max_speed);
    }

    VehiclePerformance perf_;

    double curvature_at(const track::CurvatureBasedTrack& track, double s) const {
        if (perf_.curvature_override.empty()) {
            return track.get_curvature(s);
        }
        const auto& data = perf_.curvature_override;
        if (s <= data.front().first) {
            return data.front().second;
        }
        if (s >= data.back().first) {
            return data.back().second;
        }
        for (std::size_t i = 0; i + 1 < data.size(); ++i) {
            const auto& p0 = data[i];
            const auto& p1 = data[i + 1];
            if (s >= p0.first && s < p1.first) {
                const double ds = std::max(p1.first - p0.first, 1e-9);
                const double alpha = (s - p0.first) / ds;
                return p0.second + alpha * (p1.second - p0.second);
            }
        }
        return data.back().second;
    }
};

/// 簡易ラップタイム計算
inline double compute_lap_time(const std::vector<VelocityPoint>& velocity_profile) {
    double lap_time = 0.0;
    for (size_t i = 1; i < velocity_profile.size(); ++i) {
        const auto& p0 = velocity_profile[i - 1];
        const auto& p1 = velocity_profile[i];
        double ds = p1.s - p0.s;
        double v_avg = (p0.v + p1.v) / 2.0;
        if (v_avg > 1e-6) lap_time += ds / v_avg;
    }
    return lap_time;
}

/// g-g diagramのサンプルポイントを生成
/// @param perf 車両性能パラメータ
/// @param n_points サンプル数
/// @return (ax, ay)のペア
inline std::vector<std::pair<double, double>> generate_gg_diagram(
    const VehiclePerformance& perf, int n_points = 50)
{
    std::vector<std::pair<double, double>> gg_points;

    // 円形の加速度限界（簡略版）
    for (int i = 0; i < n_points; ++i) {
        double theta = 2.0 * M_PI * i / n_points;

        // 前後方向で非対称（加速 < 減速）
        double ax_max = (std::cos(theta) > 0) ? perf.max_lon_accel : -perf.max_lon_decel;
        double ay_max = perf.max_lat_accel;

        // 楕円近似
        double ax = ax_max * std::cos(theta);
        double ay = ay_max * std::sin(theta);

        gg_points.emplace_back(ax, ay);
    }

    return gg_points;
}

} // namespace fsae::laptime
