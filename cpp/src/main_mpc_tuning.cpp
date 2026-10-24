/// @file main_mpc_tuning.cpp
/// @brief MPC重みパラメータの自動チューニング (Phase 8c)
///
/// グリッドサーチでMPC重みを最適化し、PID相当の横偏差性能を目指す

#include <fsae/driver/pid_driver.hpp>
#ifdef HAS_OSQP
#include <fsae/driver/mpc_driver.hpp>
#endif
#include <fsae/vehicle/vehicle_14dof_new.hpp>
#include <fsae/track/track_model.hpp>
#include <fsae/laptime/velocity_profile.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <algorithm>

using namespace fsae;

/// State14DOF → VehicleState 変換
vehicle::VehicleState to_vehicle_state(const vehicle::State14DOF& state14) {
    vehicle::VehicleState vs;
    vs.x = state14.x;
    vs.y = state14.y;
    vs.z = state14.z;
    vs.phi = state14.phi;
    vs.theta = state14.theta;
    vs.psi = state14.psi;
    vs.vx = state14.vx;
    vs.vy = state14.vy;
    vs.vz = state14.vz;
    vs.p = state14.p;
    vs.q = state14.q;
    vs.r = state14.r;
    vs.z_u = state14.z_u;
    vs.vz_u = state14.vz_u;
    vs.omega = state14.omega;
    vs.tire_xi = state14.tire_xi;
    vs.tire_eta = state14.tire_eta;
    vs.damper_z = state14.damper_z;
    return vs;
}

/// チューニング結果
struct TuningResult {
    // パラメータ
    double w_x, w_y, w_psi;
    double w_delta_rate;
    double Cf, Cr;  // 自転車モデルパラメータ

    // 性能指標
    double lateral_error_rms;
    double lateral_error_max;
    double steering_rate_rms;
    double lap_time;
    double avg_solve_time_ms;
    int solve_failures;
    bool converged;
};

#ifdef HAS_OSQP
/// MPCシミュレーション（簡易版）
TuningResult run_mpc_tuning(
    const track::CurvatureBasedTrack& track,
    const std::vector<laptime::VelocityPoint>& velocity_profile,
    const control::MPCConfig& config,
    double Cf, double Cr)
{
    TuningResult result;
    result.w_x = config.w_x;
    result.w_y = config.w_y;
    result.w_psi = config.w_psi;
    result.w_delta_rate = config.w_delta_rate;
    result.Cf = Cf;
    result.Cr = Cr;
    result.converged = false;

    try {
        // カスタム自転車パラメータでMPCドライバー作成
        driver::MPCPathFollowingDriver driver(track, velocity_profile, config);

        // 車両初期化
        vehicle::VehicleParams vp;
        vehicle::Vehicle14DOF_New vehicle(vp);
        vehicle.reset();
        vehicle.set_initial_velocity(8.0);

        // 初期位置（円形トラック）
        vehicle.state().x = 30.0;
        vehicle.state().y = 0.0;
        vehicle.state().z = 0.0;
        vehicle.state().psi = M_PI / 2.0;

        // ゴールテープ
        double start_x = vehicle.state().x;
        double start_y = vehicle.state().y;
        double start_heading = vehicle.state().psi;
        double finish_nx = std::cos(start_heading);
        double finish_ny = std::sin(start_heading);
        double prev_finish_dist = 0.0;
        bool crossed_away = false;

        // 統計
        double sum_lateral_sq = 0.0;
        double max_lateral = 0.0;
        double sum_steering_rate_sq = 0.0;
        int sample_count = 0;

        double prev_x = vehicle.state().x;
        double prev_y = vehicle.state().y;
        double absolute_distance = 0.0;

        track::TrackDistanceQuery track_query(track, 0.5);
        double hint_s = -1.0;

        double prev_steering = 0.0;
        double last_mpc_time = -config.dt;
        vehicle::DriverInput input;

        const double dt = 0.001;
        const double sim_time = 30.0;

        for (double t = 0.0; t < sim_time; t += dt) {
            // MPC制御
            if (t - last_mpc_time >= config.dt) {
                last_mpc_time = t;
                auto vs = to_vehicle_state(vehicle.state());
                input = driver.compute_control(vs, t);
            }

            // 車両ステップ
            vehicle.step(dt, input);

            // 距離計測
            if (t > 0.0) {
                double dx = vehicle.state().x - prev_x;
                double dy = vehicle.state().y - prev_y;
                absolute_distance += std::sqrt(dx * dx + dy * dy);
                prev_x = vehicle.state().x;
                prev_y = vehicle.state().y;
            }

            // ラップ完了判定
            double finish_dist = (vehicle.state().x - start_x) * finish_nx
                               + (vehicle.state().y - start_y) * finish_ny;

            if (!result.converged) {
                if (!crossed_away && absolute_distance > track.length() * 0.3) {
                    crossed_away = true;
                }
                if (crossed_away && absolute_distance > track.length() * 0.8) {
                    if ((prev_finish_dist < 0.0 && finish_dist >= 0.0) ||
                        (prev_finish_dist > 0.0 && finish_dist <= 0.0)) {
                        double dx_start = vehicle.state().x - start_x;
                        double dy_start = vehicle.state().y - start_y;
                        if (std::sqrt(dx_start * dx_start + dy_start * dy_start) < track.length() * 0.1) {
                            result.converged = true;
                            result.lap_time = t;
                        }
                    }
                }
            }
            prev_finish_dist = finish_dist;

            // 統計収集（10ms毎）
            if (static_cast<int>(t * 1000) % 10 == 0 && t > 0.0) {
                auto nearest = track_query.nearest(vehicle.state().x, vehicle.state().y, hint_s);
                hint_s = nearest.s;

                double lateral = std::abs(nearest.n);
                sum_lateral_sq += lateral * lateral;
                max_lateral = std::max(max_lateral, lateral);

                double steering_rate = (input.steering - prev_steering) / 0.01;
                sum_steering_rate_sq += steering_rate * steering_rate;
                prev_steering = input.steering;

                sample_count++;
            }

            if (result.converged) break;
        }

        // 統計計算
        if (sample_count > 0) {
            result.lateral_error_rms = std::sqrt(sum_lateral_sq / sample_count);
            result.lateral_error_max = max_lateral;
            result.steering_rate_rms = std::sqrt(sum_steering_rate_sq / sample_count);
        }

        // MPC統計
        auto stats = driver.stats();
        result.avg_solve_time_ms = stats.avg_solve_time_ms();
        result.solve_failures = stats.fail_count;

    } catch (const std::exception& e) {
        result.converged = false;
    }

    return result;
}
#endif

int main() {
    std::cout << "=== Phase 8c: MPC Parameter Tuning ===" << std::endl;

#ifndef HAS_OSQP
    std::cerr << "Error: OSQP not available" << std::endl;
    return 1;
#endif


#ifdef HAS_OSQP
    // トラック生成
    auto track = track::create_circle_track(30.0, 3.0);
    std::cout << "Track: Circle (R=30m, L=" << track.length() << "m)" << std::endl;

    // 速度プロファイル
    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 30.0;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 1.0);

    // PIDベンチマーク（目標性能）
    std::cout << "\n=== PID Benchmark ===" << std::endl;
    std::cout << "Target lateral error RMS: ~1.5 cm" << std::endl;

    // グリッドサーチ
    std::cout << "\n=== Grid Search ===" << std::endl;

    // 重みの候補
    std::vector<double> w_xy_candidates = {50, 100, 200, 500, 1000};
    std::vector<double> w_delta_rate_candidates = {0.5, 1.0, 2.0, 5.0};
    std::vector<double> Cf_candidates = {16607, 40000, 60000};  // 実測値、デフォルト、大きめ

    std::vector<TuningResult> results;

    int total_runs = w_xy_candidates.size() * w_delta_rate_candidates.size() * Cf_candidates.size();
    int run_count = 0;

    std::cout << "Total configurations: " << total_runs << std::endl;
    std::cout << std::string(60, '=') << std::endl;

    for (double w_xy : w_xy_candidates) {
        for (double w_delta_rate : w_delta_rate_candidates) {
            for (double Cf : Cf_candidates) {
                run_count++;

                control::MPCConfig config;
                config.N = 20;
                config.dt = 0.025;
                config.w_x = w_xy;
                config.w_y = w_xy;
                config.w_psi = w_xy / 2.5;  // 比例
                config.w_delta_rate = w_delta_rate;

                double Cr = Cf * 0.9;  // 前後比90%

                std::cout << "\r[" << run_count << "/" << total_runs << "] "
                          << "w_xy=" << std::setw(5) << w_xy
                          << " w_dr=" << std::setw(4) << w_delta_rate
                          << " Cf=" << std::setw(5) << static_cast<int>(Cf/1000) << "k"
                          << std::flush;

                auto result = run_mpc_tuning(track, velocity_profile, config, Cf, Cr);

                if (result.converged && result.solve_failures == 0) {
                    results.push_back(result);
                }
            }
        }
    }

    std::cout << "\n" << std::string(60, '=') << std::endl;
    std::cout << "\nCompleted " << results.size() << " successful runs" << std::endl;

    if (results.empty()) {
        std::cout << "No successful configurations found!" << std::endl;
        return 1;
    }

    // 結果をソート（横偏差RMSが小さい順）
    std::sort(results.begin(), results.end(),
              [](const TuningResult& a, const TuningResult& b) {
                  return a.lateral_error_rms < b.lateral_error_rms;
              });

    // トップ10を表示
    std::cout << "\n=== Top 10 Configurations (by lateral error RMS) ===" << std::endl;
    std::cout << std::string(100, '-') << std::endl;
    std::cout << std::setw(6) << "Rank"
              << std::setw(8) << "w_xy"
              << std::setw(8) << "w_dr"
              << std::setw(8) << "Cf(k)"
              << std::setw(12) << "Lat.RMS(cm)"
              << std::setw(12) << "Lat.Max(cm)"
              << std::setw(12) << "Steer.RMS"
              << std::setw(12) << "Lap(s)"
              << std::setw(10) << "Solve(ms)"
              << std::endl;
    std::cout << std::string(100, '-') << std::endl;

    for (size_t i = 0; i < std::min(size_t(10), results.size()); ++i) {
        const auto& r = results[i];
        std::cout << std::setw(6) << (i + 1)
                  << std::setw(8) << static_cast<int>(r.w_x)
                  << std::setw(8) << r.w_delta_rate
                  << std::setw(8) << static_cast<int>(r.Cf / 1000)
                  << std::setw(12) << std::fixed << std::setprecision(2) << r.lateral_error_rms * 100
                  << std::setw(12) << r.lateral_error_max * 100
                  << std::setw(12) << std::setprecision(3) << r.steering_rate_rms
                  << std::setw(12) << std::setprecision(3) << r.lap_time
                  << std::setw(10) << std::setprecision(3) << r.avg_solve_time_ms
                  << std::endl;
    }

    // 最良設定をファイル出力
    std::ofstream ofs("mpc_best_config.txt");
    const auto& best = results[0];
    ofs << "// Best MPC Configuration (Phase 8c)\n";
    ofs << "config.N = 20;\n";
    ofs << "config.dt = 0.025;\n";
    ofs << "config.w_x = " << best.w_x << ";\n";
    ofs << "config.w_y = " << best.w_y << ";\n";
    ofs << "config.w_psi = " << best.w_psi << ";\n";
    ofs << "config.w_delta_rate = " << best.w_delta_rate << ";\n";
    ofs << "// Bicycle model: Cf = " << best.Cf << ", Cr = " << best.Cr << "\n";
    ofs << "// Performance: lateral_rms=" << (best.lateral_error_rms * 100) << "cm, lap=" << best.lap_time << "s\n";

    std::cout << "\nBest configuration saved to mpc_best_config.txt" << std::endl;
#endif

    return 0;
}
