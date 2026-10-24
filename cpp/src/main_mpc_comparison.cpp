/// @file main_mpc_comparison.cpp
/// @brief PID vs MPC 性能比較シミュレーション (Phase 8b)
///
/// 同一トラック・速度プロファイルでPIDとMPCドライバーを比較
/// 評価指標: ラップタイム、横偏差RMS、制御平滑性、計算コスト

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
#include <string>
#include <cmath>
#include <vector>
#include <numeric>

using namespace fsae;

/// State14DOF → VehicleState 変換ヘルパー
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

/// シミュレーション結果
struct SimResult {
    std::string driver_name;
    double lap_time = 0.0;              // [s]
    double distance_traveled = 0.0;     // [m]

    // 横偏差統計
    double max_lateral_error = 0.0;     // [m]
    double rms_lateral_error = 0.0;     // [m]
    double avg_lateral_error = 0.0;     // [m]

    // 速度統計
    double max_speed = 0.0;             // [m/s]
    double avg_speed = 0.0;             // [m/s]
    double rms_speed_error = 0.0;       // [m/s]

    // 制御平滑性（レート）
    double max_steering_rate = 0.0;     // [rad/s]
    double rms_steering_rate = 0.0;     // [rad/s]
    double max_throttle_rate = 0.0;     // [1/s]
    double rms_throttle_rate = 0.0;     // [1/s]

    // MPC専用統計
    double avg_solve_time_ms = 0.0;     // [ms]
    double max_solve_time_ms = 0.0;     // [ms]
    int solve_fail_count = 0;

    bool lap_completed = false;

    // タイムスタンプログ
    std::vector<double> time_log;
    std::vector<double> lateral_error_log;
    std::vector<double> speed_error_log;
    std::vector<double> steering_log;
    std::vector<double> throttle_log;
};

/// シミュレーション設定
struct ComparisonConfig {
    std::string track_name = "circle";
    double track_radius = 30.0;        // [m]
    double track_width = 3.0;          // [m]
    double simulation_time = 30.0;     // [s]
    double dt = 0.001;                 // [s] (1ms)
    double output_rate = 0.01;         // [s] (10ms毎に記録)

    // 車両性能
    double max_lat_accel = 1.5 * 9.81;
    double max_lon_accel = 1.0 * 9.81;
    double max_lon_decel = 1.8 * 9.81;
    double max_speed = 30.0;
    double initial_speed = 8.0;        // [m/s]
};

/// トラック生成
track::CurvatureBasedTrack create_track(const ComparisonConfig& config) {
    if (config.track_name == "circle") {
        return track::create_circle_track(config.track_radius, config.track_width);
    } else if (config.track_name == "figure_eight") {
        return track::create_figure_eight_track(config.track_radius, config.track_width);
    } else {
        return track::create_circle_track(config.track_radius, config.track_width);
    }
}

/// 統計計算ヘルパー
template<typename T>
double compute_rms(const std::vector<T>& data) {
    if (data.empty()) return 0.0;
    double sum_sq = 0.0;
    for (const auto& val : data) {
        sum_sq += val * val;
    }
    return std::sqrt(sum_sq / data.size());
}

template<typename T>
double compute_max_abs(const std::vector<T>& data) {
    if (data.empty()) return 0.0;
    double max_val = 0.0;
    for (const auto& val : data) {
        max_val = std::max(max_val, std::abs(val));
    }
    return max_val;
}

/// PIDドライバーでシミュレーション実行
SimResult run_with_pid(
    const ComparisonConfig& config,
    const track::CurvatureBasedTrack& track,
    const std::vector<laptime::VelocityPoint>& velocity_profile)
{
    std::cout << "\n=== Running with PID Driver ===" << std::endl;

    SimResult result;
    result.driver_name = "PID";

    // PIDドライバー初期化
    driver::PIDGains gains;  // デフォルトゲイン
    driver::PIDPathFollowingDriver driver(track, velocity_profile, gains);

    // 車両初期化
    vehicle::VehicleParams vehicle_params;
    vehicle::Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(config.initial_speed);

    // 初期位置
    vehicle.state().x = config.track_radius;
    vehicle.state().y = 0.0;
    vehicle.state().z = 0.0;
    vehicle.state().psi = M_PI / 2.0;

    // ゴールテープ設定
    double start_x = vehicle.state().x;
    double start_y = vehicle.state().y;
    double start_heading = vehicle.state().psi;
    double finish_nx = std::cos(start_heading);
    double finish_ny = std::sin(start_heading);
    double prev_finish_dist = 0.0;
    bool crossed_away = false;

    // 走行距離計測
    double prev_x = vehicle.state().x;
    double prev_y = vehicle.state().y;
    double absolute_distance = 0.0;

    // 統計用
    double sum_lateral_error_sq = 0.0;
    double sum_speed_error_sq = 0.0;
    double sum_speed = 0.0;
    int sample_count = 0;

    // レート計算用
    double prev_steering = 0.0;
    double prev_throttle = 0.0;
    std::vector<double> steering_rate_log;
    std::vector<double> throttle_rate_log;

    double last_output_time = -config.output_rate;

    for (double t = 0.0; t < config.simulation_time; t += config.dt) {
        // 制御入力計算
        auto input = driver.compute_control(vehicle.state(), t);

        // 車両ステップ
        vehicle.step(config.dt, input);

        // Frenet座標取得
        auto frenet = driver.get_current_frenet(vehicle.state());

        // 走行距離更新
        if (t > 0.0) {
            double dx = vehicle.state().x - prev_x;
            double dy = vehicle.state().y - prev_y;
            absolute_distance += std::sqrt(dx * dx + dy * dy);
            prev_x = vehicle.state().x;
            prev_y = vehicle.state().y;
        }

        // ゴールテープ判定
        double finish_dist = (vehicle.state().x - start_x) * finish_nx
                           + (vehicle.state().y - start_y) * finish_ny;

        if (!result.lap_completed) {
            if (!crossed_away && absolute_distance > track.length() * 0.3) {
                crossed_away = true;
            }

            if (crossed_away && absolute_distance > track.length() * 0.8) {
                if ((prev_finish_dist < 0.0 && finish_dist >= 0.0) ||
                    (prev_finish_dist > 0.0 && finish_dist <= 0.0)) {
                    double dx_start = vehicle.state().x - start_x;
                    double dy_start = vehicle.state().y - start_y;
                    double dist_to_start = std::sqrt(dx_start * dx_start + dy_start * dy_start);
                    if (dist_to_start < track.length() * 0.1) {
                        result.lap_completed = true;
                        result.lap_time = t;
                        result.distance_traveled = absolute_distance;
                        std::cout << "Lap completed at t=" << t << " s" << std::endl;
                    }
                }
            }
        }

        prev_finish_dist = finish_dist;

        // 統計収集（output_rate毎）
        if (t - last_output_time >= config.output_rate) {
            last_output_time = t;

            double lateral_error = std::abs(frenet.n);
            double speed = vehicle.state().speed();
            // 速度プロファイルから目標速度を線形補間
            double target_speed = 10.0;  // デフォルト値
            for (size_t i = 0; i < velocity_profile.size() - 1; ++i) {
                if (frenet.s >= velocity_profile[i].s && frenet.s < velocity_profile[i+1].s) {
                    double alpha = (frenet.s - velocity_profile[i].s) /
                                   (velocity_profile[i+1].s - velocity_profile[i].s);
                    target_speed = velocity_profile[i].v + alpha * (velocity_profile[i+1].v - velocity_profile[i].v);
                    break;
                }
            }
            double speed_error = speed - target_speed;

            result.max_lateral_error = std::max(result.max_lateral_error, lateral_error);
            result.max_speed = std::max(result.max_speed, speed);

            sum_lateral_error_sq += lateral_error * lateral_error;
            sum_speed_error_sq += speed_error * speed_error;
            sum_speed += speed;
            sample_count++;

            // ログ記録
            result.time_log.push_back(t);
            result.lateral_error_log.push_back(lateral_error);
            result.speed_error_log.push_back(speed_error);
            result.steering_log.push_back(input.steering);
            result.throttle_log.push_back(input.throttle);

            // レート計算
            if (sample_count > 1) {
                double steering_rate = (input.steering - prev_steering) / config.output_rate;
                double throttle_rate = (input.throttle - prev_throttle) / config.output_rate;
                steering_rate_log.push_back(steering_rate);
                throttle_rate_log.push_back(throttle_rate);
            }

            prev_steering = input.steering;
            prev_throttle = input.throttle;
        }

        // ラップ完了後は終了
        if (result.lap_completed) break;
    }

    // 統計計算
    if (sample_count > 0) {
        result.rms_lateral_error = std::sqrt(sum_lateral_error_sq / sample_count);
        result.rms_speed_error = std::sqrt(sum_speed_error_sq / sample_count);
        result.avg_speed = sum_speed / sample_count;
    }

    result.rms_steering_rate = compute_rms(steering_rate_log);
    result.max_steering_rate = compute_max_abs(steering_rate_log);
    result.rms_throttle_rate = compute_rms(throttle_rate_log);
    result.max_throttle_rate = compute_max_abs(throttle_rate_log);

    return result;
}

#ifdef HAS_OSQP
/// MPCドライバーでシミュレーション実行
SimResult run_with_mpc(
    const ComparisonConfig& config,
    const track::CurvatureBasedTrack& track,
    const std::vector<laptime::VelocityPoint>& velocity_profile)
{
    std::cout << "\n=== Running with MPC Driver ===" << std::endl;

    SimResult result;
    result.driver_name = "MPC";

    // MPC設定（横偏差重視にチューニング）
    control::MPCConfig mpc_config;
    mpc_config.N = 20;
    mpc_config.dt = 0.025;  // 40Hz

    // 重み調整: 横偏差を重視
    mpc_config.w_x = 50.0;      // 5.0 → 50.0 (10倍)
    mpc_config.w_y = 50.0;      // 5.0 → 50.0 (10倍)
    mpc_config.w_psi = 20.0;    // 2.0 → 20.0 (10倍)
    mpc_config.w_delta_rate = 1.0;  // 0.1 → 1.0 (平滑性向上)

    // 14DOFパラメータから自転車パラメータを正確に抽出
    vehicle::VehicleParams vp;
    auto bicycle_params = vehicle::BicycleParams::from_vehicle_params(vp);

    std::cout << "Bicycle model params:" << std::endl;
    std::cout << "  Cf = " << bicycle_params.Cf << " N/rad" << std::endl;
    std::cout << "  Cr = " << bicycle_params.Cr << " N/rad" << std::endl;

    // MPCドライバー初期化（自転車モデルパラメータは内部で設定される）
    driver::MPCPathFollowingDriver driver(track, velocity_profile, mpc_config);

    // 車両初期化
    vehicle::VehicleParams vehicle_params;
    vehicle::Vehicle14DOF_New vehicle(vehicle_params);
    vehicle.reset();
    vehicle.set_initial_velocity(config.initial_speed);

    // 初期位置
    vehicle.state().x = config.track_radius;
    vehicle.state().y = 0.0;
    vehicle.state().z = 0.0;
    vehicle.state().psi = M_PI / 2.0;

    // ゴールテープ設定
    double start_x = vehicle.state().x;
    double start_y = vehicle.state().y;
    double start_heading = vehicle.state().psi;
    double finish_nx = std::cos(start_heading);
    double finish_ny = std::sin(start_heading);
    double prev_finish_dist = 0.0;
    bool crossed_away = false;

    // 走行距離計測
    double prev_x = vehicle.state().x;
    double prev_y = vehicle.state().y;
    double absolute_distance = 0.0;

    // 統計用
    double sum_lateral_error_sq = 0.0;
    double sum_speed_error_sq = 0.0;
    double sum_speed = 0.0;
    int sample_count = 0;

    // レート計算用
    double prev_steering = 0.0;
    double prev_throttle = 0.0;
    std::vector<double> steering_rate_log;
    std::vector<double> throttle_rate_log;

    // Frenet座標計算用（TrackDistanceQuery）
    track::TrackDistanceQuery track_query(track, 0.5);
    double hint_s = -1.0;

    double last_output_time = -config.output_rate;
    double last_mpc_time = -mpc_config.dt;
    vehicle::DriverInput input;

    for (double t = 0.0; t < config.simulation_time; t += config.dt) {
        // MPC制御入力計算（40Hz = 25ms毎）
        if (t - last_mpc_time >= mpc_config.dt) {
            last_mpc_time = t;
            auto vs = to_vehicle_state(vehicle.state());
            input = driver.compute_control(vs, t);
        }

        // 車両ステップ（1kHz）
        vehicle.step(config.dt, input);

        // 走行距離更新
        if (t > 0.0) {
            double dx = vehicle.state().x - prev_x;
            double dy = vehicle.state().y - prev_y;
            absolute_distance += std::sqrt(dx * dx + dy * dy);
            prev_x = vehicle.state().x;
            prev_y = vehicle.state().y;
        }

        // ゴールテープ判定
        double finish_dist = (vehicle.state().x - start_x) * finish_nx
                           + (vehicle.state().y - start_y) * finish_ny;

        if (!result.lap_completed) {
            if (!crossed_away && absolute_distance > track.length() * 0.3) {
                crossed_away = true;
            }

            if (crossed_away && absolute_distance > track.length() * 0.8) {
                if ((prev_finish_dist < 0.0 && finish_dist >= 0.0) ||
                    (prev_finish_dist > 0.0 && finish_dist <= 0.0)) {
                    double dx_start = vehicle.state().x - start_x;
                    double dy_start = vehicle.state().y - start_y;
                    double dist_to_start = std::sqrt(dx_start * dx_start + dy_start * dy_start);
                    if (dist_to_start < track.length() * 0.1) {
                        result.lap_completed = true;
                        result.lap_time = t;
                        result.distance_traveled = absolute_distance;
                        std::cout << "Lap completed at t=" << t << " s" << std::endl;
                    }
                }
            }
        }

        prev_finish_dist = finish_dist;

        // 統計収集（output_rate毎）
        if (t - last_output_time >= config.output_rate) {
            last_output_time = t;

            // Frenet座標計算
            auto nearest = track_query.nearest(vehicle.state().x, vehicle.state().y, hint_s);
            hint_s = nearest.s;
            double lateral_error = std::abs(nearest.n);

            double speed = vehicle.state().speed();
            // 簡易的な目標速度（ドライバー内部の速度プロファイルから推定）
            double target_speed = 10.0;  // 仮値（本来はdriver内部から取得）
            double speed_error = speed - target_speed;

            result.max_lateral_error = std::max(result.max_lateral_error, lateral_error);
            result.max_speed = std::max(result.max_speed, speed);

            sum_lateral_error_sq += lateral_error * lateral_error;
            sum_speed_error_sq += speed_error * speed_error;
            sum_speed += speed;
            sample_count++;

            // ログ記録
            result.time_log.push_back(t);
            result.lateral_error_log.push_back(lateral_error);
            result.speed_error_log.push_back(speed_error);
            result.steering_log.push_back(input.steering);
            result.throttle_log.push_back(input.throttle);

            // レート計算
            if (sample_count > 1) {
                double steering_rate = (input.steering - prev_steering) / config.output_rate;
                double throttle_rate = (input.throttle - prev_throttle) / config.output_rate;
                steering_rate_log.push_back(steering_rate);
                throttle_rate_log.push_back(throttle_rate);
            }

            prev_steering = input.steering;
            prev_throttle = input.throttle;
        }

        // ラップ完了後は終了
        if (result.lap_completed) break;
    }

    // 統計計算
    if (sample_count > 0) {
        result.rms_lateral_error = std::sqrt(sum_lateral_error_sq / sample_count);
        result.rms_speed_error = std::sqrt(sum_speed_error_sq / sample_count);
        result.avg_speed = sum_speed / sample_count;
    }

    result.rms_steering_rate = compute_rms(steering_rate_log);
    result.max_steering_rate = compute_max_abs(steering_rate_log);
    result.rms_throttle_rate = compute_rms(throttle_rate_log);
    result.max_throttle_rate = compute_max_abs(throttle_rate_log);

    // MPC統計取得
    auto mpc_stats = driver.stats();
    result.avg_solve_time_ms = mpc_stats.avg_solve_time_ms();
    result.max_solve_time_ms = mpc_stats.max_solve_time_ms;
    result.solve_fail_count = mpc_stats.fail_count;

    return result;
}
#endif

/// 結果を表示
void print_comparison(const SimResult& pid_result, const SimResult& mpc_result) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "   PID vs MPC Performance Comparison" << std::endl;
    std::cout << "========================================\n" << std::endl;

    // ラップタイム
    std::cout << "Lap Time:" << std::endl;
    std::cout << "  PID: " << std::fixed << std::setprecision(3) << pid_result.lap_time << " s" << std::endl;
    std::cout << "  MPC: " << mpc_result.lap_time << " s" << std::endl;
    if (pid_result.lap_completed && mpc_result.lap_completed) {
        double diff = mpc_result.lap_time - pid_result.lap_time;
        double pct = (diff / pid_result.lap_time) * 100.0;
        std::cout << "  Difference: " << std::showpos << diff << " s (" << pct << "%)" << std::noshowpos << std::endl;
    }

    // 横偏差
    std::cout << "\nLateral Error:" << std::endl;
    std::cout << "  PID RMS: " << pid_result.rms_lateral_error * 100.0 << " cm" << std::endl;
    std::cout << "  MPC RMS: " << mpc_result.rms_lateral_error * 100.0 << " cm" << std::endl;
    double lateral_improvement = (1.0 - mpc_result.rms_lateral_error / pid_result.rms_lateral_error) * 100.0;
    std::cout << "  MPC improvement: " << std::showpos << lateral_improvement << "%" << std::noshowpos << std::endl;

    std::cout << "  PID Max: " << pid_result.max_lateral_error * 100.0 << " cm" << std::endl;
    std::cout << "  MPC Max: " << mpc_result.max_lateral_error * 100.0 << " cm" << std::endl;

    // 制御平滑性
    std::cout << "\nControl Smoothness (Steering Rate):" << std::endl;
    std::cout << "  PID RMS: " << pid_result.rms_steering_rate << " rad/s" << std::endl;
    std::cout << "  MPC RMS: " << mpc_result.rms_steering_rate << " rad/s" << std::endl;
    double smooth_improvement = (1.0 - mpc_result.rms_steering_rate / pid_result.rms_steering_rate) * 100.0;
    std::cout << "  MPC improvement: " << std::showpos << smooth_improvement << "%" << std::noshowpos << std::endl;

    // MPC計算コスト
    std::cout << "\nMPC Computation Cost:" << std::endl;
    std::cout << "  Average solve time: " << mpc_result.avg_solve_time_ms << " ms" << std::endl;
    std::cout << "  Max solve time: " << mpc_result.max_solve_time_ms << " ms" << std::endl;
    std::cout << "  Solve failures: " << mpc_result.solve_fail_count << std::endl;

    std::cout << "\n========================================\n" << std::endl;
}

/// 結果をCSVに保存
void save_results_csv(const SimResult& pid_result, const SimResult& mpc_result, const std::string& filename) {
    std::ofstream ofs(filename);
    ofs << "time,pid_lateral_error,mpc_lateral_error,pid_steering,mpc_steering,pid_throttle,mpc_throttle\n";

    size_t n = std::min(pid_result.time_log.size(), mpc_result.time_log.size());
    for (size_t i = 0; i < n; ++i) {
        ofs << pid_result.time_log[i] << ","
            << pid_result.lateral_error_log[i] << ","
            << mpc_result.lateral_error_log[i] << ","
            << pid_result.steering_log[i] << ","
            << mpc_result.steering_log[i] << ","
            << pid_result.throttle_log[i] << ","
            << mpc_result.throttle_log[i] << "\n";
    }

    std::cout << "Results saved to " << filename << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "=== Phase 8b: PID vs MPC Comparison Simulation ===" << std::endl;

#ifndef HAS_OSQP
    std::cerr << "Error: OSQP not compiled. MPC driver unavailable." << std::endl;
    std::cerr << "Please rebuild with -DHAS_OSQP=1" << std::endl;
    return 1;
#endif

    ComparisonConfig config;

    // コマンドライン引数解析
    if (argc > 1) {
        config.track_name = argv[1];
    }

    std::cout << "Track: " << config.track_name << std::endl;
    std::cout << "Radius: " << config.track_radius << " m" << std::endl;

    // 1. トラック生成
    auto track = create_track(config);
    std::cout << "Track length: " << track.length() << " m" << std::endl;

    // 2. 速度プロファイル生成
    laptime::VehiclePerformance perf;
    perf.max_lat_accel = config.max_lat_accel;
    perf.max_lon_accel = config.max_lon_accel;
    perf.max_lon_decel = config.max_lon_decel;
    perf.max_speed = config.max_speed;

    laptime::VelocityProfileGenerator vel_gen(perf);
    auto velocity_profile = vel_gen.generate(track, 1.0);

    double theoretical_lap_time = laptime::compute_lap_time(velocity_profile);
    std::cout << "Theoretical lap time: " << theoretical_lap_time << " s\n" << std::endl;

    // 3. PIDシミュレーション
    auto pid_result = run_with_pid(config, track, velocity_profile);

#ifdef HAS_OSQP
    // 4. MPCシミュレーション
    auto mpc_result = run_with_mpc(config, track, velocity_profile);

    // 5. 結果比較
    print_comparison(pid_result, mpc_result);

    // 6. CSV保存
    save_results_csv(pid_result, mpc_result, "mpc_comparison_" + config.track_name + ".csv");
#else
    std::cout << "\nMPC simulation skipped (OSQP not available)" << std::endl;
#endif

    return 0;
}
