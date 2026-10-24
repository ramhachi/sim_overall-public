// src/main_14dof.cpp
/// 14DOFフルビークルモデル デモ

#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <array>
#include <stdexcept>
#include <string>

#include "fsae/vehicle/vehicle_14dof_new.hpp"

#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

namespace {
std::string ensure_output_path(const std::string& filename) {
    std::filesystem::path output_dir = std::filesystem::path("output") / "14dof";
    std::filesystem::create_directories(output_dir);
    return (output_dir / filename).string();
}

std::array<double, 4> tire_slip_angle_deg(const Vehicle14DOF_New& vehicle) {
    const auto& s = vehicle.state();
    return {
        s.tire_eta[0] * 180.0 / M_PI,
        s.tire_eta[1] * 180.0 / M_PI,
        s.tire_eta[2] * 180.0 / M_PI,
        s.tire_eta[3] * 180.0 / M_PI,
    };
}
}

/// 14DOFシミュレーション時系列データ
struct SimData {
    std::vector<double> time;
    std::vector<double> x, y, psi;
    std::vector<double> vx, vy, r;
    std::vector<double> ax, ay;
    std::vector<double> steering, throttle, brake;
    std::vector<int> gear;
    std::vector<double> engine_rpm, engine_torque;
    std::vector<double> omega_RL, omega_RR, omega_diff;
    std::vector<std::array<double, 4>> Fz;
    std::vector<std::array<double, 4>> slip_angle;
    
    void reserve(size_t n) {
        time.reserve(n);
        x.reserve(n); y.reserve(n); psi.reserve(n);
        vx.reserve(n); vy.reserve(n); r.reserve(n);
        ax.reserve(n); ay.reserve(n);
        steering.reserve(n); throttle.reserve(n); brake.reserve(n);
        gear.reserve(n);
        engine_rpm.reserve(n); engine_torque.reserve(n);
        omega_RL.reserve(n); omega_RR.reserve(n); omega_diff.reserve(n);
        Fz.reserve(n); slip_angle.reserve(n);
    }
    
    void save_csv(const std::string& filename) {
        std::ofstream out(filename);
        out << "time,x,y,psi,vx,vy,r,ax,ay,steering,throttle,brake,gear,"
            << "engine_rpm,engine_torque,omega_RL,omega_RR,omega_diff,"
            << "Fz_FL,Fz_FR,Fz_RL,Fz_RR,"
            << "slip_angle_deg_FL,slip_angle_deg_FR,slip_angle_deg_RL,slip_angle_deg_RR\n";
        
        for (size_t i = 0; i < time.size(); ++i) {
            out << time[i] << ","
                << x[i] << "," << y[i] << "," << psi[i] << ","
                << vx[i] << "," << vy[i] << "," << r[i] << ","
                << ax[i] << "," << ay[i] << ","
                << steering[i] << "," << throttle[i] << "," << brake[i] << ","
                << gear[i] << ","
                << engine_rpm[i] << "," << engine_torque[i] << ","
                << omega_RL[i] << "," << omega_RR[i] << "," << omega_diff[i] << ","
                << Fz[i][0] << "," << Fz[i][1] << "," << Fz[i][2] << "," << Fz[i][3] << ","
                << slip_angle[i][0] << "," << slip_angle[i][1] << "," 
                << slip_angle[i][2] << "," << slip_angle[i][3] << "\n";
        }
        spdlog::info("Saved {} data points to {}", time.size(), filename);
    }
};

/// ステップステア試験
void run_step_steer_test(double pinn_eval_rate_hz) {
    spdlog::info("=== Step Steer Test (14DOF) ===");
    
    // パラメータ
    auto params = VehicleParams::default_fsae();
    params.tire.pinn_eval_rate_hz = pinn_eval_rate_hz;
    Vehicle14DOF_New vehicle(params);
    
    // 初期速度 20 m/s (72 km/h) - MATLAB と統一
    vehicle.set_initial_velocity(20.0);
    
    // シミュレーション設定
    const double t_end = 5.0;
    const double dt = 0.001;
    const double step_time = 1.0;
    const double step_steer = 0.1;  // 5.7 deg
    
    SimData data;
    data.reserve(static_cast<size_t>(t_end / dt) + 1);
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    while (vehicle.time() < t_end) {
        // 入力
        DriverInput input;
        input.throttle = 0.3;  // 定速維持
        input.steering = (vehicle.time() >= step_time) ? step_steer : 0.0;
        input.gear = 3;  // 3速固定
        
        // デバッグ: 最初の数ステップでのタイヤ力を表示
        if (vehicle.time() < 0.01 || (vehicle.time() >= 1.0 && vehicle.time() < 1.01)) {
            auto tire_Fx = vehicle.get_tire_Fx();
            auto tire_Fy = vehicle.get_tire_Fy();
            auto tire_Fz = vehicle.get_tire_Fz();
            spdlog::info("t={:.3f} vx={:.2f} r={:.3f} steer={:.3f}", 
                         vehicle.time(), vehicle.state().vx, vehicle.state().r, input.steering);
            spdlog::info("  Tire Fx: [{:.1f}, {:.1f}, {:.1f}, {:.1f}]",
                         tire_Fx[0], tire_Fx[1], tire_Fx[2], tire_Fx[3]);
            spdlog::info("  Tire Fy: [{:.1f}, {:.1f}, {:.1f}, {:.1f}]",
                         tire_Fy[0], tire_Fy[1], tire_Fy[2], tire_Fy[3]);
            spdlog::info("  Tire Fz: [{:.1f}, {:.1f}, {:.1f}, {:.1f}]",
                         tire_Fz[0], tire_Fz[1], tire_Fz[2], tire_Fz[3]);
        }
        
        // データ保存
        auto& s = vehicle.state();
        data.time.push_back(vehicle.time());
        data.x.push_back(s.x);
        data.y.push_back(s.y);
        data.psi.push_back(s.psi * 180.0 / M_PI);  // deg
        data.vx.push_back(s.vx);
        data.vy.push_back(s.vy);
        data.r.push_back(s.r * 180.0 / M_PI);  // deg/s
        data.ax.push_back((s.vx - vx_prev) / dt);
        data.ay.push_back((s.vy - vy_prev) / dt + s.vx * s.r);  // 遠心力補正
        data.steering.push_back(input.steering * 180.0 / M_PI);
        data.throttle.push_back(input.throttle);
        data.brake.push_back(input.brake);
        data.gear.push_back(input.gear);
        data.engine_rpm.push_back(vehicle.get_engine_rpm(input.gear));
        data.engine_torque.push_back(vehicle.get_engine_torque(input.gear, input.throttle));
        data.omega_RL.push_back(s.omega[2]);
        data.omega_RR.push_back(s.omega[3]);
        data.omega_diff.push_back(s.omega[2] - s.omega[3]);
        
        data.Fz.push_back(vehicle.get_tire_Fz());
        data.slip_angle.push_back(tire_slip_angle_deg(vehicle));
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        // 積分
        vehicle.step(dt, input);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    auto& s = vehicle.state();
    spdlog::info("Simulation complete:");
    spdlog::info("  Duration: {:.2f} s", t_end);
    spdlog::info("  Data points: {}", data.time.size());
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final position: x={:.1f} m, y={:.1f} m", s.x, s.y);
    spdlog::info("  Final yaw: {:.1f} deg", s.psi * 180.0 / M_PI);
    spdlog::info("  Final speed: {:.1f} m/s", s.speed());
    
    // CSV出力
    data.save_csv(ensure_output_path("step_steer_result.csv"));
}

/// 定常円旋回試験
void run_steady_state_circle_test(double pinn_eval_rate_hz) {
    spdlog::info("=== Steady State Circle Test (14DOF) ===");
    
    auto params = VehicleParams::default_fsae();
    params.tire.pinn_eval_rate_hz = pinn_eval_rate_hz;
    Vehicle14DOF_New vehicle(params);
    
    // 初期速度 10 m/s
    vehicle.set_initial_velocity(10.0);
    
    const double t_end = 10.0;
    const double dt = 0.001;
    const double target_radius = 15.0;  // 15m円
    
    SimData data;
    data.reserve(static_cast<size_t>(t_end / dt) + 1);
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    while (vehicle.time() < t_end) {
        auto& s = vehicle.state();
        
        // ステアリングを調整して目標半径を維持
        double current_radius = (std::abs(s.r) > 0.01) ? s.speed() / std::abs(s.r) : 1000;
        double steer_cmd = std::atan(params.mass.wheelbase / target_radius);
        
        DriverInput input;
        input.throttle = 0.25;
        input.steering = steer_cmd;
        input.gear = 2;  // 2速
        
        // データ保存（100ms毎）
        if (static_cast<int>(vehicle.time() * 1000) % 100 == 0) {
            data.time.push_back(vehicle.time());
            data.x.push_back(s.x);
            data.y.push_back(s.y);
            data.psi.push_back(s.psi * 180.0 / M_PI);
            data.vx.push_back(s.vx);
            data.vy.push_back(s.vy);
            data.r.push_back(s.r * 180.0 / M_PI);
            data.ax.push_back((s.vx - vx_prev) / dt);
            data.ay.push_back(s.vx * s.r);
            data.steering.push_back(input.steering * 180.0 / M_PI);
            data.throttle.push_back(input.throttle);
            data.brake.push_back(0);
            data.gear.push_back(input.gear);
            data.engine_rpm.push_back(vehicle.get_engine_rpm(input.gear));
            data.engine_torque.push_back(vehicle.get_engine_torque(input.gear, input.throttle));
            data.omega_RL.push_back(s.omega[2]);
            data.omega_RR.push_back(s.omega[3]);
            data.omega_diff.push_back(s.omega[2] - s.omega[3]);
            data.Fz.push_back(vehicle.get_tire_Fz());
            data.slip_angle.push_back(tire_slip_angle_deg(vehicle));
        }
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        vehicle.step(dt, input);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    auto& s = vehicle.state();
    spdlog::info("Simulation complete:");
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final position: x={:.1f} m, y={:.1f} m", s.x, s.y);
    
    data.save_csv(ensure_output_path("steady_circle_result.csv"));
}

/// 加速試験（エンジン・LSD性能評価）
void run_acceleration_test(double pinn_eval_rate_hz) {
    spdlog::info("=== Acceleration Test (14DOF with Engine & LSD) ===")
;
    
    auto params = VehicleParams::default_fsae();
    params.tire.pinn_eval_rate_hz = pinn_eval_rate_hz;
    Vehicle14DOF_New vehicle(params);
    
    // 初速 5 m/s（2速で開始）
    vehicle.set_initial_velocity(5.0);
    
    const double t_end = 10.0;
    const double dt = 0.001;
    
    SimData data;
    data.reserve(static_cast<size_t>(t_end / dt) + 1);
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    // 自動シフトアップの閾値
    const double shift_rpm = 13000.0;  // 13000 rpmでシフトアップ
    int current_gear = 2;  // 2速スタート
    
    auto start = std::chrono::high_resolution_clock::now();
    
    while (vehicle.time() < t_end) {
        auto& s = vehicle.state();
        
        // 自動シフトアップロジック
        double current_rpm = vehicle.get_engine_rpm(current_gear);
        if (current_rpm > shift_rpm && current_gear < 6 && vehicle.time() > 0.5) {
            current_gear++;
            spdlog::info("  Shift up to gear {} at t={:.2f}s, v={:.1f} km/h, rpm={:.0f}", 
                        current_gear, vehicle.time(), s.vx * 3.6, current_rpm);
        }
        
        // 入力
        DriverInput input;
        input.throttle = 1.0;  // 全開加速
        input.steering = 0.0;  // 直進
        input.gear = current_gear;
        
        // データ保存（10ms毎）
        if (static_cast<int>(vehicle.time() * 1000) % 10 == 0) {
            data.time.push_back(vehicle.time());
            data.x.push_back(s.x);
            data.y.push_back(s.y);
            data.psi.push_back(s.psi * 180.0 / M_PI);
            data.vx.push_back(s.vx);
            data.vy.push_back(s.vy);
            data.r.push_back(s.r * 180.0 / M_PI);
            data.ax.push_back((s.vx - vx_prev) / dt);
            data.ay.push_back((s.vy - vy_prev) / dt + s.vx * s.r);
            data.steering.push_back(input.steering * 180.0 / M_PI);
            data.throttle.push_back(input.throttle);
            data.brake.push_back(input.brake);
            data.gear.push_back(input.gear);
            data.engine_rpm.push_back(vehicle.get_engine_rpm(input.gear));
            data.engine_torque.push_back(vehicle.get_engine_torque(input.gear, input.throttle));
            data.omega_RL.push_back(s.omega[2]);
            data.omega_RR.push_back(s.omega[3]);
            data.omega_diff.push_back(s.omega[2] - s.omega[3]);
            data.Fz.push_back(vehicle.get_tire_Fz());
            data.slip_angle.push_back(tire_slip_angle_deg(vehicle));
        }
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        vehicle.step(dt, input);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    auto& s = vehicle.state();
    spdlog::info("Simulation complete:");
    spdlog::info("  Duration: {:.2f} s", t_end);
    spdlog::info("  Data points: {}", data.time.size());
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final gear: {}", current_gear);
    spdlog::info("  Final speed: {:.1f} m/s ({:.1f} km/h)", s.vx, s.vx * 3.6);
    spdlog::info("  Distance: {:.1f} m", s.x);
    spdlog::info("  Final engine RPM: {:.0f} rpm", vehicle.get_engine_rpm(current_gear));
    
    // 75m加速計測（FSAE標準）
    double t_75m = -1.0;
    for (size_t i = 0; i < data.x.size(); ++i) {
        if (data.x[i] >= 75.0) {
            t_75m = data.time[i];
            spdlog::info("  75m acceleration: {:.2f} s", t_75m);
            break;
        }
    }
    
    data.save_csv(ensure_output_path("acceleration_result.csv"));
}

/// 旋回加速試験（LSD効果評価）
void run_cornering_acceleration_test(double pinn_eval_rate_hz) {
    spdlog::info("=== Cornering Acceleration Test (LSD Effect) ===")
;
    
    auto params = VehicleParams::default_fsae();
    params.tire.pinn_eval_rate_hz = pinn_eval_rate_hz;
    Vehicle14DOF_New vehicle(params);
    
    // 初速 10 m/s
    vehicle.set_initial_velocity(10.0);
    vehicle.state().r = 0.5;  // 初期ヨーレート（旋回中）
    
    const double t_end = 3.0;
    const double dt = 0.001;
    
    SimData data;
    data.reserve(static_cast<size_t>(t_end / dt) + 1);
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    while (vehicle.time() < t_end) {
        auto& s = vehicle.state();
        
        // 入力（旋回しながら加速）
        DriverInput input;
        input.throttle = std::min(vehicle.time() / 1.0, 1.0);  // 1秒かけて全開
        input.steering = 0.15;  // 約8.6度の舵角
        input.gear = 2;  // 2速
        
        // データ保存（10ms毎）
        if (static_cast<int>(vehicle.time() * 1000) % 10 == 0) {
            data.time.push_back(vehicle.time());
            data.x.push_back(s.x);
            data.y.push_back(s.y);
            data.psi.push_back(s.psi * 180.0 / M_PI);
            data.vx.push_back(s.vx);
            data.vy.push_back(s.vy);
            data.r.push_back(s.r * 180.0 / M_PI);
            data.ax.push_back((s.vx - vx_prev) / dt);
            data.ay.push_back((s.vy - vy_prev) / dt + s.vx * s.r);
            data.steering.push_back(input.steering * 180.0 / M_PI);
            data.throttle.push_back(input.throttle);
            data.brake.push_back(input.brake);
            data.gear.push_back(input.gear);
            data.engine_rpm.push_back(vehicle.get_engine_rpm(input.gear));
            data.engine_torque.push_back(vehicle.get_engine_torque(input.gear, input.throttle));
            data.omega_RL.push_back(s.omega[2]);
            data.omega_RR.push_back(s.omega[3]);
            data.omega_diff.push_back(s.omega[2] - s.omega[3]);
            data.Fz.push_back(vehicle.get_tire_Fz());
            data.slip_angle.push_back(tire_slip_angle_deg(vehicle));
        }
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        vehicle.step(dt, input);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    auto& s = vehicle.state();
    spdlog::info("Simulation complete:");
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final speed: {:.1f} m/s ({:.1f} km/h)", s.vx, s.vx * 3.6);
    spdlog::info("  Final yaw rate: {:.2f} deg/s", s.r * 180.0 / M_PI);
    spdlog::info("  Final wheel speed diff: {:.2f} rad/s", s.omega[2] - s.omega[3]);
    
    data.save_csv(ensure_output_path("cornering_acceleration_result.csv"));
}

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("FSAE 14DOF Vehicle Dynamics Simulator v0.2.0");
    spdlog::info("  with Engine Torque Map & Salisbury LSD");
    
    std::string test_type = (argc > 1) ? argv[1] : "acceleration";
    double pinn_eval_rate_hz = VehicleParams::default_fsae().tire.pinn_eval_rate_hz;
    if (argc > 2) {
        if (argc != 4 || std::string(argv[2]) != "--pinn-eval-rate-hz") {
            spdlog::error(
                "Usage: fsae_14dof [step_steer|circle|acceleration|cornering] "
                "[--pinn-eval-rate-hz HZ]");
            return 2;
        }
        try {
            std::size_t parsed = 0;
            const std::string value = argv[3];
            pinn_eval_rate_hz = std::stod(value, &parsed);
            if (parsed != value.size() || !std::isfinite(pinn_eval_rate_hz) ||
                pinn_eval_rate_hz < 0.0) {
                throw std::invalid_argument("invalid PINN evaluation rate");
            }
        } catch (const std::exception&) {
            spdlog::error("PINN evaluation rate must be a finite non-negative number");
            return 2;
        }
    }
    spdlog::info("PINN evaluation rate: {} Hz (0 = every RK4/K&C evaluation)",
                 pinn_eval_rate_hz);
    
    if (test_type == "step_steer") {
        run_step_steer_test(pinn_eval_rate_hz);
    } else if (test_type == "circle") {
        run_steady_state_circle_test(pinn_eval_rate_hz);
    } else if (test_type == "acceleration") {
        run_acceleration_test(pinn_eval_rate_hz);
    } else if (test_type == "cornering") {
        run_cornering_acceleration_test(pinn_eval_rate_hz);
    } else {
        spdlog::info("Available tests: step_steer, circle, acceleration, cornering");
    }
    
    return 0;
}
