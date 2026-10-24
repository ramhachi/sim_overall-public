// src/main_3dof.cpp
/// 3DOFプレーナービークルモデル デモ（デバッグ用）

#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>

#include "fsae/vehicle/vehicle_3dof.hpp"

#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("FSAE 3DOF Vehicle Dynamics Simulator (Debug Version)");
    
    // パラメータ
    auto params = VehicleParams::default_fsae();
    Vehicle3DOF vehicle(params);
    
    // 初期速度 15 m/s
    vehicle.set_initial_velocity(15.0);
    
    // シミュレーション設定
    const double t_end = 5.0;
    const double dt = 0.001;
    const double step_time = 1.0;
    const double step_steer = 0.1;  // ~5.7 deg
    
    // CSV出力用
    std::ofstream csv("3dof_step_steer_result.csv");
    csv << "time,x,y,psi,vx,vy,r,ax,ay,steering\n";
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    int output_count = 0;
    while (vehicle.time() < t_end) {
        // 入力
        DriverInput input;
        input.throttle = 0.3;
        input.steering = (vehicle.time() >= step_time) ? step_steer : 0.0;
        
        // データ保存（1ms毎）
        auto& s = vehicle.state();
        double ax = (s.vx - vx_prev) / dt;
        double ay = (s.vy - vy_prev) / dt + s.vx * s.r;
        
        csv << vehicle.time() << ","
            << s.x << "," << s.y << ","
            << s.psi * 180.0 / M_PI << ","
            << s.vx << "," << s.vy << ","
            << s.r * 180.0 / M_PI << ","
            << ax << "," << ay << ","
            << input.steering * 180.0 / M_PI << "\n";
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        // 積分
        vehicle.step(dt, input);
        output_count++;
        
        // 発散チェック
        if (std::isnan(s.vx) || std::abs(s.vx) > 1000) {
            spdlog::error("Simulation diverged at t = {:.3f} s", vehicle.time());
            break;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    csv.close();
    
    auto& s = vehicle.state();
    spdlog::info("=== 3DOF Simulation Complete ===");
    spdlog::info("  Duration: {:.2f} s", vehicle.time());
    spdlog::info("  Data points: {}", output_count);
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final position: x={:.1f} m, y={:.1f} m", s.x, s.y);
    spdlog::info("  Final yaw: {:.1f} deg", s.psi * 180.0 / M_PI);
    spdlog::info("  Final speed: {:.1f} m/s", s.speed());
    spdlog::info("  Final yaw rate: {:.1f} deg/s", s.r * 180.0 / M_PI);
    spdlog::info("Saved to 3dof_step_steer_result.csv");
    
    return 0;
}
