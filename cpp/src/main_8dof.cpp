/// @file main_8dof.cpp
/// @brief 8DOFビークルモデル デモ

#include <iostream>
#include <fstream>
#include <cmath>
#include <iomanip>
#include "fsae/vehicle/vehicle_8dof.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main(int, char**) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("FSAE 8DOF Vehicle Dynamics Simulator");
    
    auto params = VehicleParams::default_fsae();
    Vehicle8DOF vehicle(params);
    
    spdlog::info("Equilibrium z_sprung: {:.4f} m", vehicle.z_equilibrium());
    
    // 初期速度設定
    const double v0 = 15.0;
    vehicle.set_initial_velocity(v0);
    
    // シミュレーション設定
    const double dt = 0.001;
    const double t_end = 5.0;
    const double step_time = 1.0;
    const double step_steer = 0.1;
    const double throttle = 0.3;
    
    // CSV出力
    std::ofstream csv("8dof_step_steer_result.csv");
    csv << std::fixed << std::setprecision(6);
    csv << "time,x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,"
        << "z_u0,z_u1,z_u2,z_u3,vz_u0,vz_u1,vz_u2,vz_u3\n";
    
    bool diverged = false;
    int step_count = 0;
    
    while (vehicle.time() < t_end && !diverged) {
        auto& s = vehicle.state();
        
        // CSV出力
        csv << vehicle.time() << ","
            << s.x << "," << s.y << "," << s.z << ","
            << (s.phi * 180/M_PI) << "," << (s.theta * 180/M_PI) << "," << (s.psi * 180/M_PI) << ","
            << s.vx << "," << s.vy << "," << s.vz << ","
            << (s.p * 180/M_PI) << "," << (s.q * 180/M_PI) << "," << (s.r * 180/M_PI) << ","
            << s.z_u[0] << "," << s.z_u[1] << "," << s.z_u[2] << "," << s.z_u[3] << ","
            << s.vz_u[0] << "," << s.vz_u[1] << "," << s.vz_u[2] << "," << s.vz_u[3] << "\n";
        
        // 発散チェック
        if (std::isnan(s.vx) || std::abs(s.z) > 10 || std::abs(s.theta) > M_PI) {
            spdlog::error("Simulation diverged at t = {:.3f} s", vehicle.time());
            diverged = true;
            break;
        }
        
        // 入力
        DriverInput input{};
        input.throttle = throttle;
        input.steering = (vehicle.time() >= step_time) ? step_steer : 0.0;
        
        vehicle.step(dt, input);
        step_count++;
    }
    
    csv.close();
    
    auto& s = vehicle.state();
    spdlog::info("=== 8DOF Simulation {} ===", diverged ? "FAILED" : "Complete");
    spdlog::info("  Duration: {:.2f} s", vehicle.time());
    spdlog::info("  Data points: {}", step_count);
    spdlog::info("  Final position: x={:.1f} m, y={:.1f} m", s.x, s.y);
    spdlog::info("  Final speed: {:.1f} m/s", s.speed());
    spdlog::info("  Final yaw rate: {:.1f} deg/s", s.r * 180/M_PI);
    spdlog::info("Saved to 8dof_step_steer_result.csv");
    
    return diverged ? 1 : 0;
}
