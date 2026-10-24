// src/main_6dof.cpp
/// 6DOFビークルモデル デモ

#include <iostream>
#include <fstream>
#include <chrono>
#include <cmath>

#include "fsae/vehicle/vehicle_6dof.hpp"

#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("FSAE 6DOF Vehicle Dynamics Simulator");
    
    // パラメータ
    auto params = VehicleParams::default_fsae();
    Vehicle6DOF vehicle(params);
    
    spdlog::info("Equilibrium z position: {:.4f} m", vehicle.z_equilibrium());
    
    // 初期速度 15 m/s
    vehicle.set_initial_velocity(15.0);
    
    // シミュレーション設定
    const double t_end = 5.0;
    const double dt = 0.001;
    const double step_time = 1.0;
    const double step_steer = 0.1;  // ~5.7 deg
    
    // CSV出力
    std::ofstream csv("6dof_step_steer_result.csv");
    csv << "time,x,y,z,phi,theta,psi,vx,vy,vz,p,q,r,ax,ay\n";
    
    double vx_prev = vehicle.state().vx;
    double vy_prev = vehicle.state().vy;
    
    auto start = std::chrono::high_resolution_clock::now();
    
    int output_count = 0;
    bool diverged = false;
    
    while (vehicle.time() < t_end) {
        DriverInput input;
        input.throttle = 0.3;
        input.steering = (vehicle.time() >= step_time) ? step_steer : 0.0;
        
        auto& s = vehicle.state();
        double ax = (s.vx - vx_prev) / dt;
        double ay = (s.vy - vy_prev) / dt + s.vx * s.r;
        
        csv << vehicle.time() << ","
            << s.x << "," << s.y << "," << s.z << ","
            << s.phi * 180.0 / M_PI << ","
            << s.theta * 180.0 / M_PI << ","
            << s.psi * 180.0 / M_PI << ","
            << s.vx << "," << s.vy << "," << s.vz << ","
            << s.p * 180.0 / M_PI << ","
            << s.q * 180.0 / M_PI << ","
            << s.r * 180.0 / M_PI << ","
            << ax << "," << ay << "\n";
        
        vx_prev = s.vx;
        vy_prev = s.vy;
        
        vehicle.step(dt, input);
        output_count++;
        
        // 発散チェック
        if (std::isnan(s.vx) || std::abs(s.vx) > 1000 || 
            std::abs(s.z) > 10 || std::abs(s.phi) > M_PI) {
            spdlog::error("Simulation diverged at t = {:.3f} s", vehicle.time());
            spdlog::error("  vx={:.2f}, z={:.4f}, phi={:.2f}deg", 
                         s.vx, s.z, s.phi * 180.0 / M_PI);
            diverged = true;
            break;
        }
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    csv.close();
    
    auto& s = vehicle.state();
    spdlog::info("=== 6DOF Simulation {} ===", diverged ? "FAILED" : "Complete");
    spdlog::info("  Duration: {:.2f} s", vehicle.time());
    spdlog::info("  Data points: {}", output_count);
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final position: x={:.1f} m, y={:.1f} m, z={:.4f} m", s.x, s.y, s.z);
    spdlog::info("  Final attitude: φ={:.2f}°, θ={:.2f}°, ψ={:.1f}°", 
                 s.phi * 180.0 / M_PI, s.theta * 180.0 / M_PI, s.psi * 180.0 / M_PI);
    spdlog::info("  Final speed: {:.1f} m/s", s.speed());
    spdlog::info("  Final yaw rate: {:.1f} deg/s", s.r * 180.0 / M_PI);
    spdlog::info("Saved to 6dof_step_steer_result.csv");
    
    return diverged ? 1 : 0;
}
