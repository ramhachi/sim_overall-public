// src/test_8dof.cpp
/// 8DOFビークルモデル テスト

#include <iostream>
#include <fstream>
#include <cmath>
#include "fsae/vehicle/vehicle_8dof.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== 8DOF Vehicle Model Test ===");
    
    auto params = VehicleParams::default_fsae();
    Vehicle8DOF vehicle(params);
    
    spdlog::info("Equilibrium positions:");
    spdlog::info("  z_sprung = {:.4f} m", vehicle.z_equilibrium());
    spdlog::info("  z_u[FL] = {:.4f} m", vehicle.state().z_u[0]);
    
    // 初期速度 15 m/s
    vehicle.set_initial_velocity(15.0);
    
    const double dt = 0.001;
    
    // テスト1: 平衡テスト（静止）
    spdlog::info("\n--- Test 1: Static Equilibrium ---");
    vehicle.reset();
    for (int step = 0; step < 100; ++step) {
        DriverInput input{};
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        if (step % 20 == 0) {
            spdlog::info("t={:.3f}: z={:.5f}, z_u[0]={:.5f}, vz={:.6f}", 
                        vehicle.time(), s.z, s.z_u[0], s.vz);
        }
        
        if (std::isnan(s.z) || std::abs(s.z) > 1 || std::abs(s.vz) > 10) {
            spdlog::error("Static test DIVERGED at t={:.3f}", vehicle.time());
            return 1;
        }
    }
    spdlog::info("Static test PASSED");
    
    // テスト2: 速度テスト
    spdlog::info("\n--- Test 2: Velocity (1s) ---");
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);
    
    for (int step = 0; step < 1000; ++step) {
        DriverInput input{};
        input.throttle = 0.3;
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        if (step % 200 == 0) {
            spdlog::info("t={:.3f}: vx={:.2f}, z={:.5f}, theta={:.3f}°", 
                        vehicle.time(), s.vx, s.z, s.theta * 180/M_PI);
        }
        
        if (std::isnan(s.vx) || std::abs(s.z) > 1 || std::abs(s.theta) > 0.5) {
            spdlog::error("Velocity test DIVERGED at t={:.3f}", vehicle.time());
            spdlog::error("  vx={:.2f}, z={:.5f}, theta={:.3f}°", 
                         s.vx, s.z, s.theta * 180/M_PI);
            return 1;
        }
    }
    spdlog::info("Velocity test PASSED");
    
    // テスト3: ステアリングテスト
    spdlog::info("\n--- Test 3: Steering (3s) ---");
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);
    
    for (int step = 0; step < 3000; ++step) {
        DriverInput input{};
        input.throttle = 0.3;
        input.steering = (vehicle.time() >= 1.0) ? 0.1 : 0.0;
        
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        if (step % 500 == 0) {
            spdlog::info("t={:.3f}: vx={:.2f}, vy={:.2f}, r={:.1f}°/s, phi={:.2f}°", 
                        vehicle.time(), s.vx, s.vy, s.r * 180/M_PI, s.phi * 180/M_PI);
        }
        
        if (std::isnan(s.vx) || std::abs(s.z) > 1 || std::abs(s.phi) > 0.5) {
            spdlog::error("Steering test DIVERGED at t={:.3f}", vehicle.time());
            return 1;
        }
    }
    spdlog::info("Steering test PASSED");
    
    spdlog::info("\n=== All 8DOF tests PASSED ===");
    return 0;
}
