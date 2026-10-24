// src/test_6dof_equilibrium.cpp
/// 6DOF平衡状態のデバッグ

#include <iostream>
#include <cmath>
#include "fsae/vehicle/vehicle_6dof.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== 6DOF Equilibrium Test ===");
    
    auto params = VehicleParams::default_fsae();
    Vehicle6DOF vehicle(params);
    
    // 初期速度なし（静止状態）
    vehicle.state().vx = 0;
    for (int i = 0; i < 4; ++i) {
        vehicle.state().omega[i] = 0;
    }
    
    spdlog::info("Initial state:");
    spdlog::info("  z = {:.6f} m (equilibrium = {:.6f})", vehicle.state().z, vehicle.z_equilibrium());
    spdlog::info("  vx={:.2f}, vy={:.2f}, vz={:.6f}", vehicle.state().vx, vehicle.state().vy, vehicle.state().vz);
    spdlog::info("  phi={:.4f}, theta={:.4f}, psi={:.4f} deg", 
                 vehicle.state().phi * 180/M_PI, vehicle.state().theta * 180/M_PI, vehicle.state().psi * 180/M_PI);
    
    // 10ステップ進めて変化を観察
    const double dt = 0.001;
    DriverInput input{};  // ゼロ入力
    
    for (int step = 0; step < 100; ++step) {
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        
        if (step < 10 || step % 10 == 0) {
            spdlog::info("t={:.3f}: z={:.6f}, vz={:.6f}, theta={:.4f}°, q={:.6f}", 
                        vehicle.time(), s.z, s.vz, s.theta * 180/M_PI, s.q);
        }
        
        if (std::isnan(s.z) || std::abs(s.z) > 1 || std::abs(s.theta) > 0.5) {
            spdlog::error("DIVERGED at t={:.3f}", vehicle.time());
            break;
        }
    }
    
    spdlog::info("Final state after 0.1s:");
    auto& s = vehicle.state();
    spdlog::info("  z={:.6f} m (delta={:.6f})", s.z, s.z - vehicle.z_equilibrium());
    spdlog::info("  vz={:.6f} m/s", s.vz);
    spdlog::info("  theta={:.4f}° q={:.6f} rad/s", s.theta * 180/M_PI, s.q);
    
    return 0;
}
