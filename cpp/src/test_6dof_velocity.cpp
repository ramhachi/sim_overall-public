// src/test_6dof_velocity.cpp
/// 6DOF速度状態でのデバッグ

#include <iostream>
#include <cmath>
#include "fsae/vehicle/vehicle_6dof.hpp"
#include "fsae/math/rotation.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== 6DOF Velocity Test ===");
    
    auto params = VehicleParams::default_fsae();
    Vehicle6DOF vehicle(params);
    
    // 初期速度 15 m/s
    vehicle.set_initial_velocity(15.0);
    
    spdlog::info("Initial state:");
    spdlog::info("  vx={:.2f} m/s, wheel omega={:.2f} rad/s", 
                 vehicle.state().vx, vehicle.state().omega[0]);
    
    const double dt = 0.001;
    
    // テスト1: ゼロ入力で慣性走行（1秒間）
    spdlog::info("\n--- Test 1: Coasting (zero throttle) for 1s ---");
    for (int step = 0; step < 1000; ++step) {
        DriverInput input{};  // ゼロ入力
        input.throttle = 0;
        input.steering = 0;
        
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        if (step < 10 || step % 100 == 0) {
            spdlog::info("t={:.3f}: vx={:.3f}, z={:.6f}, theta={:.3f}°", 
                        vehicle.time(), s.vx, s.z, s.theta * 180/M_PI);
        }
        
        if (std::isnan(s.vx) || std::abs(s.z) > 1 || std::abs(s.theta) > 0.5) {
            spdlog::error("DIVERGED at t={:.3f}", vehicle.time());
            spdlog::error("  vx={:.3f}, z={:.6f}, theta={:.3f}°", s.vx, s.z, s.theta * 180/M_PI);
            return 1;
        }
    }
    spdlog::info("Coasting test PASSED");
    
    // リセットしてテスト2
    vehicle.reset();
    vehicle.set_initial_velocity(15.0);
    
    spdlog::info("\n--- Test 2: With throttle (0.3) ---");
    for (int step = 0; step < 100; ++step) {
        DriverInput input{};
        input.throttle = 0.3;
        input.steering = 0;
        
        vehicle.step(dt, input);
        
        auto& s = vehicle.state();
        if (step < 10 || step % 20 == 0) {
            spdlog::info("t={:.3f}: vx={:.3f}, z={:.6f}, theta={:.3f}°", 
                        vehicle.time(), s.vx, s.z, s.theta * 180/M_PI);
        }
        
        if (std::isnan(s.vx) || std::abs(s.z) > 1 || std::abs(s.theta) > 0.5) {
            spdlog::error("DIVERGED at t={:.3f}", vehicle.time());
            spdlog::error("  vx={:.3f}, z={:.6f}, theta={:.3f}°", s.vx, s.z, s.theta * 180/M_PI);
            return 1;
        }
    }
    spdlog::info("Throttle test PASSED");
    
    spdlog::info("\n=== All tests PASSED ===");
    return 0;
}
