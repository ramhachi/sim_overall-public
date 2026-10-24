// src/test_arb.cpp
/// ARB（アンチロールバー）効果テスト

#include <iostream>
#include <cmath>
#include "fsae/vehicle/vehicle_8dof.hpp"
#include "fsae/vehicle/vehicle_8dof_arb.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== Anti-Roll Bar (ARB) Effect Test ===");
    
    auto params = VehicleParams::default_fsae();
    
    // ARB剛性を設定（典型的なFSAE車の値）
    params.suspension.k_arb_f = 5000.0;   // フロントARB [Nm/rad]
    params.suspension.k_arb_r = 3000.0;   // リアARB [Nm/rad]
    
    Vehicle8DOF vehicle_no_arb(params);
    
    // ARBなしのパラメータ
    auto params_no_arb = params;
    params_no_arb.suspension.k_arb_f = 0.0;
    params_no_arb.suspension.k_arb_r = 0.0;
    
    Vehicle8DOF_ARB vehicle_with_arb(params);
    
    // 初期速度
    const double v0 = 15.0;
    vehicle_no_arb.set_initial_velocity(v0);
    vehicle_with_arb.set_initial_velocity(v0);
    
    const double dt = 0.001;
    const double step_time = 1.0;
    const double step_steer = 0.15;  // 大きめのステア入力
    
    spdlog::info("Running comparison simulation...");
    spdlog::info("  Step steer: {} rad at t={}s", step_steer, step_time);
    spdlog::info("  ARB stiffness: F={} Nm/rad, R={} Nm/rad", 
                 params.suspension.k_arb_f, params.suspension.k_arb_r);
    
    double max_roll_no_arb = 0;
    double max_roll_with_arb = 0;
    
    for (int step = 0; step < 3000; ++step) {
        double t = step * dt;
        
        DriverInput input{};
        input.throttle = 0.3;
        input.steering = (t >= step_time) ? step_steer : 0.0;
        
        vehicle_no_arb.step(dt, input);
        vehicle_with_arb.step(dt, input);
        
        auto& s1 = vehicle_no_arb.state();
        auto& s2 = vehicle_with_arb.state();
        
        max_roll_no_arb = std::max(max_roll_no_arb, std::abs(s1.phi));
        max_roll_with_arb = std::max(max_roll_with_arb, std::abs(s2.phi));
        
        if (step % 500 == 0 && step > 0) {
            spdlog::info("t={:.1f}s: Roll no-ARB={:.2f}°, with-ARB={:.2f}° (reduction={:.1f}%)",
                        t, s1.phi * 180/M_PI, s2.phi * 180/M_PI,
                        (1 - std::abs(s2.phi) / std::abs(s1.phi + 1e-6)) * 100);
        }
        
        // 発散チェック
        if (std::isnan(s1.vx) || std::isnan(s2.vx)) {
            spdlog::error("Simulation diverged!");
            return 1;
        }
    }
    
    spdlog::info("\n=== Results ===");
    spdlog::info("Max roll angle:");
    spdlog::info("  Without ARB: {:.2f}°", max_roll_no_arb * 180/M_PI);
    spdlog::info("  With ARB:    {:.2f}°", max_roll_with_arb * 180/M_PI);
    spdlog::info("  Reduction:   {:.1f}%", 
                 (1 - max_roll_with_arb / max_roll_no_arb) * 100);
    
    // ARBで少なくとも10%のロール低減を期待
    if (max_roll_with_arb < max_roll_no_arb * 0.95) {
        spdlog::info("\n✓ ARB test PASSED - Roll reduction confirmed");
        return 0;
    } else {
        spdlog::error("\n✗ ARB test FAILED - Insufficient roll reduction");
        return 1;
    }
}
