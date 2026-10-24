// src/test_arb_debug.cpp
/// ARB デバッグテスト

#include <iostream>
#include <cmath>
#include "fsae/vehicle/vehicle_8dof.hpp"
#include "fsae/vehicle/vehicle_8dof_arb.hpp"
#include <spdlog/spdlog.h>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::debug);
    spdlog::info("=== ARB Debug Test ===");
    
    auto params = VehicleParams::default_fsae();
    
    // ARB剛性を設定
    params.suspension.k_arb_f = 5000.0;
    params.suspension.k_arb_r = 3000.0;
    
    spdlog::info("=== Parameters ===");
    spdlog::info("k_arb_f = {} Nm/rad", params.suspension.k_arb_f);
    spdlog::info("k_arb_r = {} Nm/rad", params.suspension.k_arb_r);
    spdlog::info("track_f = {} m", params.mass.track_f);
    spdlog::info("track_r = {} m", params.mass.track_r);
    spdlog::info("Ixx = {} kg*m^2", params.mass.Ixx);
    
    Vehicle8DOF vehicle_no_arb(params);
    Vehicle8DOF_ARB vehicle_with_arb(params);
    
    // 初期速度
    const double v0 = 15.0;
    vehicle_no_arb.set_initial_velocity(v0);
    vehicle_with_arb.set_initial_velocity(v0);
    
    const double dt = 0.001;
    const double step_steer = 0.15;
    
    spdlog::info("\n=== Simulation ===");
    
    for (int step = 0; step < 2500; ++step) {
        double t = step * dt;
        
        DriverInput input{};
        input.throttle = 0.3;
        input.steering = (t >= 1.0) ? step_steer : 0.0;
        
        vehicle_no_arb.step(dt, input);
        vehicle_with_arb.step(dt, input);
        
        auto& s1 = vehicle_no_arb.state();
        auto& s2 = vehicle_with_arb.state();
        
        // ステアリング入力直後にデバッグ出力
        if (step >= 1000 && step <= 1010) {
            auto [arb_f, arb_r] = vehicle_with_arb.get_arb_forces();
            spdlog::info("t={:.3f}s:", t);
            spdlog::info("  no-ARB:   phi={:.4f}° p={:.4f}°/s", s1.phi * 180/M_PI, s1.p * 180/M_PI);
            spdlog::info("  with-ARB: phi={:.4f}° p={:.4f}°/s", s2.phi * 180/M_PI, s2.p * 180/M_PI);
            spdlog::info("  ARB torque: F={:.2f} Nm, R={:.2f} Nm", arb_f, arb_r);
        }
        
        // 定期的なサマリー
        if (step % 500 == 0 && step > 0) {
            auto [arb_f, arb_r] = vehicle_with_arb.get_arb_forces();
            spdlog::info("t={:.1f}s: Roll no-ARB={:.3f}°, with-ARB={:.3f}°, ARB_F={:.1f}Nm, ARB_R={:.1f}Nm",
                        t, s1.phi * 180/M_PI, s2.phi * 180/M_PI, arb_f, arb_r);
        }
        
        if (std::isnan(s1.vx) || std::isnan(s2.vx)) {
            spdlog::error("Simulation diverged at t={:.3f}s!", t);
            return 1;
        }
    }
    
    auto& s1 = vehicle_no_arb.state();
    auto& s2 = vehicle_with_arb.state();
    
    spdlog::info("\n=== Final Results ===");
    spdlog::info("Roll no-ARB:   {:.3f}°", s1.phi * 180/M_PI);
    spdlog::info("Roll with-ARB: {:.3f}°", s2.phi * 180/M_PI);
    
    double reduction = (1 - std::abs(s2.phi) / std::abs(s1.phi + 1e-6)) * 100;
    spdlog::info("Reduction: {:.1f}%", reduction);
    
    return 0;
}
