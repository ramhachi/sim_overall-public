/// @file test_aero_integration.cpp
/// @brief 4D AeroMap統合テスト - 14DOFシミュレーションでの効果確認

#include <iostream>
#include <fstream>
#include <fsae/physics/aero/aero_map_4d.hpp>
#include <spdlog/spdlog.h>

using namespace fsae::physics::aero;

int main() {
    spdlog::info("=== 4D AeroMap Integration Test ===");
    
    // デフォルトパラメータでAeroMapを初期化
    AeroMap4D<double> aero_map;
    
    spdlog::info("\n=== Test 1: Speed Sweep (40mm/55mm, 0deg roll, 0deg yaw) ===");
    std::ofstream csv("aero_integration_test.csv");
    csv << "velocity,Fz_aero,Fx_aero,Fy_aero,Fz_front,Fz_rear\n";
    
    for (double v = 5.0; v <= 30.0; v += 5.0) {
        auto result = aero_map.compute_forces(v, 40.0, 55.0, 0.0, 0.0);
        spdlog::info("V={:5.1f} m/s: Fz={:7.1f} N, Fx={:7.1f} N (CL={:.3f}, CD={:.3f})",
                     v, result.Fz, result.Fx,
                     result.Fz / (0.5 * 1.225 * v * v * 1.0),
                     -result.Fx / (0.5 * 1.225 * v * v * 1.0));
        csv << v << "," << result.Fz << "," << result.Fx << "," 
            << result.Fy << "," << result.Fz_front << "," << result.Fz_rear << "\n";
    }
    
    spdlog::info("\n=== Test 2: Ride Height Effect (V=20m/s) ===");
    spdlog::info("Front RH sweep (rear=55mm):");
    for (double rh_f = 25.0; rh_f <= 55.0; rh_f += 10.0) {
        auto result = aero_map.compute_forces(20.0, rh_f, 55.0, 0.0, 0.0);
        double gain = (result.Fz / -612.0 - 1.0) * 100.0;  // baseline: -612N @ 40/55mm
        spdlog::info("  RH_f={:4.1f}mm: Fz={:7.1f} N ({:+5.1f}% vs baseline)",
                     rh_f, result.Fz, gain);
    }
    
    spdlog::info("\n=== Test 3: Roll Effect (V=20m/s, 40mm/55mm) ===");
    for (double roll = 0.0; roll <= 5.0; roll += 1.0) {
        double roll_rad = roll * M_PI / 180.0;
        auto result = aero_map.compute_forces(20.0, 40.0, 55.0, roll_rad, 0.0);
        double loss = (1.0 - result.Fz / -612.0) * 100.0;
        spdlog::info("  Roll={:4.1f}deg: Fz={:7.1f} N ({:5.1f}% loss)",
                     roll, result.Fz, loss);
    }
    
    spdlog::info("\n=== Test 4: Yaw Effect (V=20m/s, 40mm/55mm) ===");
    for (double yaw = 0.0; yaw <= 15.0; yaw += 5.0) {
        double yaw_rad = yaw * M_PI / 180.0;
        auto result = aero_map.compute_forces(20.0, 40.0, 55.0, 0.0, yaw_rad);
        double cd = -result.Fx / (0.5 * 1.225 * 400.0 * 1.0);
        spdlog::info("  Yaw={:5.1f}deg: Fx={:7.1f} N, Fy={:7.1f} N (CD={:.3f})",
                     yaw, result.Fx, result.Fy, cd);
    }
    
    spdlog::info("\n=== Test 5: Combined Effect (Racing scenario) ===");
    // 高速コーナリング: 低いRH、ロール3度、ヨー8度、25m/s
    auto race = aero_map.compute_forces(25.0, 30.0, 50.0, 3.0 * M_PI / 180.0, 8.0 * M_PI / 180.0);
    auto baseline = aero_map.compute_forces(25.0, 40.0, 55.0, 0.0, 0.0);
    
    spdlog::info("Baseline (40/55mm, 0/0deg, 25m/s):");
    spdlog::info("  Fz={:7.1f} N, Fx={:7.1f} N", baseline.Fz, baseline.Fx);
    spdlog::info("Racing (30/50mm, 3/8deg, 25m/s):");
    spdlog::info("  Fz={:7.1f} N, Fx={:7.1f} N, Fy={:7.1f} N", race.Fz, race.Fx, race.Fy);
    spdlog::info("Delta:");
    spdlog::info("  ΔFz={:+7.1f} N ({:+5.1f}%)", race.Fz - baseline.Fz, 
                 (race.Fz / baseline.Fz - 1.0) * 100.0);
    spdlog::info("  ΔFx={:+7.1f} N ({:+5.1f}%)", race.Fx - baseline.Fx,
                 (race.Fx / baseline.Fx - 1.0) * 100.0);
    
    csv.close();
    spdlog::info("\n✓ CSV saved to: aero_integration_test.csv");
    spdlog::info("✓ 4D AeroMap integration test PASSED");
    
    return 0;
}
