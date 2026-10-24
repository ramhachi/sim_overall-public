/// @file test_aero_map_4d.cpp
/// @brief 4Dエアロマップのテスト

#include <fsae/physics/aero/aero_map_4d.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <fstream>

using namespace fsae::physics::aero;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== 4D Aero Map Test ===");
    
    // エアロマップ作成
    AeroMapParams<double> params;
    params.rho = 1.225;
    params.A_ref = 1.0;
    params.L_ref = 1.6;
    params.x_cg = 0.8;
    params.CL_default = -2.5;
    params.CD_default = 1.0;
    
    AeroMap4D<double> aero_map(params);
    
    // グリッド情報
    const auto& grid = aero_map.grid();
    spdlog::info("Grid dimensions:");
    spdlog::info("  RH_front: {} points ({}-{} mm)", 
        grid.rh_f.size(), grid.rh_f.front(), grid.rh_f.back());
    spdlog::info("  RH_rear:  {} points ({}-{} mm)", 
        grid.rh_r.size(), grid.rh_r.front(), grid.rh_r.back());
    spdlog::info("  Roll:     {} points ({:.1f}-{:.1f} deg)", 
        grid.roll.size(), grid.roll.front()*180/M_PI, grid.roll.back()*180/M_PI);
    spdlog::info("  Yaw:      {} points ({:.1f}-{:.1f} deg)", 
        grid.yaw.size(), grid.yaw.front()*180/M_PI, grid.yaw.back()*180/M_PI);
    
    // === テスト1: ベースライン ===
    spdlog::info("\n=== Test 1: Baseline Condition ===");
    double rh_f = 40, rh_r = 55, roll = 0, yaw = 0;
    double velocity = 20;  // m/s
    
    auto coef = aero_map.compute(rh_f, rh_r, roll, yaw);
    auto forces = aero_map.compute_forces(velocity, rh_f, rh_r, roll, yaw);
    
    spdlog::info("RH_f={:.0f}mm, RH_r={:.0f}mm, roll={:.1f}deg, yaw={:.1f}deg, V={:.0f}m/s",
        rh_f, rh_r, roll*180/M_PI, yaw*180/M_PI, velocity);
    spdlog::info("Coefficients: CL={:.3f}, CD={:.3f}, CY={:.3f}", coef.CL, coef.CD, coef.CY);
    spdlog::info("CoP_x={:.3f}m, CoP_dist={:.2f}", coef.CoP_x, coef.CoP_dist);
    spdlog::info("Forces: Fz={:.0f}N, Fx={:.0f}N, Fy={:.0f}N", forces.Fz, forces.Fx, forces.Fy);
    spdlog::info("  Fz_front={:.0f}N, Fz_rear={:.0f}N", forces.Fz_front, forces.Fz_rear);
    
    // === テスト2: ライドハイト効果 ===
    spdlog::info("\n=== Test 2: Ride Height Effect ===");
    std::vector<double> rh_test = {25, 35, 45, 55};
    for (double rh : rh_test) {
        auto c = aero_map.compute(rh, 55, 0, 0);
        auto f = aero_map.compute_forces(velocity, rh, 55, 0, 0);
        spdlog::info("RH_f={:.0f}mm: CL={:.3f}, Fz={:.0f}N", rh, c.CL, f.Fz);
    }
    
    // === テスト3: ロール効果 ===
    spdlog::info("\n=== Test 3: Roll Effect ===");
    std::vector<double> roll_test = {0, 0.044, 0.087};  // 0, 2.5, 5 deg
    for (double r : roll_test) {
        auto c = aero_map.compute(40, 55, r, 0);
        spdlog::info("Roll={:.1f}deg: CL={:.3f} (loss={:.1f}%)", 
            r*180/M_PI, c.CL, (1 + c.CL/coef.CL)*100);
    }
    
    // === テスト4: ヨー効果 ===
    spdlog::info("\n=== Test 4: Yaw Effect ===");
    std::vector<double> yaw_test = {0, 0.087, 0.175, 0.262};  // 0, 5, 10, 15 deg
    for (double y : yaw_test) {
        auto c = aero_map.compute(40, 55, 0, y);
        spdlog::info("Yaw={:.1f}deg: CL={:.3f}, CD={:.3f}, CY={:.3f}", 
            y*180/M_PI, c.CL, c.CD, c.CY);
    }
    
    // === テスト5: 速度スイープ ===
    spdlog::info("\n=== Test 5: Velocity Sweep ===");
    std::vector<double> v_test = {10, 15, 20, 25, 30};
    for (double v : v_test) {
        auto f = aero_map.compute_forces(v, 40, 55, 0, 0);
        spdlog::info("V={:.0f}m/s: Fz={:.0f}N, Fx={:.0f}N", v, f.Fz, f.Fx);
    }
    
    // === CSV出力 ===
    spdlog::info("\n=== Generating CSV Data ===");
    std::ofstream ofs("aero_map_data.csv");
    ofs << "rh_f,rh_r,roll,yaw,CL,CD,CY,CoP_x,CoP_dist\n";
    
    for (double rh_f : {30.0, 40.0, 50.0}) {
        for (double rh_r : {40.0, 55.0, 70.0}) {
            for (double roll : {-0.044, 0.0, 0.044}) {
                for (double yaw : {-0.087, 0.0, 0.087}) {
                    auto c = aero_map.compute(rh_f, rh_r, roll, yaw);
                    ofs << rh_f << "," << rh_r << "," 
                        << roll*180/M_PI << "," << yaw*180/M_PI << ","
                        << c.CL << "," << c.CD << "," << c.CY << ","
                        << c.CoP_x << "," << c.CoP_dist << "\n";
                }
            }
        }
    }
    ofs.close();
    spdlog::info("Saved to aero_map_data.csv");
    
    // === 検証 ===
    spdlog::info("\n=== Verification ===");
    bool passed = true;
    
    // CL範囲チェック（-4.0 < CL < -1.0）
    if (coef.CL > -4.0 && coef.CL < -1.0) {
        spdlog::info("✓ CL in realistic range: {:.3f}", coef.CL);
    } else {
        spdlog::warn("✗ CL out of range: {:.3f}", coef.CL);
        passed = false;
    }
    
    // CD範囲チェック
    if (coef.CD > 0.5 && coef.CD < 2.0) {
        spdlog::info("✓ CD in realistic range: {:.3f}", coef.CD);
    } else {
        spdlog::warn("✗ CD out of range: {:.3f}", coef.CD);
        passed = false;
    }
    
    // ダウンフォースチェック（負の値）
    if (forces.Fz < 0) {
        spdlog::info("✓ Downforce is negative (correct): {:.0f} N", forces.Fz);
    } else {
        spdlog::warn("✗ Fz should be negative: {:.0f} N", forces.Fz);
        passed = false;
    }
    
    // ドラッグチェック（負の値）
    if (forces.Fx < 0) {
        spdlog::info("✓ Drag is negative (correct): {:.0f} N", forces.Fx);
    } else {
        spdlog::warn("✗ Fx should be negative: {:.0f} N", forces.Fx);
        passed = false;
    }
    
    // ライドハイト効果（低いほどダウンフォース増加）
    auto c_low = aero_map.compute(25, 40, 0, 0);
    auto c_high = aero_map.compute(55, 75, 0, 0);
    if (c_low.CL < c_high.CL) {  // より負 = より大きなダウンフォース
        spdlog::info("✓ Ground effect correct: low RH gives more DF");
    } else {
        spdlog::warn("✗ Ground effect incorrect");
        passed = false;
    }
    
    if (passed) {
        spdlog::info("\n4D Aero Map test PASSED");
    } else {
        spdlog::error("\n4D Aero Map test FAILED");
        return 1;
    }
    
    return 0;
}
