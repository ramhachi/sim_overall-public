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
    spdlog::info("CoP_x={:.3f}m, CoP_y_x={:.3f}m", coef.CoP_x, coef.CoP_y_x);
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
    ofs << "rh_f,rh_r,roll,yaw,CL,CD,CY,CoP_x,CoP_y_x\n";
    
    for (double rh_f : {30.0, 40.0, 50.0}) {
        for (double rh_r : {40.0, 55.0, 70.0}) {
            for (double roll : {-0.044, 0.0, 0.044}) {
                for (double yaw : {-0.087, 0.0, 0.087}) {
                    auto c = aero_map.compute(rh_f, rh_r, roll, yaw);
                    ofs << rh_f << "," << rh_r << "," 
                        << roll*180/M_PI << "," << yaw*180/M_PI << ","
                        << c.CL << "," << c.CD << "," << c.CY << ","
                        << c.CoP_x << "," << c.CoP_y_x << "\n";
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
    
    // === #31/#32/#33 判定 ===
    auto check = [&](bool ok, const char* what) {
        if (ok) spdlog::info("✓ {}", what);
        else { spdlog::error("✗ {}", what); passed = false; }
    };
    auto near = [](double a, double b, double tol = 1e-9) { return std::abs(a - b) < tol; };

    // 目標バランス: CoP_x = balance·L_ref（旧: 0.45 m を直接 m として使用していた二重定義）
    check(near(coef.CoP_x, 0.45 * 1.6), "CoP_x = aero_balance * L_ref at nominal RH");
    auto f_split = aero_map.compute_forces(20, 40, 55, 0, 0);
    check(near(f_split.Fz_front, f_split.Fz * 0.45), "Fz_front derived from CoP_x");

    // 域外フラグ + カウンタ（クランプは維持）
    const auto before = aero_map.out_of_domain_count();
    auto ood = aero_map.compute(10, 55, 0, 0.5);
    check(ood.out_of_domain == (1u | 8u), "out-of-domain bitmask RHf|yaw");
    check(aero_map.out_of_domain_count() == before + 1, "out-of-domain counter increments");
    check(near(ood.CL, aero_map.compute(20, 55, 0, grid.yaw.back()).CL), "out-of-domain input is clamped");

    // DRS 目標（drs_on データ無し）: CL/CD 比 + リアウイング規則の CoP 前進
    auto on = aero_map.compute(40, 55, 0, 0, true);
    check(near(on.CL, coef.CL * 0.54) && near(on.CD, coef.CD * 0.74), "DRS target CL/CD ratios");
    check(near(on.CoP_x, coef.CoP_x / 0.54), "DRS rear-wing rule: CoP_on = CoP_off / r_CL");
    AeroMap4D<double> shifted(params);
    shifted.params().drs_CoP_x_shift = 0.1;
    check(near(shifted.compute(40, 55, 0, 0, true).CoP_x, coef.CoP_x + 0.1), "explicit DRS CoP shift target");
    auto f_off = aero_map.compute_forces(20, 40, 55, 0, 0, 0.0);
    auto f_on = aero_map.compute_forces(20, 40, 55, 0, 0, 1.0);
    auto f_half = aero_map.compute_forces(20, 40, 55, 0, 0, 0.5);
    check(near(f_half.Fz, 0.5 * (f_off.Fz + f_on.Fz)), "DRS flap blend is linear");

    // h_aero: ドラッグが CG より上 → ノーズアップ My = −Δh·Fx
    AeroMap4D<double> high(params);
    high.params().h_aero = high.params().h_cg + 0.2;
    AeroMap4D<double> level(params);
    level.params().h_aero = level.params().h_cg;
    auto fh = high.compute_forces(20, 40, 55, 0, 0.1);
    auto fl = level.compute_forces(20, 40, 55, 0, 0.1);
    check(near(fh.My - fl.My, -0.2 * fh.Fx) && fh.My > fl.My, "drag at h_aero gives nose-up pitch moment");
    check(near(fh.Mx - fl.Mx, 0.2 * fh.Fy), "side force at h_aero gives roll moment");

    // 実測マップ（合成例）は目標を上書きし、drs_on ブロックを使う
    AeroMap4D<double> measured(params);
    const std::string example = std::string(FSAE_SOURCE_DIR) + "/data/aero/aero_map_format_example_SYNTHETIC.json";
    check(measured.load_from_json(example) && measured.last_error().empty(), "example map loads");
    auto m_off = measured.compute(40, 55, 0, 0);
    auto m_on = measured.compute(40, 55, 0, 0, true);
    check(near(m_off.CL, -2.5) && near(m_off.CoP_y_x, 0.9), "measured map overrides targets (CoP_y_x separate)");
    check(near(m_on.CL, -1.35) && near(m_on.CD, 0.75) && near(m_on.CoP_x, 0.95), "drs_on block selected by flag");

    // ロード失敗は理由を返し、既存マップを変更しない
    {
        std::ofstream bad("aero_bad.json");
        bad << R"({"axes":{"rh_front_mm":[30,50],"rh_rear_mm":[45,65],"roll_rad":[-0.05,0.05],"yaw_rad":[-0.1,0.1]},)"
            << R"("data":{"CL":[1,2,3],"CD":[],"CY":[],"CoP_x":[]}})";
    }
    AeroMap4D<double> keep(params);
    const double cl_before = keep.compute(40, 55, 0, 0).CL;
    check(!keep.load_from_json("aero_bad.json"), "malformed map rejected");
    check(keep.last_error().find("'CL' has 3 values, expected 16") != std::string::npos, "load error names key and shape");
    check(near(keep.compute(40, 55, 0, 0).CL, cl_before), "failed load keeps previous map");
    check(!keep.load_from_json("no_such_file.json") && keep.last_error() == "cannot open file", "missing file reason");

    // direct-load 基準点 → CG 変換: リア車軸・CG 高さの −350 N → My = +x_cg·350（ノーズアップ）、前輪配分 0
    {
        std::ofstream d("aero_direct.json");
        auto arr = [&](double v) { std::string s = "["; for (int i = 0; i < 16; ++i) s += (i ? "," : "") + std::to_string(v); return s + "]"; };
        d << R"({"V_ref_mps":20,"reference_point":{"x_from_rear_axle_m":0,"y_m":0,"height_m":)" << params.h_cg << "},"
          << R"("axes":{"rh_front_mm":[30,50],"rh_rear_mm":[45,65],"roll_rad":[-0.05,0.05],"yaw_rad":[-0.1,0.1]},"data":{)"
          << R"("Fx_N":)" << arr(0) << R"(,"Fy_N":)" << arr(0) << R"(,"Fz_N":)" << arr(-350)
          << R"(,"Mx_Nm":)" << arr(0) << R"(,"My_Nm":)" << arr(0) << R"(,"Mz_Nm":)" << arr(0) << "}}";
    }
    AeroMap4D<double> direct(params);
    check(direct.load_from_json("aero_direct.json") && direct.uses_direct_load_map(), "direct-load map with reference_point loads");
    auto fd = direct.compute_forces(20, 40, 55, 0, 0);
    check(near(fd.My, 0.8 * 350.0, 1e-6) && near(fd.Fz_front, 0.0, 1e-6), "reference-point moment transfer to CG");

    if (passed) {
        spdlog::info("\n4D Aero Map test PASSED");
    } else {
        spdlog::error("\n4D Aero Map test FAILED");
        return 1;
    }
    
    return 0;
}
