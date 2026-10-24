/// @file test_lugre_vehicle.cpp
/// @brief 8DOF + Bouc-Wen + LuGre 統合モデルテスト

#include <fsae/vehicle/vehicle_8dof_full.hpp>
#include <fsae/vehicle/vehicle_8dof_bw.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <fstream>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== LuGre Friction Vehicle Test ===");
    
    // パラメータ設定
    VehicleParams params;
    
    // Bouc-Wenパラメータ
    params.suspension.c0 = 2000.0;
    params.suspension.bw_alpha = 1500.0;
    params.suspension.bw_A = 100.0;
    params.suspension.bw_beta = 300.0;
    params.suspension.bw_gamma = 0.0;
    params.suspension.bw_n = 1.0;
    
    // LuGreパラメータ（設計書5.4準拠）
    params.suspension.lugre_sigma0 = 10000.0;  // 毛剛性
    params.suspension.lugre_sigma1 = 10.0;     // 毛減衰
    params.suspension.lugre_sigma2 = 0.0;      // 粘性摩擦
    params.suspension.lugre_Fc = 20.0;         // クーロン摩擦
    params.suspension.lugre_Fs = 30.0;         // 静止摩擦
    params.suspension.lugre_vs = 0.01;         // ストライベック速度
    
    // ARB
    params.suspension.k_arb_f = 5000.0;
    params.suspension.k_arb_r = 3000.0;
    
    // モデル作成
    Vehicle8DOF_Full model_full(params);     // Bouc-Wen + LuGre
    Vehicle8DOF_BW model_bw(params);         // Bouc-Wenのみ（比較用）
    
    // 初速設定
    double v0 = 15.0;
    model_full.set_initial_velocity(v0);
    model_bw.set_initial_velocity(v0);
    
    // シミュレーション設定
    double dt = 0.001;
    double t_end = 3.0;
    double t_steer = 1.0;
    double delta = 0.15;
    
    // CSVファイル
    std::ofstream ofs("lugre_comparison.csv");
    ofs << "time,phi_full,phi_bw,friction_force_fl,friction_z_fl,damper_force_fl\n";
    
    spdlog::info("Running simulation...");
    spdlog::info("  LuGre: sigma0={}, Fc={}, Fs={}, vs={}",
        params.suspension.lugre_sigma0, params.suspension.lugre_Fc,
        params.suspension.lugre_Fs, params.suspension.lugre_vs);
    
    double max_phi_full = 0, max_phi_bw = 0;
    double max_friction_force = 0;
    
    for (double t = 0; t <= t_end; t += dt) {
        DriverInput input;
        input.throttle = 0.1;
        input.steering = (t >= t_steer) ? delta : 0;
        
        model_full.step(dt, input);
        model_bw.step(dt, input);
        
        // ロール角
        double phi_full = std::abs(model_full.state().phi);
        double phi_bw = std::abs(model_bw.state().phi);
        
        max_phi_full = std::max(max_phi_full, phi_full);
        max_phi_bw = std::max(max_phi_bw, phi_bw);
        
        // LuGre力
        auto lugre_forces = model_full.get_lugre_forces();
        max_friction_force = std::max(max_friction_force, std::abs(lugre_forces[0]));
        
        // 記録（10ステップごと）
        if (static_cast<int>(t / dt) % 10 == 0) {
            auto bw_forces = model_full.get_bouc_wen_forces();
            ofs << t << "," 
                << model_full.state().phi * 180.0 / M_PI << ","
                << model_bw.state().phi * 180.0 / M_PI << ","
                << lugre_forces[0] << ","
                << model_full.state().friction_z[0] << ","
                << bw_forces[0] << "\n";
        }
    }
    
    ofs.close();
    
    // 結果表示
    spdlog::info("");
    spdlog::info("=== Results ===");
    spdlog::info("Max roll angle:");
    spdlog::info("  Full (BW+LuGre): {:.3f}°", max_phi_full * 180.0 / M_PI);
    spdlog::info("  BW only:         {:.3f}°", max_phi_bw * 180.0 / M_PI);
    
    spdlog::info("");
    spdlog::info("LuGre friction:");
    spdlog::info("  Max friction force: {:.1f} N", max_friction_force);
    
    // LuGre状態
    auto& state = model_full.state();
    spdlog::info("");
    spdlog::info("Final LuGre bristle deformation:");
    spdlog::info("  FL: z={:.6f}", state.friction_z[0]);
    spdlog::info("  FR: z={:.6f}", state.friction_z[1]);
    spdlog::info("  RL: z={:.6f}", state.friction_z[2]);
    spdlog::info("  RR: z={:.6f}", state.friction_z[3]);
    
    auto lugre_forces = model_full.get_lugre_forces();
    spdlog::info("");
    spdlog::info("Final LuGre friction forces:");
    spdlog::info("  FL: {:.1f} N", lugre_forces[0]);
    spdlog::info("  FR: {:.1f} N", lugre_forces[1]);
    spdlog::info("  RL: {:.1f} N", lugre_forces[2]);
    spdlog::info("  RR: {:.1f} N", lugre_forces[3]);
    
    spdlog::info("");
    spdlog::info("Saved to lugre_comparison.csv");
    
    // 検証
    bool passed = true;
    
    // ロール角が現実的範囲
    double max_phi_deg = max_phi_full * 180.0 / M_PI;
    if (max_phi_deg > 0.5 && max_phi_deg < 10.0) {
        spdlog::info("✓ Roll angle in realistic range: {:.3f}°", max_phi_deg);
    } else {
        spdlog::warn("✗ Roll angle out of expected range: {:.3f}°", max_phi_deg);
        passed = false;
    }
    
    // 摩擦力が発生している
    if (max_friction_force > 1.0) {
        spdlog::info("✓ Friction force active: max={:.1f} N", max_friction_force);
    } else {
        spdlog::warn("✗ Friction force too small: {:.1f} N", max_friction_force);
        passed = false;
    }
    
    // 毛変形が非ゼロ
    double z_sum = std::abs(state.friction_z[0]) + std::abs(state.friction_z[1]) 
                 + std::abs(state.friction_z[2]) + std::abs(state.friction_z[3]);
    if (z_sum > 1e-9) {
        spdlog::info("✓ Bristle deformation active (sum|z|={:.6f})", z_sum);
    } else {
        spdlog::warn("✗ Bristle deformation not active");
        passed = false;
    }
    
    if (passed) {
        spdlog::info("");
        spdlog::info("LuGre vehicle test PASSED");
    } else {
        spdlog::error("");
        spdlog::error("LuGre vehicle test FAILED");
        return 1;
    }
    
    return 0;
}
