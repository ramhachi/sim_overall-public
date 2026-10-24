/// @file test_bouc_wen_vehicle.cpp
/// @brief Bouc-Wenダンパー統合8DOFモデルのテスト

#include <fsae/vehicle/vehicle_8dof_bw.hpp>
#include <fsae/vehicle/vehicle_8dof_arb.hpp>
#include <spdlog/spdlog.h>
#include <cmath>
#include <fstream>

using namespace fsae::vehicle;

int main() {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("=== Bouc-Wen Damper Vehicle Test ===");
    
    // パラメータ設定
    VehicleParams params;
    
    // Bouc-Wenパラメータ（ヒステリシスを明確にするため強めに設定）
    params.suspension.c0 = 2000.0;          // 粘性減衰 [Ns/m]
    params.suspension.bw_alpha = 1500.0;    // ヒステリシス強度
    params.suspension.bw_A = 100.0;
    params.suspension.bw_beta = 300.0;
    params.suspension.bw_gamma = 0.0;
    params.suspension.bw_n = 1.0;
    
    // ARB（前回テスト済みの値）
    params.suspension.k_arb_f = 5000.0;
    params.suspension.k_arb_r = 3000.0;
    
    // モデル作成
    Vehicle8DOF_BW model_bw(params);
    Vehicle8DOF_ARB model_linear(params);  // 比較用（線形ダンパー）
    
    // 初速設定
    double v0 = 15.0;
    model_bw.set_initial_velocity(v0);
    model_linear.set_initial_velocity(v0);
    
    // シミュレーション設定
    double dt = 0.001;
    double t_end = 3.0;
    double t_steer = 1.0;
    double delta = 0.15;
    
    // CSVファイル
    std::ofstream ofs("bouc_wen_comparison.csv");
    ofs << "time,phi_bw,phi_linear,vz_bw,vz_linear,bw_force_fl,damper_z_fl\n";
    
    spdlog::info("Running simulation...");
    spdlog::info("  Bouc-Wen: alpha={}, A={}, beta={}, n={}", 
        params.suspension.bw_alpha, params.suspension.bw_A,
        params.suspension.bw_beta, params.suspension.bw_n);
    
    // ヒステリシスループ記録用
    std::ofstream hyst_ofs("hysteresis_loop.csv");
    hyst_ofs << "time,susp_velocity,damper_force,damper_z\n";
    
    double max_phi_bw = 0, max_phi_linear = 0;
    
    for (double t = 0; t <= t_end; t += dt) {
        DriverInput input;
        input.throttle = 0.1;
        input.steering = (t >= t_steer) ? delta : 0;
        
        model_bw.step(dt, input);
        model_linear.step(dt, input);
        
        // ロール角
        double phi_bw = std::abs(model_bw.state().phi);
        double phi_linear = std::abs(model_linear.state().phi);
        
        max_phi_bw = std::max(max_phi_bw, phi_bw);
        max_phi_linear = std::max(max_phi_linear, phi_linear);
        
        // 記録（10ステップごと）
        if (static_cast<int>(t / dt) % 10 == 0) {
            auto bw_forces = model_bw.get_bouc_wen_forces();
            ofs << t << "," 
                << model_bw.state().phi * 180.0 / M_PI << ","
                << model_linear.state().phi * 180.0 / M_PI << ","
                << model_bw.state().vz << ","
                << model_linear.state().vz << ","
                << bw_forces[0] << ","
                << model_bw.state().damper_z[0] << "\n";
            
            // ヒステリシスループ（FL輪のみ）
            // サスペンション速度近似
            double susp_vel = model_bw.state().vz_u[0];
            hyst_ofs << t << "," << susp_vel << "," 
                     << bw_forces[0] << "," << model_bw.state().damper_z[0] << "\n";
        }
    }
    
    ofs.close();
    hyst_ofs.close();
    
    // 結果表示
    spdlog::info("");
    spdlog::info("=== Results ===");
    spdlog::info("Max roll angle:");
    spdlog::info("  Bouc-Wen: {:.3f}°", max_phi_bw * 180.0 / M_PI);
    spdlog::info("  Linear:   {:.3f}°", max_phi_linear * 180.0 / M_PI);
    
    // Bouc-Wen履歴変数
    auto& state = model_bw.state();
    spdlog::info("");
    spdlog::info("Final Bouc-Wen hysteresis variables:");
    spdlog::info("  FL: z={:.6f}", state.damper_z[0]);
    spdlog::info("  FR: z={:.6f}", state.damper_z[1]);
    spdlog::info("  RL: z={:.6f}", state.damper_z[2]);
    spdlog::info("  RR: z={:.6f}", state.damper_z[3]);
    
    // Bouc-Wen力
    auto bw_forces = model_bw.get_bouc_wen_forces();
    spdlog::info("");
    spdlog::info("Final Bouc-Wen damper forces:");
    spdlog::info("  FL: {:.1f} N", bw_forces[0]);
    spdlog::info("  FR: {:.1f} N", bw_forces[1]);
    spdlog::info("  RL: {:.1f} N", bw_forces[2]);
    spdlog::info("  RR: {:.1f} N", bw_forces[3]);
    
    spdlog::info("");
    spdlog::info("Saved to bouc_wen_comparison.csv and hysteresis_loop.csv");
    
    // 検証
    bool passed = true;
    
    // ロール角が現実的な範囲内か（0.5° 〜 10°）
    double max_phi_deg = max_phi_bw * 180.0 / M_PI;
    if (max_phi_deg > 0.5 && max_phi_deg < 10.0) {
        spdlog::info("✓ Roll angle in realistic range: {:.3f}°", max_phi_deg);
    } else {
        spdlog::warn("✗ Roll angle out of expected range: {:.3f}°", max_phi_deg);
        passed = false;
    }
    
    // 履歴変数が非ゼロ（ヒステリシスが発生している）
    double z_sum = std::abs(state.damper_z[0]) + std::abs(state.damper_z[1]) 
                 + std::abs(state.damper_z[2]) + std::abs(state.damper_z[3]);
    if (z_sum > 1e-6) {
        spdlog::info("✓ Hysteresis active (sum|z|={:.6f})", z_sum);
    } else {
        spdlog::warn("✗ Hysteresis not active");
        passed = false;
    }
    
    if (passed) {
        spdlog::info("");
        spdlog::info("Bouc-Wen vehicle test PASSED");
    } else {
        spdlog::error("");
        spdlog::error("Bouc-Wen vehicle test FAILED");
        return 1;
    }
    
    return 0;
}
