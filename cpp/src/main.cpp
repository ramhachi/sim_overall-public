// src/main.cpp
/// FSAE Simulator main entry point

#include <iostream>
#include <fstream>
#include <chrono>
#include <vector>
#include <filesystem>

#include "fsae/physics/tire/tmeasy.hpp"
#include "fsae/physics/suspension/bouc_wen.hpp"
#include "fsae/solver/rosenbrock.hpp"
#include "fsae/math/rotation.hpp"

#include <Eigen/Dense>
#include <spdlog/spdlog.h>
#include <nlohmann/json.hpp>

using namespace fsae;

namespace {
std::string ensure_output_path(const std::string& filename) {
    std::filesystem::path output_dir = std::filesystem::path("output") / "quarter_car";
    std::filesystem::create_directories(output_dir);
    return (output_dir / filename).string();
}
}

/// 時系列データ保存用構造体
struct TimeSeriesData {
    std::vector<double> time;
    std::vector<double> z_s;      // ばね上変位
    std::vector<double> z_u;      // ばね下変位
    std::vector<double> dz_s;     // ばね上速度
    std::vector<double> dz_u;     // ばね下速度
    std::vector<double> F_spring; // ばね力
    std::vector<double> F_damper; // ダンパー力
    std::vector<double> F_tire;   // タイヤ力
    std::vector<double> z_road;   // 路面入力
    
    void reserve(size_t n) {
        time.reserve(n);
        z_s.reserve(n);
        z_u.reserve(n);
        dz_s.reserve(n);
        dz_u.reserve(n);
        F_spring.reserve(n);
        F_damper.reserve(n);
        F_tire.reserve(n);
        z_road.reserve(n);
    }
    
    void save_csv(const std::string& filename) {
        std::ofstream out(filename);
        out << "time,z_s,z_u,dz_s,dz_u,F_spring,F_damper,F_tire,z_road\n";
        for (size_t i = 0; i < time.size(); ++i) {
            out << time[i] << ","
                << z_s[i] << ","
                << z_u[i] << ","
                << dz_s[i] << ","
                << dz_u[i] << ","
                << F_spring[i] << ","
                << F_damper[i] << ","
                << F_tire[i] << ","
                << z_road[i] << "\n";
        }
        spdlog::info("Saved {} data points to {}", time.size(), filename);
    }
};

/// Quarter Carシミュレーション（時系列出力版）
void run_quarter_car_demo() {
    spdlog::info("=== Quarter Car Demo ===");
    
    // パラメータ
    const double m_s = 62.5;    // ばね上質量 [kg]
    const double m_u = 12.0;    // ばね下質量 [kg]
    const double k_s = 25000.0; // ばねレート [N/m]
    const double k_t = 200000.0;// タイヤ剛性 [N/m]
    
    // Bouc-Wenパラメータ（MATLABと完全一致）
    physics::suspension::BoucWenParams<double> bw_params;
    bw_params.c0 = 1500.0;
    bw_params.k0 = 0.0;       // 重要: MATLABと一致させる
    bw_params.alpha = 5000.0;
    bw_params.A = 1.0;
    bw_params.beta = 500.0;
    bw_params.gamma = 100.0;
    bw_params.n = 2.0;
    
    // 時系列データ
    TimeSeriesData data;
    data.reserve(10000);
    
    // シミュレーション設定
    const double t_start = 0.0;
    const double t_end = 2.0;
    const double dt_output = 0.001;  // 出力間隔 1ms
    const double step_time = 0.2;    // ステップ入力タイミング
    const double step_height = 0.02; // ステップ高さ 20mm
    
    // 状態: [z_s, z_u, dz_s, dz_u, z_bw]
    Eigen::VectorXd x(5);
    x.setZero();
    
    // 時間ループ
    double t = t_start;
    
    auto start_time = std::chrono::high_resolution_clock::now();
    
    while (t < t_end) {
        // 路面入力（ステップ）
        double z_road = (t >= step_time) ? step_height : 0.0;
        
        // 状態を取り出し
        double z_s = x(0), z_u = x(1);
        double dz_s = x(2), dz_u = x(3);
        double z_bw = x(4);
        
        // 力を計算
        double delta_s = z_s - z_u;
        double v_s = dz_s - dz_u;
        
        double F_spring = k_s * delta_s;
        double F_damper = physics::suspension::compute_bouc_wen_force(bw_params, delta_s, v_s, z_bw);
        double F_tire = k_t * (z_road - z_u);
        F_tire = std::max(F_tire, 0.0);
        
        // データ保存
        data.time.push_back(t);
        data.z_s.push_back(z_s * 1000);      // mm
        data.z_u.push_back(z_u * 1000);      // mm
        data.dz_s.push_back(dz_s * 1000);    // mm/s
        data.dz_u.push_back(dz_u * 1000);    // mm/s
        data.F_spring.push_back(F_spring);
        data.F_damper.push_back(F_damper);
        data.F_tire.push_back(F_tire);
        data.z_road.push_back(z_road * 1000); // mm
        
        // RK4で積分（シンプルな実装）
        auto dynamics = [&](const Eigen::VectorXd& state) -> Eigen::VectorXd {
            double zs = state(0), zu = state(1);
            double dzs = state(2), dzu = state(3);
            double zbw = state(4);
            
            double delta = zs - zu;
            double vs = dzs - dzu;
            
            double Fsp = k_s * delta;
            double Fd = physics::suspension::compute_bouc_wen_force(bw_params, delta, vs, zbw);
            double Ft = k_t * (z_road - zu);
            Ft = std::max(Ft, 0.0);
            
            double dzbw = physics::suspension::compute_bouc_wen_dz(bw_params, vs, zbw);
            
            Eigen::VectorXd dxdt(5);
            dxdt(0) = dzs;
            dxdt(1) = dzu;
            dxdt(2) = (-Fsp - Fd) / m_s;
            dxdt(3) = (Fsp + Fd + Ft) / m_u;
            dxdt(4) = dzbw;
            
            return dxdt;
        };
        
        // RK4ステップ
        double h = dt_output;
        Eigen::VectorXd k1 = dynamics(x);
        Eigen::VectorXd k2 = dynamics(x + 0.5 * h * k1);
        Eigen::VectorXd k3 = dynamics(x + 0.5 * h * k2);
        Eigen::VectorXd k4 = dynamics(x + h * k3);
        x = x + (h / 6.0) * (k1 + 2*k2 + 2*k3 + k4);
        
        t += dt_output;
    }
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    spdlog::info("Simulation complete:");
    spdlog::info("  Duration: {:.2f} s", t_end - t_start);
    spdlog::info("  Data points: {}", data.time.size());
    spdlog::info("  Compute time: {} ms", duration.count());
    spdlog::info("  Final state: z_s={:.2f} mm, z_u={:.2f} mm", 
                 data.z_s.back(), data.z_u.back());
    
    // CSV出力
    data.save_csv(ensure_output_path("quarter_car_result.csv"));
}

int main(int argc, char** argv) {
    spdlog::set_level(spdlog::level::info);
    spdlog::info("FSAE Vehicle Dynamics Simulator v0.1.0");
    
    run_quarter_car_demo();
    
    return 0;
}
