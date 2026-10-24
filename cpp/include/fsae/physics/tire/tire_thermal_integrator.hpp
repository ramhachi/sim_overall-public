#pragma once
/// @file tire_thermal_integrator.hpp
/// @brief タイヤ熱モデル（表面温度の動的計算）

#include <array>
#include <cmath>
#include <algorithm>

namespace fsae::physics::tire {

/// タイヤ熱パラメータ
struct TireThermalParams {
    // 熱容量
    double m_tread = 0.5;          // トレッド質量 [kg]
    double c_p = 1000.0;           // 比熱 [J/(kg·K)]
    
    // 対流冷却
    double h_conv_base = 50.0;     // 基準熱伝達係数 [W/(m²·K)]
    double A_tread = 0.03;         // トレッド表面積 [m²]
    double v_ref = 10.0;           // 基準速度 [m/s]
    
    // 路面伝導
    double h_road = 200.0;         // 路面熱伝達係数 [W/(m²·K)]
    double A_contact = 0.01;       // 接地面積 [m²]
    double T_road = 25.0;          // 路面温度 [°C]
    
    // 初期温度
    double T_initial = 25.0;       // 初期温度 [°C]
};

/// スリップ情報（熱計算用）
struct TireSlipInfo {
    double vsx = 0.0;  // 縦スリップ速度 [m/s]
    double vsy = 0.0;  // 横スリップ速度 [m/s]
};

/// タイヤ熱積分器（4輪分）
class TireThermalIntegrator {
public:
    explicit TireThermalIntegrator(const TireThermalParams& params = TireThermalParams())
        : params_(params) 
    {
        // 全輪を初期温度に設定
        T_surf_.fill(params_.T_initial);
    }
    
    /// タイヤ温度を更新（設計書_統合版.md 4.3.1節の熱収支方程式）
    /// @param dt タイムステップ [s]
    /// @param forces 各輪のタイヤ力
    /// @param slip_info 各輪のスリップ情報
    /// @param v_vehicle 車速 [m/s]（対流冷却用）
    /// @param T_air 外気温 [°C]
    void update(double dt, 
                const std::array<TireForces, 4>& forces,
                const std::array<TireSlipInfo, 4>& slip_info,
                double v_vehicle,
                double T_air) 
    {
        using std::abs, std::max, std::pow;
        
        for (int i = 0; i < 4; i++) {
            // 発熱（摩擦仕事）: Q_gen = |Fx * vsx| + |Fy * vsy|
            double Q_gen = abs(forces[i].Fx * slip_info[i].vsx) 
                         + abs(forces[i].Fy * slip_info[i].vsy);
            
            // 対流冷却: Q_conv = h_conv(v) * A * (T_surf - T_air)
            // h_conv(v) = h_conv_base * (v / v_ref)^0.8
            double v_eff = max(v_vehicle, 0.1);  // ゼロ割り防止
            double h_conv = params_.h_conv_base * pow(v_eff / params_.v_ref, 0.8);
            double Q_conv = h_conv * params_.A_tread * (T_surf_[i] - T_air);
            
            // 路面伝導: Q_cond = h_road * A_contact * (T_surf - T_road)
            double Q_cond = params_.h_road * params_.A_contact 
                          * (T_surf_[i] - params_.T_road);
            
            // 熱収支: m * c_p * dT/dt = Q_gen - Q_conv - Q_cond
            double dT_dt = (Q_gen - Q_conv - Q_cond) / (params_.m_tread * params_.c_p);
            
            // オイラー法で積分
            T_surf_[i] += dT_dt * dt;
            
            // 物理的に妥当な範囲にクランプ（-10°C ~ 150°C）
            T_surf_[i] = std::clamp(T_surf_[i], -10.0, 150.0);
        }
    }
    
    /// 指定輪の温度を取得
    /// @param wheel_index 輪インデックス (0=FL, 1=FR, 2=RL, 3=RR)
    /// @return タイヤ表面温度 [°C]
    double get_temperature(int wheel_index) const {
        return T_surf_[wheel_index];
    }
    
    /// 全輪の温度を取得
    const std::array<double, 4>& get_temperatures() const {
        return T_surf_;
    }
    
    /// 温度をリセット
    void reset(double T = -1.0) {
        double temp = (T < 0) ? params_.T_initial : T;
        T_surf_.fill(temp);
    }
    
    /// パラメータを設定
    void set_params(const TireThermalParams& params) {
        params_ = params;
    }

private:
    TireThermalParams params_;
    std::array<double, 4> T_surf_;  // 各輪の表面温度 [°C]
};

} // namespace fsae::physics::tire
