#pragma once
/// @file tire_thermal_integrator.hpp
/// @brief タイヤ熱モデル（表面 1 ノード、または表面＋バルク 2 ノード）
///
/// 唯一のタイヤ熱モデル（旧 thermal.hpp の重複モデルは削除済み）。
/// TSTC（PINN 入力）は表面ノード温度。

#include <array>
#include <cmath>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <fsae/physics/tire/nn_lookup_tire.hpp>

namespace fsae::physics::tire {

/// タイヤ熱パラメータ（JSON: `tire.thermal` ブロック、キー名＝メンバ名）
struct TireThermalParams {
    // 表面ノード熱容量
    double m_tread = 0.5;          // トレッド表層質量 [kg]
    double c_p = 1000.0;           // 比熱 [J/(kg·K)]

    // 対流冷却 h_conv(v) = h_conv_base * (v / v_ref)^0.8
    double h_conv_base = 50.0;     // 基準熱伝達係数 [W/(m²·K)]
    double A_tread = 0.03;         // トレッド表面積 [m²]
    double v_ref = 10.0;           // 基準速度 [m/s]

    // 路面伝導: A_contact = Fz / P（接地圧 ≈ 空気圧）
    double h_road = 200.0;         // 路面熱伝達係数 [W/(m²·K)]
    double pressure_kpa = 82.7;    // 空気圧 [kPa]。既定 12 psi = Hoosier 10" の TTC 標準圧
    double A_contact = 0.01;       // Fz 未供給時（TireSlipInfo::Fz < 0）の固定接地面積 [m²]
    double T_road = 25.0;          // 路面温度 [°C]

    // 初期温度
    double T_initial = 45.0;       // 初期温度 [°C]（走行前の暖まった状態）

    // 2 ノード（表面＋バルク/カーカス）。m_bulk <= 0 なら 1 ノード（既定）。
    // 推奨値の根拠（実測同定値ではない）: FSAE 10" スリックのトレッド＋カーカスゴム
    // ≈ 2 kg、ゴム比熱 ≈ 1500 J/(kg·K)。G = k·A/t ≈ 0.2 W/(m·K) × 0.25 m² / 5 mm ≈ 10 W/K。
    // 発熱の 7 割を表面（摩擦）、残りをバルク（ヒステリシス）に配分。
    double m_bulk = 0.0;               // バルク質量 [kg]（推奨 2.0）
    double c_p_bulk = 1500.0;          // バルク比熱 [J/(kg·K)]
    double G_surface_bulk = 10.0;      // 表面-バルク間熱コンダクタンス [W/K]
    double surface_heat_fraction = 0.7; // 発熱の表面配分 [-]

    static TireThermalParams from_json(const nlohmann::json& j) {
        TireThermalParams p;
        auto get = [&](const char* key, double& v) { v = j.value(key, v); };
        get("m_tread", p.m_tread);
        get("c_p", p.c_p);
        get("h_conv_base", p.h_conv_base);
        get("A_tread", p.A_tread);
        get("v_ref", p.v_ref);
        get("h_road", p.h_road);
        get("pressure_kpa", p.pressure_kpa);
        get("A_contact", p.A_contact);
        get("T_road", p.T_road);
        get("T_initial", p.T_initial);
        get("m_bulk", p.m_bulk);
        get("c_p_bulk", p.c_p_bulk);
        get("G_surface_bulk", p.G_surface_bulk);
        get("surface_heat_fraction", p.surface_heat_fraction);
        return p;
    }
};

/// スリップ情報（熱計算用）
struct TireSlipInfo {
    double vsx = 0.0;  // 縦スリップ速度 [m/s]
    double vsy = 0.0;  // 横スリップ速度 [m/s]
    double Fz = -1.0;  // 垂直荷重 [N]。< 0 は未供給 → A_contact 固定
};

/// タイヤ熱積分器（4輪分）
class TireThermalIntegrator {
public:
    explicit TireThermalIntegrator(const TireThermalParams& params = TireThermalParams())
        : params_(params)
    {
        reset();
    }

    /// タイヤ温度を更新（設計書_統合版.md 4.3.1節の熱収支方程式、前進オイラー）
    /// @param dt タイムステップ [s]
    /// @param forces 各輪のタイヤ力
    /// @param slip_info 各輪のスリップ速度と Fz
    /// @param v_vehicle 車速 [m/s]（対流冷却用）
    /// @param T_air 外気温 [°C]
    void update(double dt,
                const std::array<TireForces, 4>& forces,
                const std::array<TireSlipInfo, 4>& slip_info,
                double v_vehicle,
                double T_air)
    {
        using std::abs, std::max, std::pow;
        const bool two_node = params_.m_bulk > 0.0;
        const double h_conv =
            params_.h_conv_base * pow(max(v_vehicle, 0.1) / params_.v_ref, 0.8);

        for (int i = 0; i < 4; i++) {
            const double Q_gen = abs(forces[i].Fx * slip_info[i].vsx)
                               + abs(forces[i].Fy * slip_info[i].vsy);
            const double Q_conv = h_conv * params_.A_tread * (T_surf_[i] - T_air);
            const double A_contact = slip_info[i].Fz < 0.0
                ? params_.A_contact
                : slip_info[i].Fz / (params_.pressure_kpa * 1000.0);
            const double Q_cond = params_.h_road * A_contact * (T_surf_[i] - params_.T_road);

            if (two_node) {
                const double Q_sb = params_.G_surface_bulk * (T_surf_[i] - T_bulk_[i]);
                const double f = params_.surface_heat_fraction;
                T_surf_[i] += dt * (f * Q_gen - Q_conv - Q_cond - Q_sb)
                            / (params_.m_tread * params_.c_p);
                T_bulk_[i] += dt * ((1.0 - f) * Q_gen + Q_sb)
                            / (params_.m_bulk * params_.c_p_bulk);
                T_bulk_[i] = std::clamp(T_bulk_[i], -10.0, 150.0);
            } else {
                T_surf_[i] += dt * (Q_gen - Q_conv - Q_cond) / (params_.m_tread * params_.c_p);
            }
            T_surf_[i] = std::clamp(T_surf_[i], -10.0, 150.0);
        }
    }

    /// 指定輪の表面温度 [°C] (0=FL, 1=FR, 2=RL, 3=RR)
    double get_temperature(int wheel_index) const { return T_surf_[wheel_index]; }

    /// 指定輪のバルク温度 [°C]（1 ノード時は表面と同じ初期値のまま）
    double get_bulk_temperature(int wheel_index) const { return T_bulk_[wheel_index]; }

    /// 全輪の表面温度を取得
    const std::array<double, 4>& get_temperatures() const { return T_surf_; }

    /// 温度をリセット（T < 0 なら T_initial）
    void reset(double T = -1.0) {
        const double temp = (T < 0) ? params_.T_initial : T;
        T_surf_.fill(temp);
        T_bulk_.fill(temp);
    }

    /// パラメータを設定
    void set_params(const TireThermalParams& params) { params_ = params; }

private:
    TireThermalParams params_;
    std::array<double, 4> T_surf_{};  // 各輪の表面温度 [°C]
    std::array<double, 4> T_bulk_{};  // 各輪のバルク温度 [°C]
};

} // namespace fsae::physics::tire
