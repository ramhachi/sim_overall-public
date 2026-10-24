#pragma once
/// @file engine.hpp
/// @brief エンジントルクマップモデル（CBR600RR等の高回転バイクエンジン対応）

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <utility>

namespace fsae::physics::powertrain {

/// CBR600RR 13 点 WOT トルクカーブ（vehicle_params の既定値もこれを参照すること）
inline const std::vector<double> kCbr600rrRpm = {
    1000, 3000, 5000, 7000, 9000, 10000, 11000, 12000, 12500, 13000, 13500, 14000, 14500};
inline const std::vector<double> kCbr600rrTorque = {
    35, 40, 48, 54, 60, 61, 62, 62.5, 63, 62, 60, 57, 55};

/// エンジントルクマップ
/// WOT 曲線 + 閉スロットル曲線（または 2D 部分負荷マップ）を線形 / bilinear 補間
class EngineTorqueMap {
public:
    EngineTorqueMap() = default;

    /// @brief トルクカーブを設定
    /// @param rpm_points RPM点のリスト（昇順でなければならない）
    /// @param torque_points 各RPMでのトルク値 [Nm]
    void set_curve(const std::vector<double>& rpm_points,
                   const std::vector<double>& torque_points) {
        if (rpm_points.size() != torque_points.size()) {
            throw std::invalid_argument("RPM points and torque points must have the same size");
        }
        if (rpm_points.size() < 2) {
            throw std::invalid_argument("At least 2 points required for interpolation");
        }

        rpm_points_ = rpm_points;
        torque_points_ = torque_points;
        closed_points_.clear();
        throttle_points_.clear();
        table_.clear();

        // RPMが昇順かチェック
        for (size_t i = 1; i < rpm_points_.size(); ++i) {
            if (rpm_points_[i] <= rpm_points_[i-1]) {
                throw std::invalid_argument("RPM points must be in strictly ascending order");
            }
        }
    }

    /// @brief 2D 部分負荷マップ T(rpm, throttle) を設定（ダイノ / ECU マップ）。set_curve() の後に呼ぶ。
    /// @param throttle_points スロットル開度 [0-1]（狭義昇順, >=2 点）
    /// @param torque_table rpm_points 行 × throttle_points 列 [Nm]（閉スロットル列は負でよい）
    /// 設定時は 1D×スロットル混合より優先する。
    void set_throttle_table(const std::vector<double>& throttle_points,
                            const std::vector<std::vector<double>>& torque_table) {
        if (throttle_points.size() < 2 || torque_table.size() != rpm_points_.size()) {
            throw std::invalid_argument("torque_table needs rpm_points rows and >=2 throttle_points");
        }
        for (size_t i = 1; i < throttle_points.size(); ++i) {
            if (throttle_points[i] <= throttle_points[i - 1]) {
                throw std::invalid_argument("throttle_points must be strictly ascending");
            }
        }
        std::vector<double> flat;
        for (const auto& row : torque_table) {
            if (row.size() != throttle_points.size()) {
                throw std::invalid_argument("torque_table row size must equal throttle_points size");
            }
            flat.insert(flat.end(), row.begin(), row.end());
        }
        throttle_points_ = throttle_points;
        table_ = std::move(flat);
    }

    /// @brief 閉スロットル（モータリング）トルク曲線 [Nm]（負値, rpm_points と同サイズ）。set_curve() の後に呼ぶ。
    void set_closed_curve(const std::vector<double>& torque_closed) {
        if (torque_closed.size() != rpm_points_.size()) {
            throw std::invalid_argument("torque_closed_curve must match rpm_points size");
        }
        closed_points_ = torque_closed;
    }

    /// @brief エンジントルク [Nm]
    /// - 2D テーブルがあれば bilinear 補間
    /// - 無ければ T = thr·T_WOT(rpm) + (1−thr)·T_closed(rpm)
    /// - rpm > rpm_points.back()：レブリミッタ（燃料カット）。燃焼トルク 0、摩擦/ポンプ損失 T_closed は残す
    /// - rpm < rpm_points.front()：先頭点にクランプ
    double get_torque(double rpm, double throttle = 1.0) const {
        if (rpm_points_.empty()) {
            return 0.0;
        }
        throttle = std::clamp(throttle, 0.0, 1.0);
        if (rpm > rpm_points_.back()) {
            return closed_torque(rpm_points_.back());
        }
        rpm = std::max(rpm, rpm_points_.front());
        if (!table_.empty()) {
            const auto [i, wr] = locate(rpm_points_, rpm);
            const auto [j, wt] = locate(throttle_points_, throttle);
            const size_t n = throttle_points_.size();
            auto at = [&](size_t a, size_t b) { return table_[a * n + b]; };
            return (1 - wr) * ((1 - wt) * at(i, j) + wt * at(i, j + 1))
                 + wr * ((1 - wt) * at(i + 1, j) + wt * at(i + 1, j + 1));
        }
        return throttle * interpolate(torque_points_, rpm) + (1.0 - throttle) * closed_torque(rpm);
    }

    /// @brief 閉スロットルトルク [Nm]（負）
    /// torque_closed_curve 未設定時の既定は物理的推定（実測ではない）：
    ///   599 cc 4 ストロークのモータリング MEP（摩擦 + 閉スロットルのポンプ損失）を
    ///   ピークトルク回転で ≈1.05 bar と仮定 → T = MEP·Vd/(4π) ≈ 5.0 Nm ≈ 0.08·T_peak。
    ///   摩擦は流体潤滑支配で回転数にほぼ比例とし T_closed = −0.08·T_peak·rpm/rpm_peak。
    double closed_torque(double rpm) const {
        if (rpm_points_.empty()) return 0.0;
        if (!closed_points_.empty()) {
            return interpolate(closed_points_, rpm);
        }
        static bool logged = false;  // ponytail: process-wide once flag, per-instance if multi-engine logs matter
        if (!logged) {
            logged = true;
            std::cerr << "[engine] torque_closed_curve not set: using motoring estimate "
                         "T_closed = -0.08*T_peak*rpm/rpm_peak (not measured)" << std::endl;
        }
        return -0.08 * get_peak_torque() * rpm / get_peak_torque_rpm();
    }

    /// @brief ピークトルクRPMを取得
    double get_peak_torque_rpm() const {
        if (torque_points_.empty()) return 0.0;
        
        auto max_it = std::max_element(torque_points_.begin(), torque_points_.end());
        size_t idx = std::distance(torque_points_.begin(), max_it);
        return rpm_points_[idx];
    }
    
    /// @brief ピークトルク値を取得
    double get_peak_torque() const {
        if (torque_points_.empty()) return 0.0;
        return *std::max_element(torque_points_.begin(), torque_points_.end());
    }
    
    /// @brief アイドリングRPMを取得
    double get_idle_rpm() const {
        return rpm_points_.empty() ? 1000.0 : rpm_points_.front();
    }
    
    /// @brief レッドラインRPMを取得
    double get_redline_rpm() const {
        return rpm_points_.empty() ? 14500.0 : rpm_points_.back();
    }

private:
    std::vector<double> rpm_points_;
    std::vector<double> torque_points_;
    std::vector<double> closed_points_;     // 閉スロットル曲線（空 = 既定推定）
    std::vector<double> throttle_points_;   // 2D マップのスロットル軸（空 = 1D 混合）
    std::vector<double> table_;             // rpm 行 × throttle 列（行優先）

    /// 区間インデックスと重み（範囲外は端にクランプ）
    static std::pair<size_t, double> locate(const std::vector<double>& x, double v) {
        if (v <= x.front()) return {0, 0.0};
        if (v >= x.back()) return {x.size() - 2, 1.0};
        const size_t i = static_cast<size_t>(std::upper_bound(x.begin(), x.end(), v) - x.begin()) - 1;
        return {i, (v - x[i]) / (x[i + 1] - x[i])};
    }

    /// rpm_points_ 上の 1D 線形補間
    double interpolate(const std::vector<double>& y, double rpm) const {
        const auto [i, w] = locate(rpm_points_, rpm);
        return y[i] + w * (y[i + 1] - y[i]);
    }
};

/// CBR600RRの標準トルクカーブ
/// パワーバンド: 8,000-14,000 rpm
/// ピークトルク: 約63 Nm @ 12,500 rpm
inline EngineTorqueMap create_cbr600rr_engine() {
    EngineTorqueMap engine;
    engine.set_curve(kCbr600rrRpm, kCbr600rrTorque);
    return engine;
}

/// トランスミッションギア比
struct GearRatios {
    std::vector<double> ratios;  // 各ギアの減速比（1速〜6速）
    double primary_ratio;         // 一次減速比（クランク→ミッション）
    double final_ratio;           // 最終減速比（スプロケット）
    
    /// CBR600RR標準ギア比
    static GearRatios cbr600rr_default() {
        return {
            {2.75, 2.00, 1.63, 1.43, 1.30, 1.21},  // 1-6速
            1.90,    // 一次減速比
            3.50     // 最終減速比（チェーン）
        };
    }
    
    /// 総減速比を計算
    /// @param gear ギア位置 (1-6, 0=ニュートラル)
    double get_total_ratio(int gear) const {
        if (gear <= 0 || gear > static_cast<int>(ratios.size())) {
            return 0.0;  // ニュートラルまたは範囲外
        }
        return ratios[gear - 1] * primary_ratio * final_ratio;
    }
    
    /// エンジンRPMから車輪回転数を計算
    /// @param engine_rpm エンジン回転数 [rpm]
    /// @param gear ギア位置 (1-6)
    /// @return 車輪回転数 [rad/s]
    double engine_rpm_to_wheel_omega(double engine_rpm, int gear) const {
        double total_ratio = get_total_ratio(gear);
        if (total_ratio == 0.0) return 0.0;
        
        // rpm → rad/s 変換: rpm * (2π/60) / ratio
        return (engine_rpm * 2.0 * M_PI / 60.0) / total_ratio;
    }
    
    /// 車輪回転数からエンジンRPMを計算
    /// @param wheel_omega 車輪回転数 [rad/s]
    /// @param gear ギア位置 (1-6)
    /// @return エンジン回転数 [rpm]
    double wheel_omega_to_engine_rpm(double wheel_omega, int gear) const {
        double total_ratio = get_total_ratio(gear);
        if (total_ratio == 0.0) return 0.0;
        
        // rad/s → rpm 変換: omega * ratio * (60/2π)
        return wheel_omega * total_ratio * 60.0 / (2.0 * M_PI);
    }
};

} // namespace fsae::physics::powertrain
