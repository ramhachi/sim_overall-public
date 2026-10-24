#pragma once
/// @file engine.hpp
/// @brief エンジントルクマップモデル（CBR600RR等の高回転バイクエンジン対応）

#include <vector>
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fsae::physics::powertrain {

/// エンジントルクマップ
/// RPMに対するトルク出力を線形補間で計算
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
        
        // RPMが昇順かチェック
        for (size_t i = 1; i < rpm_points_.size(); ++i) {
            if (rpm_points_[i] <= rpm_points_[i-1]) {
                throw std::invalid_argument("RPM points must be in strictly ascending order");
            }
        }
    }
    
    /// @brief 指定RPMでのエンジントルクを計算（線形補間）
    /// @param rpm エンジン回転数 [rpm]
    /// @param throttle スロットル開度 [0-1]
    /// @return エンジントルク [Nm]
    double get_torque(double rpm, double throttle = 1.0) const {
        if (rpm_points_.empty()) {
            return 0.0;
        }
        
        // スロットルを0-1にクランプ
        throttle = std::clamp(throttle, 0.0, 1.0);
        
        // RPMを範囲内にクランプ
        rpm = std::clamp(rpm, rpm_points_.front(), rpm_points_.back());
        
        // 線形補間
        double torque_max = interpolate(rpm);
        
        // スロットル開度を考慮（簡易モデル：線形）
        return torque_max * throttle;
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
    
    /// 線形補間
    double interpolate(double rpm) const {
        // 範囲外チェック（既にクランプされている前提）
        if (rpm <= rpm_points_.front()) return torque_points_.front();
        if (rpm >= rpm_points_.back()) return torque_points_.back();
        
        // 二分探索で区間を特定
        auto it = std::lower_bound(rpm_points_.begin(), rpm_points_.end(), rpm);
        size_t idx = std::distance(rpm_points_.begin(), it);
        
        if (idx == 0) return torque_points_[0];
        
        // 線形補間
        double rpm0 = rpm_points_[idx - 1];
        double rpm1 = rpm_points_[idx];
        double torque0 = torque_points_[idx - 1];
        double torque1 = torque_points_[idx];
        
        double t = (rpm - rpm0) / (rpm1 - rpm0);
        return torque0 + t * (torque1 - torque0);
    }
};

/// CBR600RRの標準トルクカーブ
/// パワーバンド: 8,000-14,000 rpm
/// ピークトルク: 約63 Nm @ 12,500 rpm
inline EngineTorqueMap create_cbr600rr_engine() {
    EngineTorqueMap engine;
    
    // RPMポイント
    std::vector<double> rpm = {
        1000,   // アイドリング
        3000,
        5000,
        7000,
        9000,
        10000,
        11000,
        12000,
        12500,  // ピーク
        13000,
        13500,
        14000,
        14500   // レッドライン
    };
    
    // トルク [Nm]
    std::vector<double> torque = {
        35,     // アイドリング
        40,
        48,
        54,
        60,
        61,
        62,
        62.5,
        63,     // ピーク
        62,
        60,
        57,
        55      // レッドライン
    };
    
    engine.set_curve(rpm, torque);
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
