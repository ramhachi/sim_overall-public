#pragma once
/// @file track_model.hpp
/// @brief トラックモデル（曲率ベース表現）
///
/// 総合ガイド §16.1 に基づく実装
/// トラックを弧長パラメータsと曲率κ(s)で表現

#include <vector>
#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace fsae::track {

/// トラック上の位置
struct TrackPoint {
    double s;          // 弧長 [m]
    double x;          // X座標 [m]
    double y;          // Y座標 [m]
    double theta;      // ヘディング角 [rad]
    double kappa;      // 曲率 [1/m]
    double width_left;  // 左端までの幅 [m]
    double width_right; // 右端までの幅 [m]
};

/// 曲率ベースのトラックモデル
class CurvatureBasedTrack {
public:
    /// @param curvature_data 曲率データ (s, κ)のペア
    /// @param x0 初期X座標 [m]
    /// @param y0 初期Y座標 [m]
    /// @param theta0 初期ヘディング角 [rad]
    /// @param track_width トラック幅 [m]
    CurvatureBasedTrack(
        std::vector<std::pair<double, double>> curvature_data,
        double x0 = 0.0,
        double y0 = 0.0,
        double theta0 = 0.0,
        double track_width = 3.0,
        bool force_loop = false)
        : curvature_data_(std::move(curvature_data))
        , x0_(x0)
        , y0_(y0)
        , theta0_(theta0)
        , track_width_(track_width)
        , is_loop_forced_(force_loop)
    {
        // 弧長でソート
        std::sort(curvature_data_.begin(), curvature_data_.end(),
                  [](const auto& a, const auto& b) { return a.first < b.first; });

        // トラックポイントを生成
        generate_track_points();
    }

    /// Explicit TrackPoint constructor (avoids integration drift)
    CurvatureBasedTrack(
        std::vector<TrackPoint> track_points,
        bool force_loop = false)
        : track_points_(std::move(track_points))
        , is_loop_forced_(force_loop)
    {
        if (track_points_.empty()) {
            throw std::runtime_error("Track points cannot be empty");
        }

        const auto& first = track_points_.front();
        x0_ = first.x;
        y0_ = first.y;
        theta0_ = first.theta;
        track_width_ = first.width_left + first.width_right; // Approx

        // Populate curvature_data_ for compatibility
        for (const auto& pt : track_points_) {
            curvature_data_.emplace_back(pt.s, pt.kappa);
        }
    }

    /// 弧長sでの曲率を取得（線形補間）
    double get_curvature(double s) const {
        if (curvature_data_.empty()) return 0.0;

        // 範囲外の場合
        if (s <= curvature_data_.front().first) return curvature_data_.front().second;
        if (s >= curvature_data_.back().first) return curvature_data_.back().second;

        // 線形補間
        for (size_t i = 0; i < curvature_data_.size() - 1; ++i) {
            const auto& p0 = curvature_data_[i];
            const auto& p1 = curvature_data_[i + 1];

            if (s >= p0.first && s < p1.first) {
                double alpha = (s - p0.first) / (p1.first - p0.first);
                return p0.second + alpha * (p1.second - p0.second);
            }
        }

        return curvature_data_.back().second;
    }

    /// 弧長sでのトラックポイントを取得
    TrackPoint get_point(double s) const {
        if (track_points_.empty()) {
            throw std::runtime_error("Track points not generated");
        }

        // 範囲外の場合
        if (s <= track_points_.front().s) return track_points_.front();
        if (s >= track_points_.back().s) return track_points_.back();

        // 線形補間
        for (size_t i = 0; i < track_points_.size() - 1; ++i) {
            const auto& p0 = track_points_[i];
            const auto& p1 = track_points_[i + 1];

            if (s >= p0.s && s < p1.s) {
                double alpha = (s - p0.s) / (p1.s - p0.s);

                TrackPoint pt;
                pt.s = s;
                pt.x = p0.x + alpha * (p1.x - p0.x);
                pt.y = p0.y + alpha * (p1.y - p0.y);
                pt.theta = p0.theta + alpha * (p1.theta - p0.theta);
                pt.kappa = p0.kappa + alpha * (p1.kappa - p0.kappa);
                pt.width_left = p0.width_left + alpha * (p1.width_left - p0.width_left);
                pt.width_right = p0.width_right + alpha * (p1.width_right - p0.width_right);

                return pt;
            }
        }

        return track_points_.back();
    }

    /// トラック全長
    double length() const {
        return track_points_.empty() ? 0.0 : track_points_.back().s;
    }

    /// 全トラックポイントを取得
    const std::vector<TrackPoint>& points() const { return track_points_; }

    /// トラックポイントを上書き（解析的計算用）
    std::vector<TrackPoint>& mutable_points() { return track_points_; }

    /// 周回トラック（閉じたループ）かどうかを判定
    bool is_closed_loop() const {
        if (is_loop_forced_) return true;
        if (curvature_data_.empty() || track_points_.empty()) return false;

        // 終点と始点の距離をチェック
        const auto& first_pt = track_points_.front();
        const auto& last_pt = track_points_.back();

        double dx = last_pt.x - first_pt.x;
        double dy = last_pt.y - first_pt.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // 距離が3m以内なら閉ループとみなす（オートクロスデータの不連続性に対応）
        constexpr double closure_threshold = 3.0;  // [m]
        if (distance < closure_threshold) {
            return true;
        }

        // フォールバック: 曲率の総積分が2πの整数倍かチェック
        // ∫κ(s)ds = Δθ (ヘディング角の変化)
        // 閉ループなら Δθ = 2πn (n: 整数)
        double total_angle_change = 0.0;
        for (size_t i = 0; i < curvature_data_.size() - 1; ++i) {
            const auto& p0 = curvature_data_[i];
            const auto& p1 = curvature_data_[i + 1];
            double ds = p1.first - p0.first;
            double kappa_avg = (p0.second + p1.second) / 2.0;
            total_angle_change += kappa_avg * ds;
        }

        // 2πの整数倍に近い（±10度以内）なら閉ループ
        double n_revolutions = total_angle_change / (2.0 * M_PI);
        double fractional_part = std::abs(n_revolutions - std::round(n_revolutions));
        constexpr double angle_tolerance = 10.0 * M_PI / 180.0;  // 10度
        return (fractional_part * 2.0 * M_PI) < angle_tolerance;
    }

private:
    /// トラックポイントを生成（積分により座標を計算）
    void generate_track_points() {
        if (curvature_data_.empty()) return;

        track_points_.clear();

        double s = 0.0;
        double x = x0_;
        double y = y0_;
        double theta = theta0_;

        const double ds = 0.1;  // 積分ステップ [m]
        double s_max = curvature_data_.back().first;

        while (s <= s_max) {
            double kappa = get_curvature(s);

            TrackPoint pt;
            pt.s = s;
            pt.x = x;
            pt.y = y;
            pt.theta = theta;
            pt.kappa = kappa;
            pt.width_left = track_width_ / 2.0;
            pt.width_right = track_width_ / 2.0;

            track_points_.push_back(pt);

            // 積分（オイラー法）
            // θ(s) = θ₀ + ∫κ(s')ds'
            // x(s) = x₀ + ∫cos(θ(s'))ds'
            // y(s) = y₀ + ∫sin(θ(s'))ds'
            theta += kappa * ds;
            x += std::cos(theta) * ds;
            y += std::sin(theta) * ds;
            s += ds;
        }

        // 周回トラックの場合、終点を開始点と一致させる
        // （数値積分の累積誤差を補正）
        if (is_closed_loop()) {
            if (!track_points_.empty()) {
                auto& last_point = track_points_.back();
                double error_x = last_point.x - x0_;
                double error_y = last_point.y - y0_;
                double total_s = last_point.s;

                // 誤差をsに応じて線形配分
                for (auto& pt : track_points_) {
                    double ratio = (total_s > 1e-6) ? (pt.s / total_s) : 0.0;
                    pt.x -= error_x * ratio;
                    pt.y -= error_y * ratio;
                }

                // 終点の方向を開始点と一致させる（重要！）
                // θは2π回転しているので、θ₀ + 2πになっているはずだが、
                // 正規化して θ₀ と一致させる
                double d_theta = last_point.theta - theta0_;
                // 2πの整数倍に丸める
                double n_revolutions = std::round(d_theta / (2.0 * M_PI));
                last_point.theta = theta0_ + n_revolutions * 2.0 * M_PI;
            }
        }
    }

    std::vector<std::pair<double, double>> curvature_data_;  // (s, κ)
    std::vector<TrackPoint> track_points_;
    double x0_, y0_, theta0_;
    double track_width_;
    bool is_loop_forced_ = false;
};

/// 円形トラックを生成（解析的計算版）
/// 円の中心を原点(0, 0)に配置、周回の連続性を保証
/// 車両は(R, 0)から北向き(θ=π/2)でスタート、反時計回り
///
/// 解析解:
///   angle = s / R  (0 → 2π)
///   x(s) = R * cos(angle)
///   y(s) = R * sin(angle)
///   θ(s) = angle + π/2  (円の接線方向)
inline CurvatureBasedTrack create_circle_track(double radius, double track_width = 3.0) {
    double kappa = 1.0 / radius;
    double circumference = 2.0 * M_PI * radius;

    // 曲率データ（一定曲率）
    std::vector<std::pair<double, double>> curvature_data;
    int n_points = 100;
    for (int i = 0; i <= n_points; ++i) {
        double s = (circumference * i) / n_points;
        curvature_data.emplace_back(s, kappa);
    }

    // 解析的にトラックポイントを生成するため、
    // ダミーの初期条件でCurvatureBasedTrackを作り、その後ポイントを上書きする
    CurvatureBasedTrack track(curvature_data, radius, 0.0, M_PI / 2.0, track_width);

    // 解析解でトラックポイントを上書き
    auto& points = track.mutable_points();
    for (auto& pt : points) {
        double angle = pt.s / radius;  // 0 → 2π
        pt.x = radius * std::cos(angle);
        pt.y = radius * std::sin(angle);
        pt.theta = angle + M_PI / 2.0;  // 接線方向（反時計回り）
        // pt.s, pt.kappa, pt.width_left, pt.width_right はそのまま
    }

    return track;
}

/// CW（時計回り）円形トラックを生成
inline CurvatureBasedTrack create_circle_cw_track(double radius, double track_width = 3.0) {
    double kappa = -1.0 / radius;  // 負の曲率 = CW
    double circumference = 2.0 * M_PI * radius;

    std::vector<std::pair<double, double>> curvature_data;
    int n_points = 100;
    for (int i = 0; i <= n_points; ++i) {
        double s = (circumference * i) / n_points;
        curvature_data.emplace_back(s, kappa);
    }

    CurvatureBasedTrack track(curvature_data, radius, 0.0, -M_PI / 2.0, track_width);

    auto& points = track.mutable_points();
    for (auto& pt : points) {
        double angle = -pt.s / radius;  // 0 → -2π (CW)
        pt.x = radius * std::cos(angle);
        pt.y = radius * std::sin(angle);
        pt.theta = angle - M_PI / 2.0;  // CW接線方向
    }

    return track;
}

/// 8の字トラックを生成（縦並び2円版、解析的計算）
/// ※ レガシー互換用。新規使用にはcreate_skidpad_trackを推奨
inline CurvatureBasedTrack create_figure_eight_track(double radius, double track_width = 3.0) {
    std::vector<std::pair<double, double>> curvature_data;

    double kappa = 1.0 / radius;
    double half_circumference = M_PI * radius;
    
    int n_points = 100;
    for (int i = 0; i <= n_points; ++i) {
        double s = (2.0 * half_circumference * i) / n_points;
        curvature_data.emplace_back(s, kappa);
    }

    CurvatureBasedTrack track(curvature_data, radius, 0.0, M_PI / 2.0, track_width);

    auto& points = track.mutable_points();
    for (auto& pt : points) {
        double s = pt.s;
        if (s <= half_circumference) {
            double angle = s / radius;
            pt.x = radius * std::cos(angle);
            pt.y = radius + radius * std::sin(angle);
            pt.theta = angle + M_PI / 2.0;
            pt.kappa = kappa;
        } else {
            double s2 = s - half_circumference;
            double angle = s2 / radius + M_PI;
            pt.x = radius * std::cos(angle);
            pt.y = -radius + radius * std::sin(angle);
            pt.theta = angle + M_PI / 2.0;
            pt.kappa = kappa;
        }
    }

    if (!points.empty()) {
        auto& last_point = points.back();
        const auto& first_point = points.front();
        last_point.x = first_point.x;
        last_point.y = first_point.y;
        last_point.theta = first_point.theta + 2.0 * M_PI;
    }

    return track;
}

/// Formula SAE スキッドパッドトラック（水平figure-8、解析的計算）
/// SAE公式仕様に基づく水平配置の8の字コース
///
/// 構成:
///   右円: 中心(+R, 0)、半径R、時計回り (κ = -1/R)
///   左円: 中心(-R, 0)、半径R、反時計回り (κ = +1/R)
///   自己交差点: (0, 0)
///   スタート: (0, 0)、北向き (θ = π/2)
///   トラック全長: 4πR（各円を完全に1周）
///
/// SAE標準寸法 (radius = 9.125m):
///   内側円直径: 15.25m、外側円直径: 21.25m、レーン幅: 3.0m
///   中心間距離: 18.25m
inline CurvatureBasedTrack create_skidpad_track(double radius = 9.125, double track_width = 3.0) {
    double full_circumference = 2.0 * M_PI * radius;  // 1円の周長
    double total_length = 2.0 * full_circumference;    // 全長 = 4πR

    // 曲率データ: 右円CW(κ<0) → 左円CCW(κ>0)
    std::vector<std::pair<double, double>> curvature_data;
    int n_points = 200;
    for (int i = 0; i <= n_points; ++i) {
        double s = (total_length * i) / n_points;
        double kappa = (s <= full_circumference) ? (-1.0 / radius) : (1.0 / radius);
        curvature_data.emplace_back(s, kappa);
    }

    // ダミー初期条件でトラック作成（後で解析解で上書き）
    CurvatureBasedTrack track(curvature_data, 0.0, 0.0, M_PI / 2.0, track_width);

    // MATLAB検証済みの解析解でトラックポイントを上書き
    auto& points = track.mutable_points();
    for (auto& pt : points) {
        double s = pt.s;

        if (s <= full_circumference) {
            // 右円: 中心(+R, 0)、時計回り
            // (0,0)は中心からα=πの位置、CWなのでαが減少: π → -π
            double alpha = M_PI - s / radius;       // π → -π
            pt.x = radius + radius * std::cos(alpha);
            pt.y = radius * std::sin(alpha);
            pt.theta = alpha - M_PI / 2.0;          // CW接線方向
            pt.kappa = -1.0 / radius;
        } else {
            // 左円: 中心(-R, 0)、反時計回り
            // (0,0)は中心からα=0の位置、CCWなのでαが増加: 0 → 2π
            double s2 = s - full_circumference;
            double alpha = s2 / radius;              // 0 → 2π
            pt.x = -radius + radius * std::cos(alpha);
            pt.y = radius * std::sin(alpha);
            pt.theta = alpha + M_PI / 2.0;           // CCW接線方向
            pt.kappa = 1.0 / radius;
        }
    }

    // 終点を始点に正確に一致させる（閉ループ保証）
    if (!points.empty()) {
        auto& last_point = points.back();
        const auto& first_point = points.front();
        last_point.x = first_point.x;
        last_point.y = first_point.y;
        // ヘディング角: 右円CW -2π + 左円CCW +2π = 0回転
        // θ(0) = π/2, θ(end) = 2π + π/2 = 5π/2  → Δθ = 2π (1回転分の蓄積)
        last_point.theta = first_point.theta;
    }

    return track;
}

/// 直線+コーナーのトラック
inline CurvatureBasedTrack create_straight_corner_track(
    double straight_length,
    double corner_radius,
    double track_width = 3.0)
{
    std::vector<std::pair<double, double>> curvature_data;

    // 直線部
    curvature_data.emplace_back(0.0, 0.0);
    curvature_data.emplace_back(straight_length, 0.0);

    // コーナー（90度）
    double corner_length = M_PI * corner_radius / 2.0;
    double kappa_corner = 1.0 / corner_radius;

    int n_corner = 25;
    for (int i = 0; i <= n_corner; ++i) {
        double s = straight_length + (corner_length * i) / n_corner;
        curvature_data.emplace_back(s, kappa_corner);
    }

    // 次の直線
    double s_end = straight_length + corner_length;
    curvature_data.emplace_back(s_end + straight_length, 0.0);

    return CurvatureBasedTrack(curvature_data, 0.0, 0.0, 0.0, track_width);
}

/// Formula SAE Acceleration Event (75m直線加速コース)
/// SAE公式仕様: 75m (246 feet) の完全直線コース
///
/// 仕様:
///   - 距離: 75.0m (SAE標準)
///   - 幅: 5.0m (SAE最小4.9mに余裕を持たせた値)
///   - スタート: (0, 0)、東向き (θ = 0)
///   - ゴール: (75, 0)
///   - 曲率: κ = 0 (完全直線)
///
/// 目的:
///   スタンディングスタートからの直線加速性能評価
inline CurvatureBasedTrack create_acceleration_track(
    double length = 75.0,
    double track_width = 5.0)
{
    // 曲率データ: 完全直線 (κ = 0)
    std::vector<std::pair<double, double>> curvature_data;
    curvature_data.emplace_back(0.0, 0.0);
    curvature_data.emplace_back(length, 0.0);
    
    // 東向きスタート (θ = 0)
    return CurvatureBasedTrack(
        curvature_data,
        0.0,        // x0
        0.0,        // y0
        0.0,        // theta0 (東向き)
        track_width
    );
}

} // namespace fsae::track
