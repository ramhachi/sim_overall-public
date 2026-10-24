#pragma once
/// @file waypoint_loader.hpp
/// @brief ウェイポイント列からトラックを生成
///
/// CSVファイルやベクトルからトラックモデルを構築

#include "track_model.hpp"
#include <vector>
#include <fstream>
#include <sstream>
#include <string>
#include <cmath>

namespace fsae::track {

/// ウェイポイント
struct Waypoint {
    double x;       // X座標 [m]
    double y;       // Y座標 [m]
    double width;   // トラック幅 [m]（オプション、デフォルト3.0）
};

/// ウェイポイント列からトラックを生成
class WaypointTrackBuilder {
public:
    /// @param waypoints ウェイポイント列
    /// @param default_width デフォルトトラック幅 [m]
    WaypointTrackBuilder(std::vector<Waypoint> waypoints, double default_width = 3.0)
        : waypoints_(std::move(waypoints))
        , default_width_(default_width)
    {}

    /// トラックモデルを構築
    /// ウェイポイント間の距離と角度から曲率を推定
    CurvatureBasedTrack build() const {
        if (waypoints_.size() < 2) {
            throw std::runtime_error("At least 2 waypoints required");
        }

        bool force_loop = false;
        if (waypoints_.size() >= 2) {
            const auto& first = waypoints_.front();
            const auto& last = waypoints_.back();
            double dx = first.x - last.x;
            double dy = first.y - last.y;
            if (std::sqrt(dx * dx + dy * dy) < 0.001) {
                force_loop = true;
            }
        }

        std::vector<TrackPoint> track_points;
        double s = 0.0;
        
        // Loop through waypoints to create TrackPoints
        for (size_t i = 0; i < waypoints_.size(); ++i) {
            const auto& wp = waypoints_[i];
            
            TrackPoint pt;
            pt.x = wp.x;
            pt.y = wp.y;
            pt.width_left = (wp.width > 0 ? wp.width : default_width_) / 2.0;
            pt.width_right = pt.width_left;
            
            // Calculate distance from previous point
            if (i > 0) {
                const auto& prev = waypoints_[i-1];
                double dx = wp.x - prev.x;
                double dy = wp.y - prev.y;
                s += std::sqrt(dx * dx + dy * dy);
            }
            pt.s = s;

            // Calculate theta and kappa
            double theta = 0.0;
            double kappa = 0.0;
            
            // Need prev and next points for curvature/theta
            const Waypoint* p_prev = nullptr;
            const Waypoint* p_next = nullptr;
            bool can_compute = false;

            if (i > 0 && i < waypoints_.size() - 1) {
                p_prev = &waypoints_[i-1];
                p_next = &waypoints_[i+1];
                can_compute = true;
            } else if (force_loop) {
                if (i == 0) {
                    p_prev = &waypoints_[waypoints_.size() - 2];
                    p_next = &waypoints_[i+1];
                    can_compute = true;
                } else if (i == waypoints_.size() - 1) {
                    p_prev = &waypoints_[i-1];
                    p_next = &waypoints_[1]; // Wrap to second point
                    can_compute = true;
                }
            } else {
                // Open track endpoints
                if (i == 0) {
                    p_next = &waypoints_[i+1];
                    double dx = p_next->x - wp.x;
                    double dy = p_next->y - wp.y;
                    theta = std::atan2(dy, dx);
                } else if (i == waypoints_.size() - 1) {
                    p_prev = &waypoints_[i-1];
                    double dx = wp.x - p_prev->x;
                    double dy = wp.y - p_prev->y;
                    theta = std::atan2(dy, dx);
                }
            }

            if (can_compute) {
                double dx_prev = wp.x - p_prev->x;
                double dy_prev = wp.y - p_prev->y;
                double dx_next = p_next->x - wp.x;
                double dy_next = p_next->y - wp.y;
                
                double theta_prev = std::atan2(dy_prev, dx_prev);
                double theta_next = std::atan2(dy_next, dx_next);
                
                theta = normalize_angle(theta_prev * 0.5 + theta_next * 0.5); // Average heading
                // Be careful with angle averaging around pi/-pi
                double sin_avg = (std::sin(theta_prev) + std::sin(theta_next)) / 2.0;
                double cos_avg = (std::cos(theta_prev) + std::cos(theta_next)) / 2.0;
                theta = std::atan2(sin_avg, cos_avg);

                double d_theta = normalize_angle(theta_next - theta_prev);
                double s_segment = std::sqrt(dx_prev*dx_prev + dy_prev*dy_prev) 
                                 + std::sqrt(dx_next*dx_next + dy_next*dy_next);
                
                if (s_segment > 1e-6) {
                    kappa = d_theta / (s_segment * 0.5); // Approximate
                }
            }
            
            pt.theta = theta;
            pt.kappa = kappa;
            
            track_points.push_back(pt);
        }

        // Apply moving average smoothing to curvature to reduce noise
        if (!track_points.empty()) {
            std::vector<double> smoothed_kappa(track_points.size());
            int window_size = 5; 
            int half_window = window_size / 2;

            for (size_t i = 0; i < track_points.size(); ++i) {
                double sum = 0.0;
                int count = 0;
                
                for (int j = -half_window; j <= half_window; ++j) {
                    int idx = static_cast<int>(i) + j;
                    
                    if (force_loop) {
                        // Wrap index for closed loops
                        if (idx < 0) idx += track_points.size();
                        if (idx >= static_cast<int>(track_points.size())) idx -= track_points.size();
                    } else {
                        // Clamp index for open tracks
                        if (idx < 0) idx = 0;
                        if (idx >= static_cast<int>(track_points.size())) idx = track_points.size() - 1;
                    }

                    sum += track_points[idx].kappa;
                    count++;
                }
                
                smoothed_kappa[i] = sum / count;
            }

            // Apply smoothed values
            for (size_t i = 0; i < track_points.size(); ++i) {
                track_points[i].kappa = smoothed_kappa[i];
            }
        }
        
        return CurvatureBasedTrack(track_points, force_loop);
    }

    /// CSVファイルからウェイポイントを読み込み
    /// フォーマット: x,y[,width]
    static std::vector<Waypoint> load_from_csv(const std::string& filename, double default_width = 3.0) {
        std::vector<Waypoint> waypoints;
        std::ifstream file(filename);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        std::string line;
        // ヘッダーをスキップ
        if (std::getline(file, line) && line.find("x") != std::string::npos) {
            // ヘッダー行をスキップ
        } else {
            // ヘッダーがない場合は最初に戻る
            file.clear();
            file.seekg(0);
        }

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            Waypoint wp{};
            wp.width = default_width;

            std::string token;
            int col = 0;

            while (std::getline(iss, token, ',')) {
                try {
                    double value = std::stod(token);
                    if (col == 0) wp.x = value;
                    else if (col == 1) wp.y = value;
                    else if (col == 2) wp.width = value;
                    col++;
                } catch (...) {
                    // パースエラーは無視
                }
            }

            if (col >= 2) {  // x, yが最低限必要
                waypoints.push_back(wp);
            }
        }

        // Filter out closely-spaced waypoints to prevent noisy theta computation
        // (Some track CSVs have waypoints separated by only millimeters, which
        //  causes atan2-based heading to jump wildly between adjacent points.)
        constexpr double MIN_WAYPOINT_SPACING = 0.5; // [m]
        if (waypoints.size() > 2) {
            std::vector<Waypoint> filtered;
            filtered.push_back(waypoints.front());
            for (size_t i = 1; i < waypoints.size(); ++i) {
                double dx = waypoints[i].x - filtered.back().x;
                double dy = waypoints[i].y - filtered.back().y;
                double dist = std::sqrt(dx * dx + dy * dy);
                if (dist >= MIN_WAYPOINT_SPACING) {
                    filtered.push_back(waypoints[i]);
                }
            }
            // Always keep the last waypoint for loop closure check
            if (filtered.back().x != waypoints.back().x ||
                filtered.back().y != waypoints.back().y) {
                filtered.push_back(waypoints.back());
            }
            waypoints = std::move(filtered);
        }

        // ループを閉じる処理
        if (waypoints.size() >= 2) {
            const auto& first = waypoints.front();
            const auto& last = waypoints.back();
            double dx = first.x - last.x;
            double dy = first.y - last.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            // 始点と終点が近く(5m以内)、かつ完全に一致していない場合は閉じる
            if (dist < 5.0 && dist > 0.001) {
                waypoints.push_back(first);
            }
        }

        return waypoints;
    }

private:
    static double normalize_angle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    std::vector<Waypoint> waypoints_;
    double default_width_;
};

} // namespace fsae::track
