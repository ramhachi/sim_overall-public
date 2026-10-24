#pragma once
/// @file track_distance.hpp
/// @brief トラック距離クエリ（グローバル座標→最近接トラック点）
///
/// Phase 8a: MPC用のFrenet座標代替
/// グローバル(x,y)から最近接トラック中心線ポイントを高速検索
/// 8の字トラックの自己交差でも正しく動作

#include "track_model.hpp"
#include <vector>
#include <cmath>
#include <limits>

namespace fsae::track {

/// 最近接点クエリ結果
struct NearestResult {
    double s;           ///< 弧長パラメータ [m]
    double n;           ///< 横偏差 [m] (左=正、右=負)
    double theta;       ///< トラック接線方向 [rad]
    double kappa;       ///< 曲率 [1/m]
    double distance;    ///< ユークリッド距離 [m]
};

/// MPC参照軌道ポイント
struct ReferencePoint {
    double x;           ///< グローバルX [m]
    double y;           ///< グローバルY [m]
    double psi;         ///< ヘディング [rad]
    double vx;          ///< 目標速度 [m/s]
    double s;           ///< 弧長 [m]
    double kappa;       ///< 曲率 [1/m]
    double width_left;  ///< 左側幅 [m]
    double width_right; ///< 右側幅 [m]
};

/// トラック距離クエリ
/// CurvatureBasedTrackからサンプル点を事前計算し、
/// hint_s付き局所探索でO(1)の最近接点検索を実現
class TrackDistanceQuery {
public:
    /// @param track トラックモデル
    /// @param ds サンプル間隔 [m] (デフォルト0.5m)
    explicit TrackDistanceQuery(
        const CurvatureBasedTrack& track,
        double ds = 0.5)
        : track_(track)
        , ds_(ds)
        , track_length_(track.length())
    {
        // 等間隔サンプル点を生成
        int n_samples = static_cast<int>(std::ceil(track_length_ / ds_)) + 1;
        samples_.reserve(n_samples);

        for (int i = 0; i < n_samples; ++i) {
            double s = std::min(static_cast<double>(i) * ds_, track_length_);
            auto pt = track_.get_point(s);
            samples_.push_back({pt.s, pt.x, pt.y, pt.theta, pt.kappa,
                                pt.width_left, pt.width_right});
        }
    }

    /// 最近接トラックポイントを検索
    /// @param x グローバルX座標 [m]
    /// @param y グローバルY座標 [m]
    /// @param hint_s 前回のs値（局所探索用、-1で全探索）
    /// @return NearestResult
    NearestResult nearest(double x, double y, double hint_s = -1.0) const {
        if (samples_.empty()) {
            return {0, 0, 0, 0, 0};
        }

        int best_idx = 0;
        double best_dist2 = std::numeric_limits<double>::max();

        if (hint_s >= 0.0) {
            // 局所探索: hint_s付近 ±search_range のサンプルのみ
            constexpr double search_range = 10.0;  // ±10m
            int hint_idx = static_cast<int>(std::round(hint_s / ds_));
            int range_idx = static_cast<int>(std::ceil(search_range / ds_));

            int start = std::max(0, hint_idx - range_idx);
            int end = std::min(static_cast<int>(samples_.size()) - 1,
                               hint_idx + range_idx);

            // 周回トラック: 端をまたぐ探索も考慮
            for (int i = start; i <= end; ++i) {
                int idx = i;
                // 周回の場合はwrap
                if (idx < 0) idx += static_cast<int>(samples_.size());
                if (idx >= static_cast<int>(samples_.size()))
                    idx -= static_cast<int>(samples_.size());

                double dx = samples_[idx].x - x;
                double dy = samples_[idx].y - y;
                double d2 = dx * dx + dy * dy;
                if (d2 < best_dist2) {
                    best_dist2 = d2;
                    best_idx = idx;
                }
            }
        } else {
            // 全探索
            for (int i = 0; i < static_cast<int>(samples_.size()); ++i) {
                double dx = samples_[i].x - x;
                double dy = samples_[i].y - y;
                double d2 = dx * dx + dy * dy;
                if (d2 < best_dist2) {
                    best_dist2 = d2;
                    best_idx = i;
                }
            }
        }

        // 局所精密化: best_idx周辺で2次補間
        // 前後のサンプルで最小距離を求める
        double s_best = samples_[best_idx].s;
        double s_fine = s_best;
        double dist_fine2 = best_dist2;

        // ±ds範囲を10分割で精密探索
        double s_lo = std::max(0.0, s_best - ds_);
        double s_hi = std::min(track_length_, s_best + ds_);
        constexpr int n_refine = 10;
        double ds_fine = (s_hi - s_lo) / n_refine;

        for (int i = 0; i <= n_refine; ++i) {
            double s_test = s_lo + i * ds_fine;
            auto pt = track_.get_point(s_test);
            double dx = pt.x - x;
            double dy = pt.y - y;
            double d2 = dx * dx + dy * dy;
            if (d2 < dist_fine2) {
                dist_fine2 = d2;
                s_fine = s_test;
            }
        }

        // 最終点での情報取得
        auto pt = track_.get_point(s_fine);
        double dist = std::sqrt(dist_fine2);

        // 横偏差の符号: トラック法線に対する車両位置
        // 法線 = (-sin(theta), cos(theta)) → 左方向が正
        double dx = x - pt.x;
        double dy = y - pt.y;
        double n = -std::sin(pt.theta) * dx + std::cos(pt.theta) * dy;

        return {s_fine, n, pt.theta, pt.kappa, dist};
    }

    /// 参照軌道を生成（MPC予測ホライズン用）
    /// @param s_start 開始弧長 [m]
    /// @param velocity_profile 速度プロファイル
    /// @param N ホライズンステップ数
    /// @param dt 時間刻み [s]
    /// @return N+1個の参照点（現在 + N予測）
    std::vector<ReferencePoint> get_reference_trajectory(
        double s_start,
        const std::vector<std::pair<double, double>>& velocity_sv,
        int N,
        double dt) const
    {
        std::vector<ReferencePoint> ref;
        ref.reserve(N + 1);

        double s = s_start;

        for (int k = 0; k <= N; ++k) {
            // sを周回対応で正規化
            double s_norm = std::fmod(s, track_length_);
            if (s_norm < 0) s_norm += track_length_;

            auto pt = track_.get_point(s_norm);

            // 速度プロファイルから目標速度を補間
            double v_ref = interpolate_velocity(s_norm, velocity_sv);

            ref.push_back({pt.x, pt.y, pt.theta, v_ref,
                           s_norm, pt.kappa, pt.width_left, pt.width_right});

            // 次ステップのs: 目標速度で前進
            s += v_ref * dt;
        }

        return ref;
    }

    /// トラック長を取得
    double track_length() const { return track_length_; }

private:
    /// サンプル点の内部表現
    struct SamplePoint {
        double s, x, y, theta, kappa;
        double width_left, width_right;
    };

    /// 速度プロファイルの線形補間
    double interpolate_velocity(
        double s,
        const std::vector<std::pair<double, double>>& sv) const
    {
        if (sv.empty()) return 10.0;  // デフォルト10m/s
        if (s <= sv.front().first) return sv.front().second;
        if (s >= sv.back().first) return sv.back().second;

        for (size_t i = 0; i < sv.size() - 1; ++i) {
            if (s >= sv[i].first && s < sv[i+1].first) {
                double alpha = (s - sv[i].first) /
                               (sv[i+1].first - sv[i].first);
                return sv[i].second + alpha * (sv[i+1].second - sv[i].second);
            }
        }
        return sv.back().second;
    }

    const CurvatureBasedTrack& track_;
    double ds_;
    double track_length_;
    std::vector<SamplePoint> samples_;
};

} // namespace fsae::track
