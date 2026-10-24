#pragma once
/// @file waypoint_loader.hpp
/// @brief ウェイポイント列からトラックを生成
///
/// CSVファイルやベクトルからトラックモデルを構築

#include "track_model.hpp"
#include "../math/spline.hpp"
#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace fsae::track {

/// ウェイポイント
struct Waypoint {
    double x;       // X座標 [m]
    double y;       // Y座標 [m]
    double width;   // トラック幅 [m]（オプション、デフォルト3.0）
    double z = 0.0;                  // 路面高さ [m]
    double bank = 0.0;               // [rad]
    double grade = std::numeric_limits<double>::quiet_NaN(); // [rad], NaN → z から導出
    double mu_scale = 1.0;           // [-]
};

/// ウェイポイント列からトラックを生成
///
/// XY を弦長パラメータの 3 次スプラインで補間し、解析的曲率
/// κ = (x'y'' - y'x'') / (x'^2 + y'^2)^1.5 を kResampleDs 間隔で再サンプリングする。
/// 移動平均を使わないためヘアピン/スラロームのピーク κ を保持する（Issue #25）。
class WaypointTrackBuilder {
public:
    static constexpr double kResampleDs = 0.2;  // [m]

    /// @param waypoints ウェイポイント列（閉ループは先頭点を末尾に重複させる）
    /// @param default_width デフォルトトラック幅 [m]
    WaypointTrackBuilder(std::vector<Waypoint> waypoints, double default_width = 3.0)
        : waypoints_(std::move(waypoints))
        , default_width_(default_width)
    {}

    /// トラックモデルを構築
    CurvatureBasedTrack build() const {
        if (waypoints_.size() < 2) {
            throw std::runtime_error("At least 2 waypoints required");
        }
        const auto& first = waypoints_.front();
        const auto& last = waypoints_.back();
        const bool closed = std::hypot(first.x - last.x, first.y - last.y) < 0.001;

        // 弦長パラメータ
        const std::size_t n = waypoints_.size();
        std::vector<double> t(n, 0.0);
        for (std::size_t i = 1; i < n; ++i) {
            t[i] = t[i - 1] + std::hypot(waypoints_[i].x - waypoints_[i - 1].x,
                                         waypoints_[i].y - waypoints_[i - 1].y);
        }
        const double total = t.back();
        if (total <= 1e-9) throw std::runtime_error("Degenerate waypoints");

        // 閉ループは両端に周回分のパッドを付けて自然スプラインの端点効果を消す
        const std::size_t pad = closed ? std::min<std::size_t>(10, n - 2) : 0;
        std::vector<double> tk, xk, yk;
        auto push = [&](double tv, const Waypoint& w) {
            if (!tk.empty() && tv - tk.back() < 1e-9) return;
            tk.push_back(tv); xk.push_back(w.x); yk.push_back(w.y);
        };
        for (std::size_t j = pad; j > 0; --j) push(t[n - 1 - j] - total, waypoints_[n - 1 - j]);
        for (std::size_t i = 0; i < n; ++i) push(t[i], waypoints_[i]);
        for (std::size_t j = 1; j <= pad; ++j) push(total + t[j], waypoints_[j]);
        math::CubicSpline<double> sx, sy;
        sx.build(tk, xk);
        sy.build(tk, yk);

        // grade 未指定なら z の中心差分から導出
        std::vector<double> grade(n);
        for (std::size_t i = 0; i < n; ++i) {
            if (!std::isnan(waypoints_[i].grade)) { grade[i] = waypoints_[i].grade; continue; }
            const std::size_t a = i > 0 ? i - 1 : 0;
            const std::size_t b = i + 1 < n ? i + 1 : n - 1;
            const double run = t[b] - t[a];
            grade[i] = run > 1e-6 ? std::atan2(waypoints_[b].z - waypoints_[a].z, run) : 0.0;
        }

        const int m = std::max(1, static_cast<int>(std::ceil(total / kResampleDs)));
        std::vector<TrackPoint> pts;
        pts.reserve(static_cast<std::size_t>(m) + 1);
        double s_acc = 0.0;
        std::size_t seg = 0;
        constexpr double h = 1e-3;  // 2階微分の中心差分（区間内で導関数は2次式なので厳密）
        for (int k = 0; k <= m; ++k) {
            const double tq = total * k / m;
            while (seg + 2 < n && t[seg + 1] <= tq) ++seg;
            const double span = std::max(t[seg + 1] - t[seg], 1e-12);
            const double a = std::clamp((tq - t[seg]) / span, 0.0, 1.0);
            auto lerp = [a](double u, double v) { return u + a * (v - u); };
            const auto& w0 = waypoints_[seg];
            const auto& w1 = waypoints_[seg + 1];

            const double xd = sx.derivative(tq), yd = sy.derivative(tq);
            const double xdd = (sx.derivative(tq + h) - sx.derivative(tq - h)) / (2.0 * h);
            const double ydd = (sy.derivative(tq + h) - sy.derivative(tq - h)) / (2.0 * h);
            const double sp2 = std::max(xd * xd + yd * yd, 1e-12);

            TrackPoint pt;
            pt.x = sx(tq);
            pt.y = sy(tq);
            if (k > 0) s_acc += std::hypot(pt.x - pts.back().x, pt.y - pts.back().y);
            pt.s = s_acc;
            pt.theta = std::atan2(yd, xd);
            pt.kappa = (xd * ydd - yd * xdd) / (sp2 * std::sqrt(sp2));
            const double w = lerp(w0.width > 0 ? w0.width : default_width_,
                                  w1.width > 0 ? w1.width : default_width_);
            pt.width_left = pt.width_right = 0.5 * w;
            pt.z = lerp(w0.z, w1.z);
            pt.bank = lerp(w0.bank, w1.bank);
            pt.grade = lerp(grade[seg], grade[seg + 1]);
            pt.mu_scale = lerp(w0.mu_scale, w1.mu_scale);
            pts.push_back(pt);
        }
        // θ を連続（アンラップ）に
        for (std::size_t i = 1; i < pts.size(); ++i) {
            pts[i].theta = pts[i - 1].theta + normalize_angle(pts[i].theta - pts[i - 1].theta);
        }
        if (closed) {
            pts.back().x = pts.front().x;
            pts.back().y = pts.front().y;
        }

        CurvatureBasedTrack track(std::move(pts), closed);
        track.set_closed(closed);
        return track;
    }

    /// CSVファイルからウェイポイントを読み込み
    ///
    /// ヘッダー行があれば列名で明示マッピング: x, y, width, z, bank, grade, mu_scale
    /// （z [m] 上向き正, bank/grade [rad]）。未知の列は無視。ヘッダー無しは従来通り x,y[,width]。
    /// 周回フラグ: `closed` 引数（CLI）優先、無ければ先頭の `# closed=true|false` コメント行、
    /// どちらも無ければ端点距離 < 5 m のヒューリスティック（警告を出す）。
    /// 閉ループは先頭点を末尾へ複製して返す。
    static std::vector<Waypoint> load_from_csv(const std::string& filename,
                                               double default_width = 3.0,
                                               std::optional<bool> closed = std::nullopt) {
        std::vector<Waypoint> waypoints;
        std::ifstream file(filename);

        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }

        // 既定（ヘッダー無し）: x,y,width
        int cx = 0, cy = 1, cw = 2, cz = -1, cb = -1, cg = -1, cm = -1;
        std::string line;
        std::streampos data_start = file.tellg();
        while (std::getline(file, line)) {
            std::string low;
            for (unsigned char c : line) {
                if (!std::isspace(c)) low.push_back(static_cast<char>(std::tolower(c)));
            }
            if (low.rfind("#", 0) == 0) {
                const auto pos = low.find("closed=");
                if (pos != std::string::npos && !closed) {
                    const std::string v = low.substr(pos + 7);
                    closed = (v == "true" || v == "1" || v == "yes");
                }
                data_start = file.tellg();
                continue;
            }
            if (low.find('x') != std::string::npos) {
                cx = cy = cw = -1;
                std::istringstream hs(low);
                std::string name;
                for (int col = 0; std::getline(hs, name, ','); ++col) {
                    if (name == "x") cx = col;
                    else if (name == "y") cy = col;
                    else if (name == "width" || name == "w") cw = col;
                    else if (name == "z") cz = col;
                    else if (name == "bank") cb = col;
                    else if (name == "grade") cg = col;
                    else if (name == "mu_scale" || name == "mu") cm = col;
                }
                if (cx < 0 || cy < 0) throw std::runtime_error("CSV header lacks x,y: " + filename);
            } else {
                file.clear();
                file.seekg(data_start);  // ヘッダー無し: このデータ行から読み直す
            }
            break;
        }

        while (std::getline(file, line)) {
            if (!line.empty() && line[0] == '#') continue;
            std::istringstream iss(line);
            Waypoint wp{};
            wp.width = default_width;
            std::string token;
            int parsed = 0;
            for (int col = 0; std::getline(iss, token, ','); ++col) {
                double value;
                try { value = std::stod(token); } catch (...) { continue; }
                if (col == cx) { wp.x = value; ++parsed; }
                else if (col == cy) { wp.y = value; ++parsed; }
                else if (col == cw) wp.width = value;
                else if (col == cz) wp.z = value;
                else if (col == cb) wp.bank = value;
                else if (col == cg) wp.grade = value;
                else if (col == cm) wp.mu_scale = value;
            }
            if (parsed >= 2) {
                waypoints.push_back(wp);
            }
        }

        // Filter out closely-spaced waypoints to prevent noisy theta computation
        // (Some track CSVs have waypoints separated by only millimeters.)
        constexpr double MIN_WAYPOINT_SPACING = 0.5; // [m]
        if (waypoints.size() > 2) {
            std::vector<Waypoint> filtered;
            filtered.push_back(waypoints.front());
            for (size_t i = 1; i < waypoints.size(); ++i) {
                double dx = waypoints[i].x - filtered.back().x;
                double dy = waypoints[i].y - filtered.back().y;
                if (std::sqrt(dx * dx + dy * dy) >= MIN_WAYPOINT_SPACING) {
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
            const Waypoint first = waypoints.front();
            const auto& last = waypoints.back();
            const double dist = std::hypot(first.x - last.x, first.y - last.y);
            if (!closed) {
                closed = dist < 5.0;
                std::cerr << "Warning: track '" << filename << "' has no explicit closed flag "
                          << "(CSV '# closed=true|false' or --track-closed/--track-open); "
                          << "endpoint-gap heuristic " << dist << " m → "
                          << (*closed ? "closed" : "open") << std::endl;
            }
            if (*closed && dist > 0.001) {
                waypoints.push_back(first);
            } else if (!*closed && dist < 0.001 && waypoints.size() > 2) {
                waypoints.pop_back();  // 明示 open なのに端点一致 → 閉ループ扱いを避ける
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
