/// @file test_track_model.cpp
/// @brief トラックモデルのテストプログラム

#include <fsae/track/track_model.hpp>
#include <fsae/track/frenet.hpp>
#include <fsae/track/waypoint_loader.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

using namespace fsae::track;

void test_circle_track() {
    std::cout << "=== Circle Track Test ===" << std::endl;

    // 半径30mの円形トラック
    auto track = create_circle_track(30.0, 3.0);

    std::cout << "Track length: " << track.length() << " m" << std::endl;

    // トラックポイントをCSVに出力
    std::ofstream ofs("circle_track.csv");
    ofs << "s,x,y,theta,kappa\n";

    for (const auto& pt : track.points()) {
        ofs << std::fixed << std::setprecision(6)
            << pt.s << "," << pt.x << "," << pt.y << ","
            << pt.theta << "," << pt.kappa << "\n";
    }

    std::cout << "Track points saved to circle_track.csv" << std::endl;
}

void test_figure_eight_track() {
    std::cout << "\n=== Figure Eight Track Test ===" << std::endl;

    auto track = create_figure_eight_track(25.0, 3.0);

    std::cout << "Track length: " << track.length() << " m" << std::endl;

    std::ofstream ofs("figure_eight_track.csv");
    ofs << "s,x,y,theta,kappa\n";

    for (const auto& pt : track.points()) {
        ofs << std::fixed << std::setprecision(6)
            << pt.s << "," << pt.x << "," << pt.y << ","
            << pt.theta << "," << pt.kappa << "\n";
    }

    std::cout << "Track points saved to figure_eight_track.csv" << std::endl;
}

void test_straight_corner_track() {
    std::cout << "\n=== Straight + Corner Track Test ===" << std::endl;

    auto track = create_straight_corner_track(50.0, 15.0, 3.0);

    std::cout << "Track length: " << track.length() << " m" << std::endl;

    std::ofstream ofs("straight_corner_track.csv");
    ofs << "s,x,y,theta,kappa\n";

    for (const auto& pt : track.points()) {
        ofs << std::fixed << std::setprecision(6)
            << pt.s << "," << pt.x << "," << pt.y << ","
            << pt.theta << "," << pt.kappa << "\n";
    }

    std::cout << "Track points saved to straight_corner_track.csv" << std::endl;
}

void test_frenet_conversion() {
    std::cout << "\n=== Frenet Coordinate Conversion Test ===" << std::endl;

    auto track = create_circle_track(30.0, 3.0);
    FrenetConverter converter(track);

    // テストポイント（円の外側）
    double x = 35.0;
    double y = 0.0;
    double psi = M_PI / 2.0;

    auto frenet = converter.global_to_frenet(x, y, psi);

    std::cout << "Global (x, y, psi): (" << x << ", " << y << ", " << psi << ")" << std::endl;
    std::cout << "Frenet (s, n, d_heading): (" << frenet.s << ", " << frenet.n << ", "
              << frenet.d_heading << ")" << std::endl;

    // 逆変換
    auto [x_back, y_back, theta_back] = converter.frenet_to_global(frenet);

    std::cout << "Converted back (x, y, theta): (" << x_back << ", " << y_back << ", "
              << theta_back << ")" << std::endl;

    // トラック内判定
    bool within = converter.is_within_track(frenet);
    std::cout << "Within track: " << (within ? "Yes" : "No") << std::endl;
}

void test_racing_line() {
    std::cout << "\n=== Racing Line Computation Test ===" << std::endl;

    // 8の字トラックでレーシングラインを計算
    auto track = create_figure_eight_track(25.0, 4.0);
    auto racing_line = compute_racing_line(track, 0.5);

    std::cout << "Racing line points: " << racing_line.size() << std::endl;

    // レーシングラインをグローバル座標に変換して出力
    FrenetConverter converter(track);
    std::ofstream ofs("racing_line.csv");
    ofs << "s,x,y,n,theta\n";

    for (const auto& frenet_pt : racing_line) {
        auto [x, y, theta] = converter.frenet_to_global(frenet_pt);

        ofs << std::fixed << std::setprecision(6)
            << frenet_pt.s << "," << x << "," << y << ","
            << frenet_pt.n << "," << theta << "\n";
    }

    std::cout << "Racing line saved to racing_line.csv" << std::endl;
}

void test_waypoint_loader() {
    std::cout << "\n=== Waypoint Loader Test ===" << std::endl;

    // テスト用ウェイポイントを作成
    std::vector<Waypoint> waypoints = {
        {0.0, 0.0, 3.0},
        {50.0, 0.0, 3.0},
        {75.0, 25.0, 3.0},
        {75.0, 50.0, 3.0},
        {50.0, 75.0, 3.0},
        {0.0, 75.0, 3.0},
        {-25.0, 50.0, 3.0},
        {-25.0, 25.0, 3.0},
        {0.0, 0.0, 3.0}
    };

    WaypointTrackBuilder builder(waypoints, 3.0);
    auto track = builder.build();

    std::cout << "Track length: " << track.length() << " m" << std::endl;

    std::ofstream ofs("waypoint_track.csv");
    ofs << "s,x,y,theta,kappa\n";

    for (const auto& pt : track.points()) {
        ofs << std::fixed << std::setprecision(6)
            << pt.s << "," << pt.x << "," << pt.y << ","
            << pt.theta << "," << pt.kappa << "\n";
    }

    std::cout << "Waypoint track saved to waypoint_track.csv" << std::endl;

    // テスト用CSVファイルを作成
    std::ofstream csv("test_waypoints.csv");
    csv << "x,y,width\n";
    for (const auto& wp : waypoints) {
        csv << wp.x << "," << wp.y << "," << wp.width << "\n";
    }
    csv.close();

    std::cout << "Test waypoints saved to test_waypoints.csv" << std::endl;

    // CSVから読み込みテスト
    try {
        auto loaded_waypoints = WaypointTrackBuilder::load_from_csv("test_waypoints.csv");
        std::cout << "Loaded " << loaded_waypoints.size() << " waypoints from CSV" << std::endl;
    } catch (const std::exception& e) {
        std::cout << "CSV load failed: " << e.what() << std::endl;
    }
}

static int g_failures = 0;
#define TRACK_CHECK(cond, msg) do { if (!(cond)) { std::cerr << "FAIL: " << msg << std::endl; ++g_failures; } } while (0)

/// Issue #25: spline κ on an analytic circle (waypoints every ~1.5 m) must be within 0.5 %.
void test_spline_curvature_circle() {
    for (double R : {7.5, 15.0, 40.0}) {
        std::vector<Waypoint> wps;
        const int n = static_cast<int>(std::ceil(2.0 * M_PI * R / 1.5));
        for (int i = 0; i <= n; ++i) {
            const double a = 2.0 * M_PI * i / n;
            wps.push_back({R * std::cos(a), R * std::sin(a), 3.0});
        }
        auto track = WaypointTrackBuilder(wps, 3.0).build();
        double max_err = 0.0;
        for (const auto& p : track.points()) {
            max_err = std::max(max_err, std::abs(p.kappa * R - 1.0));
        }
        std::cout << "circle R=" << R << " max rel kappa err=" << max_err * 100.0 << " %" << std::endl;
        TRACK_CHECK(max_err < 0.005, "circle kappa error >= 0.5 % at R=" << R);
        TRACK_CHECK(track.is_closed_loop(), "circle should be closed");

        // Frenet: a point on the centreline between samples projects with sub-mm s error
        FrenetConverter conv(track);
        const double s_true = 0.37 * track.length();
        const double a = s_true / R;
        auto f = conv.global_to_frenet(R * std::cos(a), R * std::sin(a), a + M_PI / 2.0);
        TRACK_CHECK(std::abs(f.s - s_true) < 0.01, "Frenet s error " << std::abs(f.s - s_true));
        TRACK_CHECK(std::abs(f.n) < 0.01, "Frenet n error " << f.n);
    }
}

/// Legacy (pre-#25) estimator for reporting: 3-point heading difference + 5-point box filter.
static double legacy_peak_kappa(const std::vector<Waypoint>& w) {
    const std::size_t n = w.size();
    std::vector<double> k(n, 0.0);
    for (std::size_t i = 1; i + 1 < n; ++i) {
        const double tp = std::atan2(w[i].y - w[i - 1].y, w[i].x - w[i - 1].x);
        const double tn = std::atan2(w[i + 1].y - w[i].y, w[i + 1].x - w[i].x);
        const double d = std::remainder(tn - tp, 2.0 * M_PI);
        k[i] = d / (0.5 * (std::hypot(w[i].y - w[i - 1].y, w[i].x - w[i - 1].x) +
                           std::hypot(w[i + 1].y - w[i].y, w[i + 1].x - w[i].x)));
    }
    double peak = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        double sum = 0.0;
        for (int j = -2; j <= 2; ++j) sum += k[(i + n + j) % n];
        peak = std::max(peak, std::abs(sum / 5.0));
    }
    return peak;
}

/// Issue #25/#28: autocross CSV peak κ before/after, explicit columns + closed flag.
void test_autocross_csv(const std::string& root) {
    const std::string path = root + "/data/ASE_autocross_sim.csv";
    std::ifstream probe(path);
    if (!probe) {
        std::cout << "skip autocross CSV test (run from repo root)" << std::endl;
        return;
    }
    auto wps = WaypointTrackBuilder::load_from_csv(path, 3.0, true);
    auto track = WaypointTrackBuilder(wps, 3.0).build();
    double peak = 0.0, zmin = 1e9, zmax = -1e9;
    for (const auto& p : track.points()) {
        peak = std::max(peak, std::abs(p.kappa));
        zmin = std::min(zmin, p.z);
        zmax = std::max(zmax, p.z);
    }
    const double legacy = legacy_peak_kappa(wps);
    std::cout << "autocross peak |kappa|: legacy(3pt+box5)=" << legacy
              << " 1/m, spline=" << peak << " 1/m (" << (peak / legacy - 1.0) * 100.0
              << " %), z range " << zmin << ".." << zmax << " m" << std::endl;
    TRACK_CHECK(peak >= legacy, "spline should not attenuate peak kappa");
    TRACK_CHECK(track.is_closed_loop(), "explicit closed flag ignored");
    TRACK_CHECK(zmax - zmin > 0.1, "z column not mapped");
    // Explicit open flag must win over the endpoint heuristic.
    auto open_wps = WaypointTrackBuilder::load_from_csv(path, 3.0, false);
    TRACK_CHECK(!WaypointTrackBuilder(open_wps, 3.0).build().is_closed_loop(), "explicit open ignored");
}

int main(int argc, char** argv) {
    std::cout << "Track Model Tests\n" << std::endl;

    test_circle_track();
    test_figure_eight_track();
    test_straight_corner_track();
    test_frenet_conversion();
    test_racing_line();
    test_waypoint_loader();
    test_spline_curvature_circle();
    test_autocross_csv(argc > 1 ? argv[1] : ".");

    if (g_failures > 0) {
        std::cerr << g_failures << " track model check(s) failed" << std::endl;
        return 1;
    }
    std::cout << "\nAll track model tests completed!" << std::endl;
    return 0;
}
