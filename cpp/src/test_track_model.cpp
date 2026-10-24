/// @file test_track_model.cpp
/// @brief トラックモデルのテストプログラム

#include <fsae/track/track_model.hpp>
#include <fsae/track/frenet.hpp>
#include <fsae/track/waypoint_loader.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

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

int main() {
    std::cout << "Track Model Tests\n" << std::endl;

    test_circle_track();
    test_figure_eight_track();
    test_straight_corner_track();
    test_frenet_conversion();
    test_racing_line();
    test_waypoint_loader();

    std::cout << "\nAll track model tests completed!" << std::endl;
    return 0;
}
