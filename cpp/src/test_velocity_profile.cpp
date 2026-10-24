/// @file test_velocity_profile.cpp
/// @brief 速度プロファイル生成のテストプログラム

#include <fsae/laptime/velocity_profile.hpp>
#include <fsae/track/track_model.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace fsae;

void test_circle_track_velocity_profile() {
    std::cout << "=== Circle Track Velocity Profile Test ===" << std::endl;

    // 半径30mの円形トラック
    auto track = track::create_circle_track(30.0, 3.0);

    // 車両性能パラメータ
    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;  // 1.5G
    perf.max_lon_accel = 1.0 * 9.81;  // 1.0G
    perf.max_lon_decel = 1.8 * 9.81;  // 1.8G
    perf.max_speed = 30.0;             // 30 m/s
    perf.min_speed = 5.0;              // 5 m/s

    // 速度プロファイル生成
    laptime::VelocityProfileGenerator generator(perf);
    auto velocity_profile = generator.generate(track, 1.0);

    // ラップタイム計算
    double lap_time = laptime::compute_lap_time(velocity_profile);

    std::cout << "Track length: " << track.length() << " m" << std::endl;
    std::cout << "Lap time: " << lap_time << " s" << std::endl;

    // 速度プロファイルをCSVに出力
    std::ofstream ofs("circle_velocity_profile.csv");
    ofs << "s,v,ax,ay,v_kmh\n";

    for (const auto& vp : velocity_profile) {
        ofs << std::fixed << std::setprecision(6)
            << vp.s << "," << vp.v << "," << vp.ax << "," << vp.ay << ","
            << (vp.v * 3.6) << "\n";
    }

    std::cout << "Velocity profile saved to circle_velocity_profile.csv" << std::endl;
}

void test_figure_eight_velocity_profile() {
    std::cout << "\n=== Figure Eight Velocity Profile Test ===" << std::endl;

    auto track = track::create_figure_eight_track(25.0, 3.0);

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 30.0;
    perf.min_speed = 5.0;

    laptime::VelocityProfileGenerator generator(perf);
    auto velocity_profile = generator.generate(track, 1.0);

    double lap_time = laptime::compute_lap_time(velocity_profile);

    std::cout << "Track length: " << track.length() << " m" << std::endl;
    std::cout << "Lap time: " << lap_time << " s" << std::endl;

    std::ofstream ofs("figure_eight_velocity_profile.csv");
    ofs << "s,v,ax,ay,v_kmh\n";

    for (const auto& vp : velocity_profile) {
        ofs << std::fixed << std::setprecision(6)
            << vp.s << "," << vp.v << "," << vp.ax << "," << vp.ay << ","
            << (vp.v * 3.6) << "\n";
    }

    std::cout << "Velocity profile saved to figure_eight_velocity_profile.csv" << std::endl;
}

void test_straight_corner_velocity_profile() {
    std::cout << "\n=== Straight + Corner Velocity Profile Test ===" << std::endl;

    auto track = track::create_straight_corner_track(50.0, 15.0, 3.0);

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;
    perf.max_speed = 30.0;
    perf.min_speed = 5.0;

    laptime::VelocityProfileGenerator generator(perf);
    auto velocity_profile = generator.generate(track, 0.5);

    double lap_time = laptime::compute_lap_time(velocity_profile);

    std::cout << "Track length: " << track.length() << " m" << std::endl;
    std::cout << "Lap time: " << lap_time << " s" << std::endl;

    // 加速・減速セクションを特定
    int accel_count = 0, brake_count = 0, coast_count = 0;
    for (const auto& vp : velocity_profile) {
        if (vp.ax > 0.5) accel_count++;
        else if (vp.ax < -0.5) brake_count++;
        else coast_count++;
    }

    std::cout << "Acceleration sections: " << accel_count << " points" << std::endl;
    std::cout << "Braking sections: " << brake_count << " points" << std::endl;
    std::cout << "Coasting sections: " << coast_count << " points" << std::endl;

    std::ofstream ofs("straight_corner_velocity_profile.csv");
    ofs << "s,v,ax,ay,v_kmh\n";

    for (const auto& vp : velocity_profile) {
        ofs << std::fixed << std::setprecision(6)
            << vp.s << "," << vp.v << "," << vp.ax << "," << vp.ay << ","
            << (vp.v * 3.6) << "\n";
    }

    std::cout << "Velocity profile saved to straight_corner_velocity_profile.csv" << std::endl;
}

void test_gg_diagram() {
    std::cout << "\n=== g-g Diagram Generation Test ===" << std::endl;

    laptime::VehiclePerformance perf;
    perf.max_lat_accel = 1.5 * 9.81;
    perf.max_lon_accel = 1.0 * 9.81;
    perf.max_lon_decel = 1.8 * 9.81;

    auto gg_points = laptime::generate_gg_diagram(perf, 100);

    std::cout << "Generated " << gg_points.size() << " g-g diagram points" << std::endl;

    std::ofstream ofs("gg_diagram.csv");
    ofs << "ax,ay,ax_g,ay_g\n";

    for (const auto& [ax, ay] : gg_points) {
        ofs << std::fixed << std::setprecision(6)
            << ax << "," << ay << ","
            << (ax / 9.81) << "," << (ay / 9.81) << "\n";
    }

    std::cout << "g-g diagram saved to gg_diagram.csv" << std::endl;
}

void test_performance_comparison() {
    std::cout << "\n=== Performance Comparison Test ===" << std::endl;

    auto track = track::create_circle_track(30.0, 3.0);

    // 標準性能
    laptime::VehiclePerformance perf_std;
    perf_std.max_lat_accel = 1.5 * 9.81;
    perf_std.max_lon_accel = 1.0 * 9.81;
    perf_std.max_lon_decel = 1.8 * 9.81;
    perf_std.max_speed = 30.0;

    // 高性能（エアロ強化版）
    laptime::VehiclePerformance perf_aero;
    perf_aero.max_lat_accel = 2.0 * 9.81;  // ダウンフォース追加
    perf_aero.max_lon_accel = 1.2 * 9.81;
    perf_aero.max_lon_decel = 2.2 * 9.81;
    perf_aero.max_speed = 32.0;

    laptime::VelocityProfileGenerator gen_std(perf_std);
    laptime::VelocityProfileGenerator gen_aero(perf_aero);

    auto profile_std = gen_std.generate(track, 1.0);
    auto profile_aero = gen_aero.generate(track, 1.0);

    double time_std = laptime::compute_lap_time(profile_std);
    double time_aero = laptime::compute_lap_time(profile_aero);

    std::cout << "Standard car lap time: " << time_std << " s" << std::endl;
    std::cout << "Aero car lap time: " << time_aero << " s" << std::endl;
    std::cout << "Time improvement: " << (time_std - time_aero) << " s ("
              << (100.0 * (time_std - time_aero) / time_std) << "%)" << std::endl;
}

int main() {
    std::cout << "Velocity Profile Generation Tests\n" << std::endl;

    test_circle_track_velocity_profile();
    test_figure_eight_velocity_profile();
    test_straight_corner_velocity_profile();
    test_gg_diagram();
    test_performance_comparison();

    std::cout << "\nAll velocity profile tests completed!" << std::endl;
    return 0;
}
