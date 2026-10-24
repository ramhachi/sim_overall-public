// test_engine_torque.cpp
/// エンジントルクカーブのテストプログラム

#include <iostream>
#include <iomanip>
#include "fsae/physics/powertrain/engine.hpp"

using namespace fsae::physics::powertrain;

int main() {
    std::cout << "=== Engine Torque Curve Test ===" << std::endl;
    
    // CBR600RRエンジンの作成
    auto engine = create_cbr600rr_engine();
    
    std::cout << "\nEngine Specifications:" << std::endl;
    std::cout << "  Idle RPM: " << engine.get_idle_rpm() << " rpm" << std::endl;
    std::cout << "  Peak Torque: " << engine.get_peak_torque() << " Nm @ " 
              << engine.get_peak_torque_rpm() << " rpm" << std::endl;
    std::cout << "  Redline RPM: " << engine.get_redline_rpm() << " rpm" << std::endl;
    
    // トルクカーブの表示
    std::cout << "\n=== Torque Curve ===" << std::endl;
    std::cout << "RPM\tTorque (Nm)\tPower (kW)" << std::endl;
    std::cout << std::string(50, '-') << std::endl;
    
    for (int rpm = 1000; rpm <= 15000; rpm += 1000) {
        double torque = engine.get_torque(rpm, 1.0);  // Full throttle
        double power_kw = torque * rpm * 2.0 * M_PI / 60000.0;  // kW
        
        std::cout << std::setw(5) << rpm << "\t"
                  << std::fixed << std::setprecision(1) << std::setw(6) << torque << "\t"
                  << std::setw(6) << power_kw << std::endl;
    }
    
    // スロットル開度のテスト
    std::cout << "\n=== Throttle Test @ 10000 rpm ===" << std::endl;
    std::cout << "Throttle\tTorque (Nm)" << std::endl;
    std::cout << std::string(30, '-') << std::endl;
    
    for (double throttle = 0.0; throttle <= 1.0; throttle += 0.2) {
        double torque = engine.get_torque(10000, throttle);
        std::cout << std::fixed << std::setprecision(1) 
                  << std::setw(5) << (throttle * 100) << "%\t"
                  << std::setw(6) << torque << std::endl;
    }
    
    // ギア比のテスト
    std::cout << "\n=== Gear Ratios Test ===" << std::endl;
    auto gears = GearRatios::cbr600rr_default();
    
    std::cout << "Primary ratio: " << gears.primary_ratio << std::endl;
    std::cout << "Final ratio: " << gears.final_ratio << std::endl;
    std::cout << "\nGear\tRatio\tTotal Ratio" << std::endl;
    std::cout << std::string(40, '-') << std::endl;
    
    for (int gear = 1; gear <= 6; ++gear) {
        double total = gears.get_total_ratio(gear);
        std::cout << gear << "\t" 
                  << std::fixed << std::setprecision(2) << gears.ratios[gear-1] << "\t"
                  << std::setprecision(2) << total << std::endl;
    }
    
    // 車速計算のテスト
    std::cout << "\n=== Speed Calculation (3rd gear @ 10000 rpm) ===" << std::endl;
    double tire_radius = 0.228;  // m
    double wheel_omega = gears.engine_rpm_to_wheel_omega(10000, 3);
    double speed_ms = wheel_omega * tire_radius;
    double speed_kmh = speed_ms * 3.6;
    
    std::cout << "Wheel angular velocity: " << std::fixed << std::setprecision(1) 
              << wheel_omega << " rad/s" << std::endl;
    std::cout << "Vehicle speed: " << speed_ms << " m/s (" 
              << speed_kmh << " km/h)" << std::endl;
    
    // 各ギアでの最高速度
    std::cout << "\n=== Max Speed per Gear (@ " << engine.get_redline_rpm() << " rpm) ===" << std::endl;
    std::cout << "Gear\tSpeed (km/h)" << std::endl;
    std::cout << std::string(25, '-') << std::endl;
    
    for (int gear = 1; gear <= 6; ++gear) {
        double omega = gears.engine_rpm_to_wheel_omega(engine.get_redline_rpm(), gear);
        double speed = omega * tire_radius * 3.6;
        std::cout << gear << "\t" << std::fixed << std::setprecision(1) << speed << std::endl;
    }
    
    std::cout << "\n✓ All tests completed successfully!" << std::endl;
    
    return 0;
}
