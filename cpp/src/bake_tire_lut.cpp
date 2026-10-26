/// Bake the raw PINN tire output into a 5D LUT for tire.pinn_lut_file.
/// Usage: bake_tire_lut <manifest.json> <out.lut> [nSR nSA nFz nIA nTSTC]

#include <fsae/physics/tire/nn_lookup_tire.hpp>

#include <chrono>
#include <iostream>
#include <random>
#include <string>
#include <thread>

int main(int argc, char** argv) {
    if (argc != 3 && argc != 8) {
        std::cerr << "Usage: bake_tire_lut <manifest.json> <out.lut> [nSR nSA nFz nIA nTSTC]\n";
        return 2;
    }
    // Fz dominates the error (Fy/Fz moves fast at low load); SR/SA beyond ~50 buy little.
    std::array<int, 5> counts = {48, 51, 35, 10, 13};
    if (argc == 8) {
        for (int d = 0; d < 5; ++d) {
            counts[d] = std::stoi(argv[3 + d]);
        }
    }

    fsae::physics::tire::NNTextTireModel model;
    model.load_from_manifest(argv[1]);
    const auto started = std::chrono::steady_clock::now();
    const auto lut = model.build_raw_lut(counts, std::thread::hardware_concurrency());
    fsae::physics::tire::NNTextTireModel::save_raw_lut(*lut, argv[2]);
    const fsae::physics::tire::NNTextTireModel network = model;
    model.load_raw_lut(argv[2]);
    const double sec =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    std::cout << "baked " << lut->data.size() / 3 << " nodes ("
              << lut->data.size() * sizeof(float) / 1e6 << " MB) in " << sec << " s -> "
              << argv[2] << "\n";

    // Full predict() (postprocess included) at random off-grid points, error in units of Fz.
    std::mt19937 rng(1);
    double max_err[4] = {0, 0, 0, 0}, sq_err[4] = {0, 0, 0, 0};
    constexpr int samples = 200000;
    for (int k = 0; k < samples; ++k) {
        std::array<double, 5> x{};
        for (int d = 0; d < 5; ++d) {
            const double hi = lut->lo[d] + lut->step[d] * (lut->n[d] - 1);
            x[d] = std::uniform_real_distribution<double>(lut->lo[d], hi)(rng);
        }
        const fsae::physics::tire::TireInput in{x[0], x[1], x[2], x[3], x[4]};
        const auto a = network.predict(in, true);
        const auto b = model.predict(in, true);
        const double e[4] = {(b.Fx - a.Fx) / x[2], (b.Fy - a.Fy) / x[2], (b.Mx - a.Mx) / x[2],
                             (b.Mz - a.Mz) / x[2]};
        for (int c = 0; c < 4; ++c) {
            max_err[c] = std::max(max_err[c], std::abs(e[c]));
            sq_err[c] += e[c] * e[c];
        }
    }
    const char* names[4] = {"Fx/Fz", "Fy/Fz", "Mx/Fz[m]", "Mz/Fz[m]"};
    for (int c = 0; c < 4; ++c) {
        std::cout << names[c] << " error: rms " << std::sqrt(sq_err[c] / samples) << ", max "
                  << max_err[c] << "\n";
    }
    return 0;
}
