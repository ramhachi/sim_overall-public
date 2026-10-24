#include <fsae/physics/tire/nn_lookup_tire.hpp>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

using fsae::physics::tire::NNTextTireModel;
using fsae::physics::tire::TireInput;

int main(int argc, char** argv) {
    int repetitions = 100;
    bool diagnostics_enabled = false;
    if (argc > 3) {
        std::cerr << "Usage: prof_nn_tire [repetitions] [--diagnostics]" << std::endl;
        return 2;
    }
    if (argc >= 2) {
        try {
            std::size_t parsed = 0;
            repetitions = std::stoi(argv[1], &parsed);
            if (parsed != std::string(argv[1]).size()) {
                throw std::invalid_argument("trailing characters");
            }
        } catch (const std::exception&) {
            std::cerr << "repetitions must be a positive integer" << std::endl;
            return 2;
        }
        if (repetitions <= 0) {
            std::cerr << "repetitions must be a positive integer" << std::endl;
            return 2;
        }
    }
    if (argc == 3) {
        if (std::string(argv[2]) != "--diagnostics") {
            std::cerr << "Usage: prof_nn_tire [repetitions] [--diagnostics]" << std::endl;
            return 2;
        }
        diagnostics_enabled = true;
    }

    namespace fs = std::filesystem;
    const fs::path export_dir =
        "output/tire_models/Hoosier_18p0X6p0-10_R20_Rim6";
    const fs::path manifest_path = export_dir / "manifest.json";
    const fs::path vectors_path = export_dir / "test_vectors.json";

    NNTextTireModel model;
    model.load_from_manifest(manifest_path.string());

    nlohmann::json vectors_json;
    {
        std::ifstream file(vectors_path);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open tire test vectors: " + vectors_path.string());
        }
        file >> vectors_json;
    }

    std::vector<TireInput> inputs;
    inputs.reserve(vectors_json["inputs"].size());
    for (const auto& input : vectors_json["inputs"]) {
        inputs.push_back(TireInput {
            input[0].get<double>(),
            input[1].get<double>(),
            input[2].get<double>(),
            input[3].get<double>(),
            input[4].get<double>(),
        });
    }
    if (inputs.empty()) {
        throw std::runtime_error("Tire benchmark input set is empty");
    }

    // Prime reusable buffers and model pages before measuring steady-state runtime.
    for (const auto& input : inputs) {
        (void)model.predict(input, false);
    }
    model.set_runtime_diagnostics_enabled(diagnostics_enabled);
    model.reset_runtime_diagnostics();

    double checksum = 0.0;
    const auto start = std::chrono::steady_clock::now();
    for (int repetition = 0; repetition < repetitions; ++repetition) {
        for (std::size_t i = 0; i < inputs.size(); ++i) {
            const auto forces = model.predict(inputs[i], false);
            const double weight = 1.0 + static_cast<double>(i % 17) * 0.03125;
            checksum += weight *
                (forces.Fx + 0.5 * forces.Fy + 0.125 * forces.Mz);
        }
    }
    const auto end = std::chrono::steady_clock::now();

    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(end - start).count();
    const std::size_t prediction_count =
        inputs.size() * static_cast<std::size_t>(repetitions);

    std::cout << std::setprecision(17);
    std::cout << "Predictions: " << prediction_count << '\n';
    std::cout << "Elapsed: " << elapsed_ms << " ms\n";
    std::cout << "Time per prediction: "
              << 1000.0 * elapsed_ms / static_cast<double>(prediction_count)
              << " us\n";
    std::cout << "Checksum: " << checksum << '\n';
    if (diagnostics_enabled) {
        const auto diagnostics = model.runtime_diagnostics();
        const double calls_per_prediction =
            diagnostics.prediction_calls == 0
                ? 0.0
                : static_cast<double>(diagnostics.forward_physical_calls) /
                    static_cast<double>(diagnostics.prediction_calls);
        std::cout << "Diagnostics:\n";
        std::cout << "  Prediction calls: " << diagnostics.prediction_calls << '\n';
        std::cout << "  forward_physical calls: " << diagnostics.forward_physical_calls << '\n';
        std::cout << "  forward_physical calls/prediction: " << calls_per_prediction << '\n';
        std::cout << "  Postprocess calls: " << diagnostics.postprocess_calls << '\n';
        std::cout << "  Zero-reference forwards: " << diagnostics.zero_reference_forwards << '\n';
        std::cout << "  Symmetry forwards: " << diagnostics.symmetry_forwards << '\n';
        std::cout << "  SA-zero forwards: " << diagnostics.sa_zero_forwards << '\n';
        std::cout << "  Zero-slip projections: " << diagnostics.zero_slip_projections << '\n';
        std::cout << "  Symmetry projections: " << diagnostics.symmetry_projections << '\n';
        std::cout << "  SA-zero projections: " << diagnostics.sa_zero_projections << '\n';
        std::cout << "  Friction-cap projections: " << diagnostics.friction_cap_projections << '\n';
    }
    return 0;
}
