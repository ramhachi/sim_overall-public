/// @file main_rl_actor_check.cpp
/// @brief Compare MATLAB exported actor outputs against C++ RLDriver inference.

#include <fsae/driver/rl_driver.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

void print_usage() {
    std::cerr << "Usage: rl_actor_check --actor-dir <dir> "
              << "[--test-data <csv>] [--threshold <value>]" << std::endl;
}

bool split_csv_to_doubles(const std::string& line, std::vector<double>& out) {
    out.clear();
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        try {
            out.push_back(std::stod(cell));
        } catch (...) {
            return false;
        }
    }
    return !out.empty();
}

} // namespace

int main(int argc, char** argv) {
    std::string actor_dir;
    std::string test_data;
    double threshold = 1e-3;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--actor-dir" && i + 1 < argc) {
            actor_dir = argv[++i];
        } else if (arg == "--test-data" && i + 1 < argc) {
            test_data = argv[++i];
        } else if (arg == "--threshold" && i + 1 < argc) {
            threshold = std::stod(argv[++i]);
        } else {
            print_usage();
            return 1;
        }
    }

    if (actor_dir.empty()) {
        print_usage();
        return 1;
    }
    if (test_data.empty()) {
        test_data = actor_dir + "/rl_actor_test_data.csv";
    }

    fsae::driver::RLDriver driver;
    driver.load(actor_dir);
    const int act_dim = driver.act_dim();
    const int obs_dim = driver.obs_dim();

    std::ifstream ifs(test_data);
    if (!ifs.is_open()) {
        std::cerr << "Cannot open test data: " << test_data << std::endl;
        return 1;
    }

    std::string line;
    if (!std::getline(ifs, line)) {
        std::cerr << "Test data is empty: " << test_data << std::endl;
        return 1;
    }

    const int expected_cols = obs_dim + act_dim;
    std::size_t n = 0;
    std::vector<double> max_abs_err(act_dim, 0.0);
    std::vector<double> cols;

    while (std::getline(ifs, line)) {
        if (line.empty()) continue;
        if (!split_csv_to_doubles(line, cols) || static_cast<int>(cols.size()) < expected_cols) {
            std::cerr << "Invalid CSV row at line " << (n + 2)
                      << " (got " << cols.size() << " cols, expected " << expected_cols << ")" << std::endl;
            return 1;
        }

        std::vector<double> obs(obs_dim);
        for (int i = 0; i < obs_dim; ++i) {
            obs[i] = cols[i];
        }
        const auto pred = driver.infer_normalized_action(obs);

        for (int i = 0; i < act_dim; ++i) {
            const double expected = cols[obs_dim + i];
            const double err = std::abs(pred[i] - expected);
            if (err > max_abs_err[i]) max_abs_err[i] = err;
        }
        ++n;
    }

    if (n == 0) {
        std::cerr << "No test rows found in: " << test_data << std::endl;
        return 1;
    }

    double overall_max = 0.0;
    std::cout << "rl_actor_check rows=" << n << " act_dim=" << act_dim << " max_abs_err=[";
    for (int i = 0; i < act_dim; ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << max_abs_err[i];
        overall_max = std::max(overall_max, max_abs_err[i]);
    }
    std::cout << "] overall=" << overall_max << " threshold=" << threshold << std::endl;

    if (overall_max > threshold) {
        std::cerr << "Consistency check FAILED." << std::endl;
        return 2;
    }
    std::cout << "Consistency check PASSED." << std::endl;
    return 0;
}
