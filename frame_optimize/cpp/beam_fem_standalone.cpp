/**
 * @file beam_fem_standalone.cpp
 * @brief Standalone 3D Timoshenko beam FEM solver with JSON I/O (kernel: beam_fem_core.hpp)
 *
 * Usage:
 *   ./beam_fem_standalone input.json output.json
 *   ./beam_fem_standalone --selftest          (closed-form benchmarks, exit 1 on failure)
 *
 * Input JSON:
 * {
 *   "nodes": [[x,y,z], ...],                                   // m
 *   "elements": [[ni, nj, D, t, E, G, rho, shape, jf, kAy, kAz, off_i, off_j], ...],
 *   "bcs": [[node, dof, value], ...],                          // 1-indexed
 *   "loads": [[node, dof, value], ...],
 *   "rigid_links": [[master, slave], ...]                      // optional RBE2
 * }
 * Output JSON: {"success", "U": [[ux,uy,uz,rx,ry,rz], ...], "K_torsion", "mass",
 *               "element_forces": Mx12 local end forces, "element_stress_vm" [MPa],
 *               "stress_max" [MPa], "element_buckling_ratio", "buckling_ratio", "warnings"?}
 */

#include <nlohmann/json.hpp>

#include <fstream>
#include <iostream>

#include "beam_fem_core.hpp"

using json = nlohmann::json;
namespace fo = frame_optimize;

// ---- Closed-form benchmarks (--selftest) ----------------------------------
namespace {

int g_failures = 0;

void check(const std::string& name, double got, double want, double rtol) {
    const double err = std::abs(got - want) / std::max(std::abs(want), 1e-300);
    const bool ok = err <= rtol;
    if (!ok) ++g_failures;
    std::cout << (ok ? "[PASS] " : "[FAIL] ") << name << ": got " << got << ", expected " << want
              << " (rel err " << err << ")\n";
}

// Straight member from origin along unit axis a, length L, n elements, clamped at node 1.
json cantilever(const Eigen::Vector3d& a, double L, int n, const std::vector<double>& section) {
    json in;
    in["nodes"] = json::array();
    in["elements"] = json::array();
    for (int k = 0; k <= n; ++k) {
        const Eigen::Vector3d p = a * (L * k / n);
        in["nodes"].push_back({p.x(), p.y(), p.z()});
    }
    for (int k = 1; k <= n; ++k) {
        std::vector<double> row = {double(k), double(k + 1)};
        row.insert(row.end(), section.begin(), section.end());
        in["elements"].push_back(row);
    }
    in["bcs"] = json::array();
    for (int d = 1; d <= 6; ++d) in["bcs"].push_back({1, d, 0.0});
    in["loads"] = json::array();
    return in;
}

double disp_along(const json& out, int node, const Eigen::Vector3d& dir, int offset = 0) {
    const auto& u = out["U"][node - 1];
    return u[offset + 0].get<double>() * dir.x() + u[offset + 1].get<double>() * dir.y() +
           u[offset + 2].get<double>() * dir.z();
}

int run_selftest() {
    // 25.4 x 1.65 round 4130-like tube; kA = A/2 (Cowper, thin round tube)
    const double D = 25.4, t = 1.65, E = 205.0, G = 80.0;
    fo::BeamElement sec;
    sec.D = D; sec.t = t; sec.shape = 1;
    fo::compute_section_properties(sec);
    const double A = sec.A * 1e-6, I = sec.Iy * 1e-12, J = sec.J * 1e-12, kA = A / 2;
    const std::vector<double> round = {D, t, E, G, 7850, 1, 1.0, sec.A / 2, sec.A / 2};
    const double Ep = E * 1e9, Gp = G * 1e9, P = 1000.0;

    {   // 1. axial along global y: delta = P L / (E A)
        json in = cantilever({0, 1, 0}, 1.0, 1, round);
        in["loads"].push_back({2, 2, P});
        check("axial along y", fo::solve_json(in)["U"][1][1].get<double>(), P * 1.0 / (Ep * A), 1e-12);
    }
    {   // 2. short skew cantilever, tip load normal to axis: PL^3/3EI + PL/kGA
        const Eigen::Vector3d a(1.0 / 3, 2.0 / 3, 2.0 / 3), f(2.0 / 3, 1.0 / 3, -2.0 / 3);
        const double L = 0.15;
        json in = cantilever(a, L, 4, round);
        for (int d = 0; d < 3; ++d) in["loads"].push_back({5, d + 1, P * f(d)});
        const double eb = P * L * L * L / (3 * Ep * I), shear = P * L / (Gp * kA);
        check("Timoshenko cantilever tip (skew axis, L=150 mm)", disp_along(fo::solve_json(in), 5, f),
              eb + shear, 1e-6);
        std::cout << "       shear term share: " << shear / (eb + shear) << "\n";

        // near-vertical axis exercises the other reference-vector branch
        const Eigen::Vector3d av = Eigen::Vector3d(0.1, 0.0, 1.0).normalized(), fv(0, 1, 0);
        json inv = cantilever(av, L, 3, round);
        inv["loads"].push_back({4, 2, P});
        check("Timoshenko cantilever tip (near-vertical axis)", disp_along(fo::solve_json(inv), 4, fv),
              eb + shear, 1e-6);
    }
    {   // 3. round-tube torsion along x: theta = T L / (G J); K_torsion = T / theta[deg]
        const double L = 0.5, Tq = 100.0;
        json in = cantilever({1, 0, 0}, L, 2, round);
        in["loads"].push_back({3, 4, Tq});
        const json out = fo::solve_json(in);
        const double theta = Tq * L / (Gp * J);
        check("round tube twist", out["U"][2][3].get<double>(), theta, 1e-9);
        check("round tube K_torsion [Nm/deg]", out["K_torsion"].get<double>(), Tq / (theta * 180 / fo::kPi), 1e-9);
    }
    {   // 4. square-tube torsion about global y uses Bredt J = b^3 t
        const double L = 0.5, Tq = 100.0, b = D - t;
        json in = cantilever({0, 1, 0}, L, 1, {D, t, E, G, 7850, 2, 1.0, 2 * b * t, 2 * b * t});
        in["loads"].push_back({2, 5, Tq});
        check("square tube twist (Bredt)", fo::solve_json(in)["U"][1][4].get<double>(),
              Tq * L / (Gp * b * b * b * t * 1e-12), 1e-9);
    }

    {   // 5. rigid end offsets a (clamp side) and b (tip side), Lf = L - a - b, tip load P:
        //    flexible end sees P and M = P b; tip = delta_f + b theta_f
        const double L = 0.3, a = 0.02, b = 0.03, Lf = L - a - b;
        std::vector<double> sec_off = round;
        sec_off.push_back(a);
        sec_off.push_back(b);
        json in = cantilever({0, 1, 0}, L, 1, sec_off);
        in["loads"].push_back({2, 3, P});
        const double EI = Ep * I;
        const double df = P * Lf * Lf * Lf / (3 * EI) + P * b * Lf * Lf / (2 * EI) + P * Lf / (Gp * kA);
        const double tf = P * Lf * Lf / (2 * EI) + P * b * Lf / EI;
        check("rigid end offsets cantilever tip", fo::solve_json(in)["U"][1][2].get<double>(), df + b * tf, 1e-6);
    }

    {   // 6. stress recovery: root bending sigma = P L c / I, torsion vm = sqrt(3) T c / J,
        //    Euler ratio = P / (pi^2 E I / L^2) under compression
        const double L = 0.5, c = D / 2;
        json in = cantilever({0, 1, 0}, L, 1, round);
        in["loads"].push_back({2, 3, P});
        const json out = fo::solve_json(in);
        check("root bending stress [MPa]", out["stress_max"].get<double>(), P * L * 1e3 * c / sec.Iy, 1e-9);
        check("root moment in element_forces [N*m]", std::hypot(out["element_forces"][0][4].get<double>(),
              out["element_forces"][0][5].get<double>()), P * L, 1e-9);

        json tor = cantilever({1, 0, 0}, L, 1, round);
        tor["loads"].push_back({2, 4, 100.0});
        check("torsion von Mises [MPa]", fo::solve_json(tor)["stress_max"].get<double>(),
              std::sqrt(3.0) * 100.0e3 * c / sec.J, 1e-9);

        json comp = cantilever({0, 1, 0}, L, 1, round);
        comp["loads"].push_back({2, 2, -P});
        const json cout_ = fo::solve_json(comp);
        check("Euler buckling ratio", cout_["buckling_ratio"].get<double>(), P / (fo::kPi * fo::kPi * Ep * I / (L * L)), 1e-9);
        check("axial stress [MPa]", cout_["stress_max"].get<double>(), P / sec.A, 1e-6);
    }

    {   // 7. rigid links: load on an offset slave twists the tip by P e L / (G J);
        //    clamping through a slave (Lagrange row) gives the plain cantilever answer.
        const double L = 0.5, e = 0.1;
        json in = cantilever({1, 0, 0}, L, 1, round);
        in["nodes"].push_back({L, e, 0.0});
        in["rigid_links"] = {{2, 3}};
        in["loads"].push_back({3, 3, P});
        const json out = fo::solve_json(in);
        const double tw = P * e * L / (Gp * J), uz = P * L * L * L / (3 * Ep * I) + P * L / (Gp * kA);
        check("rigid link: tip twist from offset load", out["U"][1][3].get<double>(), tw, 1e-9);
        check("rigid link: slave uz = uz_m + theta_x * e", out["U"][2][2].get<double>(), uz + tw * e, 1e-9);

        // root clamped only through three pinned points of one rigid body {1, 3, 4}:
        // pins on slaves 3 and 4 are Lagrange rows, 9 constraints for 6 DOFs (redundant)
        json bc = cantilever({1, 0, 0}, L, 1, round);
        bc["nodes"].push_back({-0.05, 0.0, 0.0});
        bc["nodes"].push_back({-0.05, 0.05, 0.0});
        bc["rigid_links"] = {{3, 1}, {4, 3}};
        bc["bcs"] = json::array();
        for (int n : {1, 3, 4})
            for (int d = 1; d <= 3; ++d) bc["bcs"].push_back({n, d, 0.0});
        bc["loads"].push_back({2, 3, P});
        check("pinned rigid body via slave BCs", fo::solve_json(bc)["U"][1][2].get<double>(), uz, 1e-8);

        json mech = cantilever({1, 0, 0}, L, 1, round);
        mech["bcs"] = {{1, 1, 0.0}, {1, 2, 0.0}, {1, 3, 0.0}};  // pinned root: mechanism
        mech["loads"].push_back({2, 3, P});
        bool threw = false;
        try { fo::solve_json(mech); } catch (const std::exception&) { threw = true; }
        check("mechanism is reported", threw ? 1.0 : 0.0, 1.0, 0.0);
    }

    {   // 8. K_torsion from a +-F couple on a rigid jig: K = (F * arm) / theta[deg], theta = T L / (G J)
        const double L = 0.5, h = 0.2, F = 500.0;
        json in = cantilever({1, 0, 0}, L, 2, round);
        in["nodes"].push_back({L, -h, 0.0});
        in["nodes"].push_back({L, h, 0.0});
        in["rigid_links"] = {{3, 4}, {3, 5}};
        in["loads"].push_back({4, 3, F});
        in["loads"].push_back({5, 3, -F});
        const double Tq = 2 * h * F, theta = Tq * L / (Gp * J);
        check("K_torsion from applied couple", fo::solve_json(in)["K_torsion"].get<double>(),
              Tq / (theta * 180 / fo::kPi), 1e-9);
    }

    std::cout << (g_failures ? "SELFTEST FAILED" : "SELFTEST PASSED") << " (" << g_failures << " failures)\n";
    return g_failures;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string(argv[1]) == "--selftest") {
        return run_selftest() == 0 ? 0 : 1;
    }
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " input.json output.json | --selftest\n";
        return 1;
    }
    json output;
    try {
        std::ifstream input_file(argv[1]);
        if (!input_file.is_open()) {
            throw std::runtime_error("Cannot open input file: " + std::string(argv[1]));
        }
        output = fo::solve_json(json::parse(input_file));
    } catch (const std::exception& e) {
        output = {{"success", false}, {"error", e.what()}};
        std::cerr << "Error: " << e.what() << std::endl;
    }
    std::ofstream output_file(argv[2]);
    if (!output_file.is_open()) {
        std::cerr << "Cannot open output file: " << argv[2] << std::endl;
        return 1;
    }
    output_file << output.dump(2);
    return output["success"].get<bool>() ? 0 : 1;
}
