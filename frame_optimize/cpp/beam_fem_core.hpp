#pragma once
// Shared 3D Timoshenko beam FEM kernel for beam_fem_standalone, beam_fem_server
// and the MEX (mex/beam_fem_solver.cpp). C++14: the MEX is built with -std=c++14.
//
// Units: nodes [m], D/t [mm], E/G [GPa], rho [kg/m^3], kA [mm^2], forces [N, N*m].
//
// Element row (node ids 1-indexed):
//   [ni nj D t E G rho shape jf kAy kAz off_i off_j]
//   kAy/kAz = shear areas [mm^2] for bending in the local x-y / x-z planes.
//   off_i/off_j = rigid end lengths [m] along the member axis (joint inside the
//   chord tube); the flexible length is Lf = L - off_i - off_j.
//   Accepted widths: 13, 11 (no offsets) and legacy 9 (Phi = 0 + warning).
//
// Constraints: BCs are eliminated (no penalty); rigid links (node pairs) merge into
// rigid bodies eliminated master-slave (RBE2); a BC on a slave DOF becomes a Lagrange-multiplier row.
// Nodes not connected (through elements/links) to any BC node are clamped.

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "torsion_rigidity.hpp"

namespace frame_optimize {

constexpr double kPi = 3.14159265358979323846;
using Mat12 = Eigen::Matrix<double, 12, 12>;
using Vec12 = Eigen::Matrix<double, 12, 1>;

struct BeamElement {
    int node_i = 0, node_j = 0;  // 0-indexed
    double D = 0.0, t = 0.0, E = 0.0, G = 0.0, rho = 0.0;
    int shape = 1;  // 1 = round, 2 = square
    double jf = 1.0;
    double kAy = 0.0, kAz = 0.0;
    double off_i = 0.0, off_j = 0.0;
    double A = 0.0, Iy = 0.0, Iz = 0.0, J = 0.0;
    double L = 0.0, Lf = 0.0;  // node-to-node and flexible length [m]
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();  // columns = local axes in global
};

struct FemInput {
    Eigen::MatrixXd nodes;  // n x 3
    std::vector<BeamElement> elements;
    std::vector<FemLoad> bcs;    // value = prescribed displacement
    std::vector<FemLoad> loads;  // value = force / moment
    std::vector<std::pair<int, int>> rigid_links;  // node pairs joined rigidly, 0-indexed
};

struct FemResult {
    Eigen::VectorXd U;
    double mass = 0.0;
    double K_torsion = 0.0;
    Eigen::MatrixXd element_forces;  // M x 12 local end forces at the flexible ends [N, N*m]:
                                     // [Fx Fy Fz Mx My Mz]_i [Fx Fy Fz Mx My Mz]_j
    Eigen::VectorXd stress_vm;       // M, max von Mises over both ends [MPa]
    Eigen::VectorXd buckling;        // M, compressive N / P_cr (0 in tension)
    double stress_max = 0.0;
    double buckling_ratio = 0.0;
    std::vector<std::string> warnings;
};

inline void compute_section_properties(BeamElement& e) {
    const double Di = e.D - 2.0 * e.t;
    if (e.t <= 0.0 || Di < 0.0) {
        throw std::runtime_error("Invalid tube: need 0 < t <= D/2");
    }
    if (e.shape == 1) {
        e.A = kPi / 4.0 * (e.D * e.D - Di * Di);
        e.Iy = e.Iz = kPi / 64.0 * (std::pow(e.D, 4) - std::pow(Di, 4));
        e.J = 2.0 * e.Iy;
    } else if (e.shape == 2) {
        e.A = e.D * e.D - Di * Di;
        e.Iy = e.Iz = (std::pow(e.D, 4) - std::pow(Di, 4)) / 12.0;
        // Bredt-Batho: J = 4 A_m^2 t / s, centreline b = D - t, A_m = b^2, s = 4b.
        // ponytail: sharp corners (catalog has no corner radius r); with r_m = r - t/2,
        // A_m -= (4 - pi) r_m^2 and s -= (8 - 2 pi) r_m once the catalog lists r.
        const double b = e.D - e.t;
        e.J = b * b * b * e.t;
    } else {
        throw std::runtime_error("Shape must be 1 (round) or 2 (square)");
    }
}

inline Eigen::Matrix3d compute_rotation_matrix(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2) {
    const Eigen::Vector3d x = (p2 - p1).normalized();
    const Eigen::Vector3d ref = std::abs(x(2)) < 0.9 ? Eigen::Vector3d(0, 0, 1) : Eigen::Vector3d(0, 1, 0);
    const Eigen::Vector3d z = x.cross(ref).normalized();
    const Eigen::Vector3d y = z.cross(x).normalized();
    Eigen::Matrix3d R;
    R.col(0) = x;
    R.col(1) = y;
    R.col(2) = z;
    return R;
}

// u_local = T * u_global. R holds the local axes as columns, so each block is R^T.
inline Mat12 compute_transformation(const Eigen::Matrix3d& R) {
    Mat12 T = Mat12::Zero();
    for (int b = 0; b < 4; ++b) {
        T.block<3, 3>(3 * b, 3 * b) = R.transpose();
    }
    return T;
}

// Phi-form Timoshenko stiffness in local coordinates (Przemieniecki).
// Phi = 12 E I / (G kA L^2); kA = 0 -> Phi = 0 (Euler-Bernoulli).
// jf scales bending and torsion (joint flexibility), not the axial term.
inline Mat12 compute_local_stiffness(const BeamElement& e, double L) {
    const double E = e.E * 1e9, G = e.G * 1e9;
    const double A = e.A * 1e-6, Iy = e.Iy * 1e-12, Iz = e.Iz * 1e-12, J = e.J * 1e-12;
    const double phi_y = e.kAy > 0.0 ? 12.0 * E * Iz / (G * e.kAy * 1e-6 * L * L) : 0.0;  // x-y plane
    const double phi_z = e.kAz > 0.0 ? 12.0 * E * Iy / (G * e.kAz * 1e-6 * L * L) : 0.0;  // x-z plane

    Mat12 K = Mat12::Zero();
    const double ea = E * A / L;
    K(0, 0) = K(6, 6) = ea;
    K(0, 6) = K(6, 0) = -ea;

    const double gj = e.jf * G * J / L;
    K(3, 3) = K(9, 9) = gj;
    K(3, 9) = K(9, 3) = -gj;

    // x-y plane: v (1, 7), theta_z (5, 11)
    const double cy = e.jf * E * Iz / (L * L * L * (1.0 + phi_y));
    K(1, 1) = K(7, 7) = 12 * cy;
    K(1, 7) = K(7, 1) = -12 * cy;
    K(1, 5) = K(5, 1) = K(1, 11) = K(11, 1) = 6 * L * cy;
    K(5, 7) = K(7, 5) = K(7, 11) = K(11, 7) = -6 * L * cy;
    K(5, 5) = K(11, 11) = (4 + phi_y) * L * L * cy;
    K(5, 11) = K(11, 5) = (2 - phi_y) * L * L * cy;

    // x-z plane: w (2, 8), theta_y (4, 10); signs flip because theta_y = -dw/dx
    const double cz = e.jf * E * Iy / (L * L * L * (1.0 + phi_z));
    K(2, 2) = K(8, 8) = 12 * cz;
    K(2, 8) = K(8, 2) = -12 * cz;
    K(2, 4) = K(4, 2) = K(2, 10) = K(10, 2) = -6 * L * cz;
    K(4, 8) = K(8, 4) = K(8, 10) = K(10, 8) = 6 * L * cz;
    K(4, 4) = K(10, 10) = (4 + phi_z) * L * L * cz;
    K(4, 10) = K(10, 4) = (2 - phi_z) * L * L * cz;
    return K;
}

// Rigid end offsets: flexible-end displacement = u_node + theta x (offset along x_local).
// Kept local: u_flex = H * u_local.
inline Mat12 compute_offset_matrix(const BeamElement& e) {
    Mat12 H = Mat12::Identity();
    H(1, 5) = e.off_i;    // v_i'  = v_i + a theta_z
    H(2, 4) = -e.off_i;   // w_i'  = w_i - a theta_y
    H(7, 11) = -e.off_j;  // v_j'  = v_j - b theta_z
    H(8, 10) = e.off_j;   // w_j'  = w_j + b theta_y
    return H;
}

// 12x12 element stiffness in global coordinates at the element's nodes.
inline Mat12 compute_global_stiffness(const BeamElement& e) {
    const Mat12 HT = compute_offset_matrix(e) * compute_transformation(e.R);
    return HT.transpose() * compute_local_stiffness(e, e.Lf) * HT;
}

// row: element row (see header), ncols: 9, 11 or 13.
inline BeamElement parse_element_row(const double* row, int ncols, const Eigen::MatrixXd& nodes) {
    if (ncols != 9 && ncols != 11 && ncols != 13) {
        throw std::runtime_error("Element rows must have 9, 11 or 13 columns");
    }
    BeamElement e;
    e.node_i = static_cast<int>(row[0]) - 1;
    e.node_j = static_cast<int>(row[1]) - 1;
    const int n = static_cast<int>(nodes.rows());
    if (e.node_i < 0 || e.node_i >= n || e.node_j < 0 || e.node_j >= n || e.node_i == e.node_j) {
        throw std::runtime_error("Element references invalid nodes: " + std::to_string(e.node_i + 1) +
                                 ", " + std::to_string(e.node_j + 1));
    }
    e.D = row[2];
    e.t = row[3];
    e.E = row[4];
    e.G = row[5];
    e.rho = row[6];
    e.shape = static_cast<int>(row[7]);
    e.jf = row[8];
    if (ncols >= 11) {
        e.kAy = row[9];
        e.kAz = row[10];
    }
    if (ncols >= 13) {
        e.off_i = row[11];
        e.off_j = row[12];
    }
    compute_section_properties(e);
    const Eigen::Vector3d p1 = nodes.row(e.node_i).transpose();
    const Eigen::Vector3d p2 = nodes.row(e.node_j).transpose();
    e.L = (p2 - p1).norm();
    if (e.L <= 0.0) {
        throw std::runtime_error("Zero-length element");
    }
    e.R = compute_rotation_matrix(p1, p2);
    e.Lf = e.L - e.off_i - e.off_j;
    if (e.off_i < 0.0 || e.off_j < 0.0 || e.Lf <= 0.05 * e.L) {
        throw std::runtime_error("Rigid end offsets must be >= 0 and leave > 5% flexible length");
    }
    return e;
}

// End forces f = K_flex * H * T * u_e, extreme-fibre von Mises at both flexible ends
// (sigma = |N|/A + bending, tau = torsion) and Euler P_cr = pi^2 E I / (K Lf)^2.
// ponytail: transverse shear stress ignored and K = 1 (pinned-pinned, conservative for
// welded nodes); add V/kA and calibrated K if members fail near the limit.
inline void compute_element_stresses(const std::vector<BeamElement>& elements, const Eigen::VectorXd& U,
                                     FemResult& res) {
    const Eigen::Index m = static_cast<Eigen::Index>(elements.size());
    res.element_forces.resize(m, 12);
    res.stress_vm.resize(m);
    res.buckling.resize(m);
    for (Eigen::Index k = 0; k < m; ++k) {
        const BeamElement& e = elements[k];
        Vec12 ue;
        ue << U.segment<6>(6 * e.node_i), U.segment<6>(6 * e.node_j);
        const Vec12 f = compute_local_stiffness(e, e.Lf) * compute_offset_matrix(e) * compute_transformation(e.R) * ue;
        res.element_forces.row(k) = f.transpose();

        const double c = e.D / 2.0;  // mm
        const double am = (e.D - e.t) * (e.D - e.t);
        double vm = 0.0;
        for (int end = 0; end < 2; ++end) {
            const double N = f(6 * end), Tq = f(6 * end + 3) * 1e3, My = f(6 * end + 4) * 1e3, Mz = f(6 * end + 5) * 1e3;
            const double sb = e.shape == 1 ? std::hypot(My, Mz) * c / e.Iy
                                           : (std::abs(My) / e.Iy + std::abs(Mz) / e.Iz) * c;
            const double tau = e.shape == 1 ? std::abs(Tq) * c / e.J : std::abs(Tq) / (2.0 * am * e.t);
            const double sigma = std::abs(N) / e.A + sb;
            vm = std::max(vm, std::sqrt(sigma * sigma + 3.0 * tau * tau));
        }
        res.stress_vm(k) = vm;
        const double P_cr = kPi * kPi * e.E * 1e9 * std::min(e.Iy, e.Iz) * 1e-12 / (e.Lf * e.Lf);
        res.buckling(k) = std::max(0.0, -f(6)) / P_cr;  // f(6) = axial force at j, tension > 0
    }
    res.stress_max = m ? res.stress_vm.maxCoeff() : 0.0;
    res.buckling_ratio = m ? res.buckling.maxCoeff() : 0.0;
}

inline FemResult solve_frame(const FemInput& in) {
    const int n_nodes = static_cast<int>(in.nodes.rows());
    const int n_dof = 6 * n_nodes;
    FemResult res;

    int n_eb = 0;
    std::vector<Eigen::Triplet<double>> trip;
    trip.reserve(144 * in.elements.size() + n_dof);
    std::vector<std::vector<int>> adjacency(n_nodes);
    for (const auto& e : in.elements) {
        if (e.kAy <= 0.0 || e.kAz <= 0.0) ++n_eb;
        res.mass += e.rho * e.A * 1e-6 * e.L;
        const Mat12 Kg = compute_global_stiffness(e);
        const int base[2] = {6 * e.node_i, 6 * e.node_j};
        for (int a = 0; a < 2; ++a)
            for (int b = 0; b < 2; ++b)
                for (int r = 0; r < 6; ++r)
                    for (int c = 0; c < 6; ++c)
                        trip.emplace_back(base[a] + r, base[b] + c, Kg(6 * a + r, 6 * b + c));
        adjacency[e.node_i].push_back(e.node_j);
        adjacency[e.node_j].push_back(e.node_i);
    }
    if (n_eb > 0) {
        res.warnings.push_back(std::to_string(n_eb) +
                               " element(s) without shear area: Euler-Bernoulli (Phi = 0)");
    }

    // Rigid links (RBE2): linked nodes form rigid bodies (union-find, so chains and shared
    // nodes merge); each body's lowest-index node is the master, every other node is a slave
    // with u_s = u_m + theta_m x (x_s - x_m), theta_s = theta_m.
    std::vector<int> root(n_nodes);
    for (int n = 0; n < n_nodes; ++n) root[n] = n;
    auto body_of = [&](int n) {
        while (root[n] != n) n = root[n] = root[root[n]];
        return n;
    };
    for (const auto& link : in.rigid_links) {
        const int a = link.first, b = link.second;
        if (a < 0 || a >= n_nodes || b < 0 || b >= n_nodes || a == b) {
            throw std::runtime_error("Rigid link references invalid nodes");
        }
        const int ra = body_of(a), rb = body_of(b);
        if (ra != rb) root[std::max(ra, rb)] = std::min(ra, rb);
        adjacency[a].push_back(b);
        adjacency[b].push_back(a);
    }
    std::vector<int> master(n_nodes, -1);
    for (int n = 0; n < n_nodes; ++n)
        if (body_of(n) != n) master[n] = body_of(n);

    // Prescribed DOFs (last BC wins); nodes not connected to any BC node are clamped.
    std::vector<char> reachable(n_nodes, 0);
    std::vector<int> queue;
    std::vector<char> has_bc(n_dof, 0);
    Eigen::VectorXd bc_value = Eigen::VectorXd::Zero(n_dof);
    for (const auto& bc : in.bcs) {
        if (bc.node < 0 || bc.node >= n_nodes || bc.dof < 0 || bc.dof > 5) {
            throw std::runtime_error("BC references invalid node/dof");
        }
        has_bc[6 * bc.node + bc.dof] = 1;
        bc_value(6 * bc.node + bc.dof) = bc.value;
        if (!reachable[bc.node]) {
            reachable[bc.node] = 1;
            queue.push_back(bc.node);
        }
    }
    for (size_t q = 0; q < queue.size(); ++q)
        for (int nb : adjacency[queue[q]])
            if (!reachable[nb]) {
                reachable[nb] = 1;
                queue.push_back(nb);
            }

    // u = Tm * q + u_p. Independent DOFs are free or prescribed (eliminated); slave DOFs are
    // combinations of master DOFs. BCs on slave DOFs become Lagrange-multiplier rows.
    std::vector<int> free_index(n_dof, -1);
    Eigen::VectorXd u_p = Eigen::VectorXd::Zero(n_dof);
    int n_free = 0;
    for (int n = 0; n < n_nodes; ++n) {
        if (master[n] != -1) continue;
        for (int d = 0; d < 6; ++d) {
            const int g = 6 * n + d;
            if (!reachable[n]) continue;  // clamped at 0
            if (has_bc[g]) u_p(g) = bc_value(g);
            else free_index[g] = n_free++;
        }
    }
    std::vector<Eigen::Triplet<double>> tt;
    auto add_term = [&](int row, int master_dof, double c) {
        if (c == 0.0) return;
        if (free_index[master_dof] >= 0) tt.emplace_back(row, free_index[master_dof], c);
        else u_p(row) += c * u_p(master_dof);
    };
    for (int n = 0; n < n_nodes; ++n) {
        const int m = master[n];
        if (m == -1) {
            for (int g = 6 * n; g < 6 * n + 6; ++g)
                if (free_index[g] >= 0) tt.emplace_back(g, free_index[g], 1.0);
            continue;
        }
        const Eigen::Vector3d r = (in.nodes.row(n) - in.nodes.row(m)).transpose();
        const int bm = 6 * m, bs = 6 * n;
        for (int d = 0; d < 3; ++d) add_term(bs + d, bm + d, 1.0);  // translation
        add_term(bs + 0, bm + 4, r.z());   // ux += ry*rz - rz*ry
        add_term(bs + 0, bm + 5, -r.y());
        add_term(bs + 1, bm + 5, r.x());   // uy += rz*rx - rx*rz
        add_term(bs + 1, bm + 3, -r.z());
        add_term(bs + 2, bm + 3, r.y());   // uz += rx*ry - ry*rx
        add_term(bs + 2, bm + 4, -r.x());
        for (int d = 3; d < 6; ++d) add_term(bs + d, bm + d, 1.0);  // rotation
    }
    Eigen::SparseMatrix<double> Tm(n_dof, n_free);
    Tm.setFromTriplets(tt.begin(), tt.end());

    std::vector<int> slave_bcs;
    for (int g = 0; g < n_dof; ++g)
        if (has_bc[g] && master[g / 6] != -1) slave_bcs.push_back(g);

    Eigen::VectorXd F = Eigen::VectorXd::Zero(n_dof);
    for (const auto& ld : in.loads) {
        if (ld.node < 0 || ld.node >= n_nodes || ld.dof < 0 || ld.dof > 5) {
            throw std::runtime_error("Load references invalid node/dof");
        }
        F(6 * ld.node + ld.dof) += ld.value;
    }

    Eigen::SparseMatrix<double> K(n_dof, n_dof);
    K.setFromTriplets(trip.begin(), trip.end());
    const Eigen::SparseMatrix<double> TmT = Tm.transpose();
    const Eigen::SparseMatrix<double> Kr = TmT * K * Tm;
    const Eigen::VectorXd Fr = TmT * (F - K * u_p);

    const int n_c = static_cast<int>(slave_bcs.size());
    std::vector<Eigen::Triplet<double>> at;
    for (int k = 0; k < Kr.outerSize(); ++k)
        for (Eigen::SparseMatrix<double>::InnerIterator it(Kr, k); it; ++it) at.emplace_back(it.row(), it.col(), it.value());
    Eigen::VectorXd rhs(n_free + n_c);
    rhs.head(n_free) = Fr;
    const Eigen::SparseMatrix<double, Eigen::RowMajor> TmR = Tm;
    for (int c = 0; c < n_c; ++c) {
        const int g = slave_bcs[c];
        for (Eigen::SparseMatrix<double, Eigen::RowMajor>::InnerIterator it(TmR, g); it; ++it) {
            at.emplace_back(n_free + c, it.col(), it.value());
            at.emplace_back(it.col(), n_free + c, it.value());
        }
        rhs(n_free + c) = bc_value(g) - u_p(g);
    }
    Eigen::SparseMatrix<double> A(n_free + n_c, n_free + n_c);
    A.setFromTriplets(at.begin(), at.end());

    res.U = u_p;
    if (n_free + n_c > 0) {
        Eigen::VectorXd x;
        Eigen::SparseLU<Eigen::SparseMatrix<double>> lu;
        lu.compute(A);
        if (lu.info() == Eigen::Success) x = lu.solve(rhs);
        if (lu.info() != Eigen::Success || !x.allFinite()) {
            // Redundant slave BCs (several pinned nodes on one rigid body) make the multiplier
            // block rank-deficient; rank-revealing QR still solves the consistent system.
            A.makeCompressed();
            Eigen::SparseQR<Eigen::SparseMatrix<double>, Eigen::COLAMDOrdering<int>> qr(A);
            if (qr.info() == Eigen::Success) x = qr.solve(rhs);
            if (qr.info() != Eigen::Success || !x.allFinite() ||
                (A * x - rhs).norm() > 1e-8 * std::max(rhs.norm(), 1.0)) {
                throw std::runtime_error("Singular stiffness: mechanism or insufficient supports");
            }
        }
        res.U += Tm * x.head(n_free);
    }

    res.K_torsion = compute_torsional_rigidity(in.nodes, res.U, in.loads);
    compute_element_stresses(in.elements, res.U, res);
    return res;
}

// JSON glue for the standalone/server front ends. Template so the MEX never
// needs nlohmann::json (only instantiated where Json is available).
template <class Json>
Json solve_json(const Json& input) {
    FemInput in;
    const auto& nj = input.at("nodes");
    in.nodes.resize(static_cast<Eigen::Index>(nj.size()), 3);
    for (size_t i = 0; i < nj.size(); ++i)
        for (int k = 0; k < 3; ++k) in.nodes(i, k) = nj[i][k].template get<double>();

    for (const auto& row_json : input.at("elements")) {
        std::vector<double> row = row_json.template get<std::vector<double>>();
        in.elements.push_back(parse_element_row(row.data(), static_cast<int>(row.size()), in.nodes));
    }
    auto read_triples = [&](const char* key, std::vector<FemLoad>& out) {
        if (!input.contains(key)) return;
        for (const auto& r : input.at(key)) {
            out.push_back({r[0].template get<int>() - 1, r[1].template get<int>() - 1, r[2].template get<double>()});
        }
    };
    read_triples("bcs", in.bcs);
    read_triples("loads", in.loads);
    if (input.contains("rigid_links")) {
        for (const auto& r : input.at("rigid_links")) {
            in.rigid_links.emplace_back(r[0].template get<int>() - 1, r[1].template get<int>() - 1);
        }
    }

    const FemResult res = solve_frame(in);
    Json out;
    out["success"] = true;
    out["mass"] = res.mass;
    out["K_torsion"] = res.K_torsion;
    std::vector<std::vector<double>> U(static_cast<size_t>(in.nodes.rows()), std::vector<double>(6));
    for (size_t i = 0; i < U.size(); ++i)
        for (int d = 0; d < 6; ++d) U[i][d] = res.U(6 * i + d);
    out["U"] = U;
    std::vector<std::vector<double>> ef(static_cast<size_t>(res.element_forces.rows()), std::vector<double>(12));
    for (size_t k = 0; k < ef.size(); ++k)
        for (int c = 0; c < 12; ++c) ef[k][c] = res.element_forces(k, c);
    out["element_forces"] = ef;
    out["element_stress_vm"] = std::vector<double>(res.stress_vm.data(), res.stress_vm.data() + res.stress_vm.size());
    out["element_buckling_ratio"] = std::vector<double>(res.buckling.data(), res.buckling.data() + res.buckling.size());
    out["stress_max"] = res.stress_max;
    out["buckling_ratio"] = res.buckling_ratio;
    if (!res.warnings.empty()) out["warnings"] = res.warnings;
    return out;
}

}  // namespace frame_optimize
