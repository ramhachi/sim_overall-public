/**
 * @file beam_fem_solver.cpp
 * @brief MEX interface for the 3D Timoshenko beam FEM kernel (cpp/beam_fem_core.hpp)
 *
 * MATLAB Interface:
 *   result = beam_fem_solver(nodes, elements, bcs, loads, options);
 *
 * Inputs:
 *   nodes    - Nx3 double [x, y, z] in meters
 *   elements - Mx13 double [node_i, node_j, D_outer, t, E, G, rho, shape, jf, kAy, kAz, off_i, off_j]
 *              shape: 1=round, 2=square; kAy/kAz: shear areas [mm^2];
 *              off_i/off_j: rigid end lengths [m]. Mx11 (no offsets) and legacy
 *              Mx9 (Phi = 0, Euler-Bernoulli, with a warning) are accepted.
 *   bcs      - Bx3 double [node_id, dof, value], dof 1-6 (ux, uy, uz, rx, ry, rz)
 *   loads    - Lx3 double [node_id, dof, value], dof 1-6 (Fx, Fy, Fz, Mx, My, Mz)
 *   options  - struct (optional): .rigid_links Rx2 [master, slave] node ids (RBE2)
 *
 * Output struct: U (Nx6), K_torsion [Nm/deg], mass [kg], success,
 *   element_forces (Mx12, local end forces at the flexible ends [Fx Fy Fz Mx My Mz]_i,_j in N, N*m),
 *   element_stress_vm (Mx1, MPa), stress_max (MPa), element_buckling_ratio (Mx1, N/P_cr), buckling_ratio
 */

#include "mex.h"
#include "../cpp/beam_fem_core.hpp"

#include <vector>

namespace fo = frame_optimize;

static std::vector<fo::FemLoad> read_triples(const mxArray* a) {
    std::vector<fo::FemLoad> out;
    if (mxIsEmpty(a)) return out;
    if (!mxIsDouble(a) || mxGetN(a) != 3) {
        mexErrMsgIdAndTxt("beamFEM:InvalidInput", "bcs/loads must be Bx3 double arrays");
    }
    const double* p = mxGetPr(a);
    const mwSize m = mxGetM(a);
    for (mwSize r = 0; r < m; ++r) {
        out.push_back({static_cast<int>(p[r]) - 1, static_cast<int>(p[r + m]) - 1, p[r + 2 * m]});
    }
    return out;
}

static mxArray* to_mx(const Eigen::MatrixXd& m) {  // Eigen and MATLAB are both column-major
    mxArray* a = mxCreateDoubleMatrix(m.rows(), m.cols(), mxREAL);
    std::copy(m.data(), m.data() + m.size(), mxGetPr(a));
    return a;
}

void mexFunction(int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[]) {
    (void)nlhs;
    if (nrhs < 4 || nrhs > 5) {
        mexErrMsgIdAndTxt("beamFEM:InvalidInput", "Usage: result = beam_fem_solver(nodes, elements, bcs, loads, [options])");
    }
    if (!mxIsDouble(prhs[0]) || mxGetN(prhs[0]) != 3) {
        mexErrMsgIdAndTxt("beamFEM:InvalidNodes", "Nodes must be Nx3 double array");
    }
    if (!mxIsDouble(prhs[1])) {
        mexErrMsgIdAndTxt("beamFEM:InvalidElements", "Elements must be a double array");
    }

    fo::FemInput in;
    fo::FemResult res;
    try {
        const mwSize n_nodes = mxGetM(prhs[0]);
        const double* np = mxGetPr(prhs[0]);
        in.nodes.resize(n_nodes, 3);
        for (mwSize i = 0; i < n_nodes; ++i)
            for (int k = 0; k < 3; ++k) in.nodes(i, k) = np[i + k * n_nodes];

        const mwSize n_el = mxGetM(prhs[1]);
        const int ncols = static_cast<int>(mxGetN(prhs[1]));
        const double* ep = mxGetPr(prhs[1]);
        std::vector<double> row(ncols);
        for (mwSize e = 0; e < n_el; ++e) {
            for (int c = 0; c < ncols; ++c) row[c] = ep[e + c * n_el];
            in.elements.push_back(fo::parse_element_row(row.data(), ncols, in.nodes));
        }
        in.bcs = read_triples(prhs[2]);
        in.loads = read_triples(prhs[3]);
        const mxArray* links = nrhs == 5 && mxIsStruct(prhs[4]) ? mxGetField(prhs[4], 0, "rigid_links") : nullptr;
        if (links && !mxIsEmpty(links)) {
            if (!mxIsDouble(links) || mxGetN(links) != 2) throw std::runtime_error("options.rigid_links must be Rx2");
            const double* lp = mxGetPr(links);
            const mwSize nl = mxGetM(links);
            for (mwSize k = 0; k < nl; ++k)
                in.rigid_links.emplace_back(static_cast<int>(lp[k]) - 1, static_cast<int>(lp[k + nl]) - 1);
        }
        res = fo::solve_frame(in);
    } catch (const std::exception& ex) {
        mexErrMsgIdAndTxt("beamFEM:SolverFailed", "%s", ex.what());
    }
    for (const auto& w : res.warnings) {
        mexWarnMsgIdAndTxt("beamFEM:Warning", "%s", w.c_str());
    }

    const char* fields[] = {"U", "K_torsion", "mass", "success", "element_forces", "element_stress_vm",
                            "stress_max", "element_buckling_ratio", "buckling_ratio"};
    plhs[0] = mxCreateStructMatrix(1, 1, 9, fields);
    const mwSize n_nodes = in.nodes.rows();
    mxArray* U = mxCreateDoubleMatrix(n_nodes, 6, mxREAL);
    double* up = mxGetPr(U);
    for (mwSize i = 0; i < n_nodes; ++i)
        for (int d = 0; d < 6; ++d) up[i + d * n_nodes] = res.U(6 * i + d);
    mxSetField(plhs[0], 0, "U", U);
    mxSetField(plhs[0], 0, "K_torsion", mxCreateDoubleScalar(res.K_torsion));
    mxSetField(plhs[0], 0, "mass", mxCreateDoubleScalar(res.mass));
    mxSetField(plhs[0], 0, "success", mxCreateDoubleScalar(1.0));
    mxSetField(plhs[0], 0, "element_forces", to_mx(res.element_forces));
    mxSetField(plhs[0], 0, "element_stress_vm", to_mx(res.stress_vm));
    mxSetField(plhs[0], 0, "stress_max", mxCreateDoubleScalar(res.stress_max));
    mxSetField(plhs[0], 0, "element_buckling_ratio", to_mx(res.buckling));
    mxSetField(plhs[0], 0, "buckling_ratio", mxCreateDoubleScalar(res.buckling_ratio));
}
