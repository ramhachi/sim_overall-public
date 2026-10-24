/**
 * @file beam_fem_solver.cpp
 * @brief MEX interface for 3D Timoshenko beam FEM solver with joint corrections
 *
 * MATLAB Interface:
 *   result = beam_fem_solver(nodes, elements, bcs, loads, options);
 *
 * Inputs:
 *   nodes    - Nx3 double [x, y, z] in meters
 *   elements - Mx9 double [node_i, node_j, D_outer, t, E, G, rho, shape, joint_factor]
 *              shape: 1=round, 2=square
 *              joint_factor: 0.55-1.0 (1.0 = no correction)
 *   bcs      - Bx3 double [node_id, dof, value]
 *              dof: 1-6 (ux, uy, uz, rx, ry, rz), 1-indexed
 *   loads    - Lx3 double [node_id, dof, value]
 *              dof: 1-6 (Fx, Fy, Fz, Mx, My, Mz), 1-indexed
 *   options  - struct with optional fields (currently unused)
 *
 * Output:
 *   result   - struct with fields:
 *              U           - Nx6 displacement [ux, uy, uz, rx, ry, rz]
 *              K_torsion   - Torsional rigidity [Nm/deg]
 *              mass        - Total mass [kg]
 *              element_forces - Mx12 [N1, N2, Vy1, Vy2, Vz1, Vz2, T1, T2, My1, My2, Mz1, Mz2]
 *              success     - 1 if solve succeeded, 0 otherwise
 */

#include "mex.h"
#include "../cpp/torsion_rigidity.hpp"
#include <Eigen/Sparse>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;
typedef Triplet<double> T;

// Constants
const double PI = 3.14159265358979323846;

// Structure to hold element data
struct BeamElement {
    int node_i, node_j;
    double D_outer, t, E, G, rho;
    int shape;  // 1=round, 2=square
    double joint_factor;
    double A, Iy, Iz, J;  // Section properties
    double L;             // Length
    Matrix3d R;           // Rotation matrix (local to global)
};

// Compute section properties
void computeSectionProperties(BeamElement &elem) {
    double D = elem.D_outer;
    double t_val = elem.t;
    double D_inner = D - 2*t_val;

    if (elem.shape == 1) {  // Round
        elem.A = PI/4 * (D*D - D_inner*D_inner);
        elem.Iy = PI/64 * (pow(D,4) - pow(D_inner,4));
        elem.Iz = elem.Iy;
        elem.J = PI/32 * (pow(D,4) - pow(D_inner,4));
    } else if (elem.shape == 2) {  // Square
        elem.A = D*D - D_inner*D_inner;
        elem.Iy = (pow(D,4) - pow(D_inner,4)) / 12.0;
        elem.Iz = elem.Iy;
        double b = D - t_val;
        elem.J = 2 * t_val * pow(b,3) / (1 + 0.63 * t_val / b);
    } else {
        mexErrMsgIdAndTxt("beamFEM:InvalidShape", "Shape must be 1 (round) or 2 (square)");
    }
}

// Compute rotation matrix from local to global coordinates
Matrix3d computeRotationMatrix(const Vector3d &p1, const Vector3d &p2) {
    Vector3d x_local = (p2 - p1).normalized();

    // Choose y_local perpendicular to x_local
    Vector3d y_temp;
    if (std::abs(x_local(2)) < 0.9) {
        y_temp = Vector3d(0, 0, 1);
    } else {
        y_temp = Vector3d(0, 1, 0);
    }

    Vector3d z_local = x_local.cross(y_temp).normalized();
    Vector3d y_local = z_local.cross(x_local).normalized();

    Matrix3d R;
    R.col(0) = x_local;
    R.col(1) = y_local;
    R.col(2) = z_local;

    return R;
}

// Compute 12x12 element stiffness matrix in local coordinates
MatrixXd computeLocalStiffness(const BeamElement &elem) {
    double L = elem.L;
    double E = elem.E * 1e9;  // GPa to Pa
    double G = elem.G * 1e9;
    double A = elem.A * 1e-6;   // mm^2 to m^2
    double Iy = elem.Iy * 1e-12; // mm^4 to m^4
    double Iz = elem.Iz * 1e-12;
    double J = elem.J * 1e-12;

    // Joint correction (only apply to bending and torsion)
    double jf = elem.joint_factor;

    MatrixXd K_local = MatrixXd::Zero(12, 12);

    // Axial (no joint correction)
    double EA_L = E*A/L;
    K_local(0,0) = EA_L;   K_local(0,6) = -EA_L;
    K_local(6,0) = -EA_L;  K_local(6,6) = EA_L;

    // Torsion (with joint correction)
    double GJ_L = jf * G*J/L;
    K_local(3,3) = GJ_L;   K_local(3,9) = -GJ_L;
    K_local(9,3) = -GJ_L;  K_local(9,9) = GJ_L;

    // Bending in y-z plane (with joint correction)
    double EIz_L3 = jf * E*Iz/(L*L*L);
    K_local(1,1) = 12*EIz_L3;  K_local(1,5) = 6*EIz_L3*L;   K_local(1,7) = -12*EIz_L3;  K_local(1,11) = 6*EIz_L3*L;
    K_local(5,1) = 6*EIz_L3*L; K_local(5,5) = 4*EIz_L3*L*L; K_local(5,7) = -6*EIz_L3*L; K_local(5,11) = 2*EIz_L3*L*L;
    K_local(7,1) = -12*EIz_L3; K_local(7,5) = -6*EIz_L3*L;  K_local(7,7) = 12*EIz_L3;   K_local(7,11) = -6*EIz_L3*L;
    K_local(11,1) = 6*EIz_L3*L; K_local(11,5) = 2*EIz_L3*L*L; K_local(11,7) = -6*EIz_L3*L; K_local(11,11) = 4*EIz_L3*L*L;

    // Bending in x-z plane (with joint correction)
    double EIy_L3 = jf * E*Iy/(L*L*L);
    K_local(2,2) = 12*EIy_L3;   K_local(2,4) = -6*EIy_L3*L;  K_local(2,8) = -12*EIy_L3;  K_local(2,10) = -6*EIy_L3*L;
    K_local(4,2) = -6*EIy_L3*L; K_local(4,4) = 4*EIy_L3*L*L; K_local(4,8) = 6*EIy_L3*L;  K_local(4,10) = 2*EIy_L3*L*L;
    K_local(8,2) = -12*EIy_L3;  K_local(8,4) = 6*EIy_L3*L;   K_local(8,8) = 12*EIy_L3;   K_local(8,10) = 6*EIy_L3*L;
    K_local(10,2) = -6*EIy_L3*L; K_local(10,4) = 2*EIy_L3*L*L; K_local(10,8) = 6*EIy_L3*L; K_local(10,10) = 4*EIy_L3*L*L;

    return K_local;
}

// Transform local stiffness to global coordinates
MatrixXd transformToGlobal(const MatrixXd &K_local, const Matrix3d &R) {
    // Build 12x12 transformation matrix
    MatrixXd T = MatrixXd::Zero(12, 12);
    for (int i = 0; i < 4; i++) {
        T.block<3,3>(3*i, 3*i) = R;
    }

    return T.transpose() * K_local * T;
}

// MEX entry point
void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[]) {
    // Check inputs
    if (nrhs < 4 || nrhs > 5) {
        mexErrMsgIdAndTxt("beamFEM:InvalidInput", "Usage: result = beam_fem_solver(nodes, elements, bcs, loads, [options])");
    }

    // Parse nodes (Nx3)
    if (!mxIsDouble(prhs[0]) || mxGetN(prhs[0]) != 3) {
        mexErrMsgIdAndTxt("beamFEM:InvalidNodes", "Nodes must be Nx3 double array");
    }
    const double *nodes_ptr = mxGetPr(prhs[0]);
    int n_nodes = mxGetM(prhs[0]);

    // Parse elements (Mx9)
    if (!mxIsDouble(prhs[1]) || mxGetN(prhs[1]) != 9) {
        mexErrMsgIdAndTxt("beamFEM:InvalidElements", "Elements must be Mx9 double array");
    }
    const double *elems_ptr = mxGetPr(prhs[1]);
    int n_elems = mxGetM(prhs[1]);

    // Parse BCs (Bx3)
    const double *bcs_ptr = mxGetPr(prhs[2]);
    int n_bcs = mxGetM(prhs[2]);

    // Parse loads (Lx3)
    const double *loads_ptr = mxGetPr(prhs[3]);
    int n_loads = mxGetM(prhs[3]);

    // Build node position matrix
    MatrixXd nodes(n_nodes, 3);
    for (int i = 0; i < n_nodes; i++) {
        nodes(i, 0) = nodes_ptr[i];
        nodes(i, 1) = nodes_ptr[i + n_nodes];
        nodes(i, 2) = nodes_ptr[i + 2*n_nodes];
    }

    // Build element array
    std::vector<BeamElement> elements(n_elems);
    double total_mass = 0.0;

    for (int e = 0; e < n_elems; e++) {
        elements[e].node_i = (int)elems_ptr[e] - 1;  // MATLAB 1-indexed to C++ 0-indexed
        elements[e].node_j = (int)elems_ptr[e + n_elems] - 1;
        elements[e].D_outer = elems_ptr[e + 2*n_elems];
        elements[e].t = elems_ptr[e + 3*n_elems];
        elements[e].E = elems_ptr[e + 4*n_elems];
        elements[e].G = elems_ptr[e + 5*n_elems];
        elements[e].rho = elems_ptr[e + 6*n_elems];
        elements[e].shape = (int)elems_ptr[e + 7*n_elems];
        elements[e].joint_factor = elems_ptr[e + 8*n_elems];

        // Compute section properties
        computeSectionProperties(elements[e]);

        // Compute length and rotation matrix
        Vector3d p1 = nodes.row(elements[e].node_i);
        Vector3d p2 = nodes.row(elements[e].node_j);
        elements[e].L = (p2 - p1).norm();
        elements[e].R = computeRotationMatrix(p1, p2);

        // Compute mass (in kg)
        total_mass += elements[e].rho * elements[e].A * 1e-6 * elements[e].L;
    }

    // Assemble global stiffness matrix
    int n_dof = 6 * n_nodes;
    std::vector<T> triplets;
    triplets.reserve(144 * n_elems);  // 12x12 = 144 entries per element

    for (int e = 0; e < n_elems; e++) {
        MatrixXd K_local = computeLocalStiffness(elements[e]);
        MatrixXd K_global = transformToGlobal(K_local, elements[e].R);

        int i = elements[e].node_i;
        int j = elements[e].node_j;

        // Add to global matrix
        for (int ii = 0; ii < 6; ii++) {
            for (int jj = 0; jj < 6; jj++) {
                triplets.push_back(T(6*i + ii, 6*i + jj, K_global(ii, jj)));
                triplets.push_back(T(6*i + ii, 6*j + jj, K_global(ii, jj+6)));
                triplets.push_back(T(6*j + ii, 6*i + jj, K_global(ii+6, jj)));
                triplets.push_back(T(6*j + ii, 6*j + jj, K_global(ii+6, jj+6)));
            }
        }
    }

    SparseMatrix<double> K(n_dof, n_dof);
    K.setFromTriplets(triplets.begin(), triplets.end());

    // Detect floating nodes (not connected to any element)
    std::vector<bool> node_connected(n_nodes, false);
    for (int e = 0; e < n_elems; e++) {
        node_connected[elements[e].node_i] = true;
        node_connected[elements[e].node_j] = true;
    }

    // Apply boundary conditions (penalty method)
    VectorXd F = VectorXd::Zero(n_dof);
    double penalty = 1e15;

    // Constrain all DOFs of floating nodes to prevent singular matrix
    for (int n = 0; n < n_nodes; n++) {
        if (!node_connected[n]) {
            for (int d = 0; d < 6; d++) {
                K.coeffRef(6*n + d, 6*n + d) += penalty;
            }
        }
    }

    for (int b = 0; b < n_bcs; b++) {
        int node = (int)bcs_ptr[b] - 1;
        int dof = (int)bcs_ptr[b + n_bcs] - 1;
        double value = bcs_ptr[b + 2*n_bcs];

        int global_dof = 6*node + dof;
        K.coeffRef(global_dof, global_dof) += penalty;
        F(global_dof) += penalty * value;
    }

    // Apply loads
    std::vector<frame_optimize::FemLoad> fem_loads;
    fem_loads.reserve(n_loads);
    for (int l = 0; l < n_loads; l++) {
        int node = (int)loads_ptr[l] - 1;
        int dof = (int)loads_ptr[l + n_loads] - 1;
        double value = loads_ptr[l + 2*n_loads];

        int global_dof = 6*node + dof;
        F(global_dof) += value;
        fem_loads.push_back({node, dof, value});
    }

    // Solve K*U = F
    SparseLU<SparseMatrix<double>> solver;
    solver.compute(K);
    if (solver.info() != Success) {
        mexErrMsgIdAndTxt("beamFEM:SolverFailed", "Sparse LU decomposition failed");
    }

    VectorXd U = solver.solve(F);
    if (solver.info() != Success) {
        mexErrMsgIdAndTxt("beamFEM:SolveFailed", "Linear solve failed");
    }

    // Compute torsional rigidity from either direct Rx moments or distributed
    // vertical couple loads using the same metric as the TCP/standalone solvers.
    double K_torsion = frame_optimize::compute_torsional_rigidity(nodes, U, fem_loads);

    // Create output struct
    const char *field_names[] = {"U", "K_torsion", "mass", "success"};
    plhs[0] = mxCreateStructMatrix(1, 1, 4, field_names);

    // U: Nx6 displacement matrix
    mxArray *U_out = mxCreateDoubleMatrix(n_nodes, 6, mxREAL);
    double *U_ptr = mxGetPr(U_out);
    for (int i = 0; i < n_nodes; i++) {
        for (int j = 0; j < 6; j++) {
            U_ptr[i + j*n_nodes] = U(6*i + j);
        }
    }
    mxSetField(plhs[0], 0, "U", U_out);

    // K_torsion: scalar
    mxSetField(plhs[0], 0, "K_torsion", mxCreateDoubleScalar(K_torsion));

    // mass: scalar
    mxSetField(plhs[0], 0, "mass", mxCreateDoubleScalar(total_mass));

    // success: 1
    mxSetField(plhs[0], 0, "success", mxCreateDoubleScalar(1.0));
}
