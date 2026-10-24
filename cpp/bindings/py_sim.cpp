// bindings/py_sim.cpp
/// pybind11 Python bindings for FSAE Simulator

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "fsae/physics/tire/tmeasy.hpp"
#include "fsae/physics/suspension/bouc_wen.hpp"
#include "fsae/physics/suspension/lugre.hpp"
#include "fsae/math/rotation.hpp"

namespace py = pybind11;
using namespace fsae;

PYBIND11_MODULE(fsae_sim_py, m) {
    m.doc() = "FSAE Vehicle Dynamics Simulator - Python bindings";
    
    // === TMeasy Tire Model ===
    py::class_<physics::tire::TMeasyParams<double>>(m, "TMeasyParams")
        .def(py::init<>())
        .def_readwrite("mu_x_max", &physics::tire::TMeasyParams<double>::mu_x_max)
        .def_readwrite("mu_y_max", &physics::tire::TMeasyParams<double>::mu_y_max)
        .def_readwrite("Fz_nom", &physics::tire::TMeasyParams<double>::Fz_nom)
        .def_readwrite("Lx", &physics::tire::TMeasyParams<double>::Lx)
        .def_readwrite("Ly", &physics::tire::TMeasyParams<double>::Ly)
        .def_readwrite("v_N", &physics::tire::TMeasyParams<double>::v_N);
    
    py::class_<physics::tire::TMeasyState<double>>(m, "TMeasyState")
        .def(py::init<>())
        .def_readwrite("xi", &physics::tire::TMeasyState<double>::xi)
        .def_readwrite("eta", &physics::tire::TMeasyState<double>::eta);
    
    py::class_<physics::tire::TMeasyOutput<double>>(m, "TMeasyOutput")
        .def_readonly("Fx", &physics::tire::TMeasyOutput<double>::Fx)
        .def_readonly("Fy", &physics::tire::TMeasyOutput<double>::Fy)
        .def_readonly("Mz", &physics::tire::TMeasyOutput<double>::Mz)
        .def_readonly("dxi_dt", &physics::tire::TMeasyOutput<double>::dxi_dt)
        .def_readonly("deta_dt", &physics::tire::TMeasyOutput<double>::deta_dt);
    
    m.def("compute_tmeasy", &physics::tire::compute_tmeasy<double>,
          "Compute TMeasy tire forces",
          py::arg("params"), py::arg("state"),
          py::arg("Fz"), py::arg("vsx"), py::arg("vsy"), py::arg("vcx"));
    
    // === Bouc-Wen Model ===
    py::class_<physics::suspension::BoucWenParams<double>>(m, "BoucWenParams")
        .def(py::init<>())
        .def_readwrite("c0", &physics::suspension::BoucWenParams<double>::c0)
        .def_readwrite("alpha", &physics::suspension::BoucWenParams<double>::alpha)
        .def_readwrite("beta", &physics::suspension::BoucWenParams<double>::beta)
        .def_readwrite("gamma", &physics::suspension::BoucWenParams<double>::gamma)
        .def_readwrite("n", &physics::suspension::BoucWenParams<double>::n);
    
    m.def("compute_bouc_wen_force",
          &physics::suspension::compute_bouc_wen_force<double>,
          "Compute Bouc-Wen damper force");
    
    m.def("compute_bouc_wen_dz",
          &physics::suspension::compute_bouc_wen_dz<double>,
          "Compute Bouc-Wen hysteresis variable derivative");
    
    // === LuGre Friction ===
    py::class_<physics::suspension::LuGreParams<double>>(m, "LuGreParams")
        .def(py::init<>())
        .def_readwrite("sigma_0", &physics::suspension::LuGreParams<double>::sigma_0)
        .def_readwrite("sigma_1", &physics::suspension::LuGreParams<double>::sigma_1)
        .def_readwrite("Fc", &physics::suspension::LuGreParams<double>::Fc)
        .def_readwrite("Fs", &physics::suspension::LuGreParams<double>::Fs);
    
    // === Rotation ===
    py::class_<math::EulerAngles<double>>(m, "EulerAngles")
        .def(py::init<>())
        .def_readwrite("phi", &math::EulerAngles<double>::phi)
        .def_readwrite("theta", &math::EulerAngles<double>::theta)
        .def_readwrite("psi", &math::EulerAngles<double>::psi);
    
    m.def("euler_to_rotation_matrix",
          &math::euler_to_rotation_matrix<double>,
          "Convert Euler angles to rotation matrix");
    
    // Version info
    m.attr("__version__") = "0.1.0";
}
