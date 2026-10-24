# bindings/CMakeLists.txt
# Python bindings using pybind11

pybind11_add_module(fsae_sim_py
    py_sim.cpp
)

target_link_libraries(fsae_sim_py PRIVATE
    fsae_core
)

# 出力先をpythonパッケージに設定
set_target_properties(fsae_sim_py PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_SOURCE_DIR}/python/fsae_sim"
)
