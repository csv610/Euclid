add_executable(euclid_topology main.cpp)
target_include_directories(euclid_topology PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_topology PRIVATE Euclid::Euclid)
set_target_properties(euclid_topology PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)