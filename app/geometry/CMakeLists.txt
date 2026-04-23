add_executable(euclid_geometry main.cpp)
target_include_directories(euclid_geometry PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_geometry PRIVATE Euclid::Euclid)
set_target_properties(euclid_geometry PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)