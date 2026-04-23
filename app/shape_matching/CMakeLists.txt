add_executable(euclid_shape_matching main.cpp)
target_include_directories(euclid_shape_matching PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_shape_matching PRIVATE Euclid::Euclid)
set_target_properties(euclid_shape_matching PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)