add_executable(euclid_primitive main.cpp)
target_include_directories(euclid_primitive PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_primitive PRIVATE Euclid::Euclid)
set_target_properties(euclid_primitive PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)