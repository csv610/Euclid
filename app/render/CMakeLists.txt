add_executable(euclid_render main.cpp)
target_include_directories(euclid_render PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_render PRIVATE Euclid::Euclid)
set_target_properties(euclid_render PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)