add_executable(euclid_stats main.cpp)
target_include_directories(euclid_stats PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_stats PRIVATE Euclid::Euclid)
set_target_properties(euclid_stats PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)