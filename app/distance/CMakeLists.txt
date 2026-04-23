add_executable(euclid_distance main.cpp)
target_include_directories(euclid_distance PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_distance PRIVATE Euclid::Euclid)
set_target_properties(euclid_distance PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)