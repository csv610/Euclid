add_executable(euclid_descriptor main.cpp)
target_include_directories(euclid_descriptor PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_descriptor PRIVATE Euclid::Euclid)
set_target_properties(euclid_descriptor PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)