add_executable(euclid_serialize main.cpp)
target_include_directories(euclid_serialize PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_serialize PRIVATE Euclid::Euclid)
set_target_properties(euclid_serialize PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)