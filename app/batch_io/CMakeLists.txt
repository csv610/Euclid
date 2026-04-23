add_executable(euclid_batch_io main.cpp)
target_include_directories(euclid_batch_io PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_batch_io PRIVATE Euclid::Euclid)
set_target_properties(euclid_batch_io PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)