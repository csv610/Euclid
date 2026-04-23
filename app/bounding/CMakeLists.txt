add_executable(euclid_bounding main.cpp)
target_include_directories(euclid_bounding PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_bounding PRIVATE Euclid::Euclid)
set_target_properties(euclid_bounding PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)