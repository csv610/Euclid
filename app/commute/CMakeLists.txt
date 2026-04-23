add_executable(euclid_commute main.cpp)
target_include_directories(euclid_commute PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_commute PRIVATE Euclid::Euclid)
set_target_properties(euclid_commute PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)