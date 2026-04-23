add_executable(euclid_view_selection main.cpp)
target_include_directories(euclid_view_selection PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_view_selection PRIVATE Euclid::Euclid)
set_target_properties(euclid_view_selection PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)