add_executable(euclid_ttk main.cpp)
target_include_directories(euclid_ttk PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_ttk PRIVATE Euclid::Euclid)
set_target_properties(euclid_ttk PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)