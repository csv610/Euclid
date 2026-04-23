add_executable(euclid_surface_recon main.cpp)
target_include_directories(euclid_surface_recon PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_surface_recon PRIVATE Euclid::Euclid)
set_target_properties(euclid_surface_recon PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)