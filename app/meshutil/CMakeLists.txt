add_executable(euclid_meshutil main.cpp)
target_include_directories(euclid_meshutil PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_meshutil PRIVATE Euclid::Euclid)
set_target_properties(euclid_meshutil PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)