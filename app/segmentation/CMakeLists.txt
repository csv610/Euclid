add_executable(euclid_segmentation main.cpp)
target_include_directories(euclid_segmentation PRIVATE ${CMAKE_BINARY_DIR}/app)
target_link_libraries(euclid_segmentation PRIVATE Euclid::Euclid)
set_target_properties(euclid_segmentation PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/apps)