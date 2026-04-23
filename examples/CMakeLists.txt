configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/config.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/config.h
)

add_subdirectory(hello_world)
add_subdirectory(spectral)
add_subdirectory(random_walk)
add_subdirectory(spin_image)
add_subdirectory(primitive)
add_subdirectory(geodesics)
add_subdirectory(topology)
add_subdirectory(scp)
add_subdirectory(view_selection)
add_subdirectory(render)
add_subdirectory(bounding_volume)
add_subdirectory(statistics)
add_subdirectory(wks)
