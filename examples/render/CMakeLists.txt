find_package(Embree3 REQUIRED)

add_executable(render
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_compile_options(render PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
        -pipe -fstack-protector-strong -fno-plt -march=native
        $<$<CONFIG:Debug>:-O0 -Wall -Wextra>>
    $<$<CXX_COMPILER_ID:GNU>:-frounding-math>
    $<$<CXX_COMPILER_ID:MSVC>:
        $<$<CONFIG:Debug>:/Od /W3 /Zi>>
)

target_compile_definitions(render PRIVATE
    EUCLID_NO_WARNING
    $<$<CXX_COMPILER_ID:MSVC>:_SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING>
)

target_include_directories(render PRIVATE
    ${CMAKE_SOURCE_DIR}/3rdparty
    ${CMAKE_BINARY_DIR}/examples
    ${Embree3_INCLUDE_DIRS}
)

target_link_libraries(render PRIVATE
    Euclid::Euclid
    Embree::Embree3
)

set_target_properties(render PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
)