add_executable(view_selection
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
)

target_compile_options(view_selection PRIVATE
    $<$<OR:$<CXX_COMPILER_ID:Clang>,$<CXX_COMPILER_ID:GNU>>:
        -pipe -fstack-protector-strong -fno-plt -march=native
        $<$<CONFIG:Debug>:-O0 -Wall -Wextra>>
    $<$<CXX_COMPILER_ID:GNU>:-frounding-math>
    $<$<CXX_COMPILER_ID:MSVC>:
        $<$<CONFIG:Debug>:/Od /W3 /Zi>>
)

target_compile_definitions(view_selection PRIVATE
    EUCLID_NO_WARNING
    $<$<CXX_COMPILER_ID:MSVC>:_SILENCE_CXX17_NEGATORS_DEPRECATION_WARNING>
)

target_include_directories(view_selection PRIVATE
    ${CMAKE_SOURCE_DIR}/3rdparty
    ${CMAKE_BINARY_DIR}/examples
)

target_link_libraries(view_selection PRIVATE
    Euclid::Euclid
)

set_target_properties(view_selection PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/examples
)