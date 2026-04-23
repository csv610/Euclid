
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was EuclidConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

get_filename_component(Euclid_CMAKE_DIR "${CMAKE_CURRENT_LIST_FILE}" PATH)

list(APPEND CMAKE_MODULE_PATH ${Euclid_CMAKE_DIR}/Modules)

include(CMakeFindDependencyMacro)

find_dependency(Boost)
find_dependency(CGAL)
find_dependency(Eigen3)
find_dependency(Libigl)
find_dependency(Threads)

# CGAL tries to override CMAKE_*_FLAGS, do not let it
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL
    "Force GGAL to maintain CMAKE_*_FLAGS"
)

if(NOT TARGET Euclid::Euclid)
    include(${CMAKE_CURRENT_LIST_DIR}/EuclidTargets.cmake)
endif()
