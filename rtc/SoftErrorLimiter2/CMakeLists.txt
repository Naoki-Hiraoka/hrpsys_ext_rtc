set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  joint_limit_table
  )
find_package(choreonoid REQUIRED)
find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)

find_package(PkgConfig)
pkg_check_modules(openrtm_aist openrtm-aist REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${hrpsys_INCLUDE_DIRS}
  ${openrtm_aist_INCLUDE_DIRS}
  )

link_directories(
  ${CHOREONOID_LIBRARY_DIRS}
  ${openrtm_aist_LIBRARY_DIRS}
  )

add_library(SoftErrorLimiter2 SHARED
  SoftErrorLimiter2.cpp
  SoftErrorLimiter2Service_impl.cpp
  beep.cpp
  )
target_link_libraries(SoftErrorLimiter2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  ${openrtm_aist_LIBRARIES}
  ${${PROJECT_NAME}_IDLLIBRARY_DIRS}
  HRPDataTypesSkel HRPDataTypesStub hrpsysUtil
  )
set_target_properties(SoftErrorLimiter2 PROPERTIES PREFIX "") # libSoftErrorLimiter2.so -> SoftErrorLimiter2.so
add_dependencies(SoftErrorLimiter2 RTMBUILD2_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

add_executable(SoftErrorLimiter2Comp SoftErrorLimiter2Comp.cpp)
target_link_libraries(SoftErrorLimiter2Comp SoftErrorLimiter2)
