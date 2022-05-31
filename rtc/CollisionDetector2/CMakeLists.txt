add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  choreonoid_qhull
  choreonoid_vclip
  )

find_package(choreonoid REQUIRED)
# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

find_package(PkgConfig)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${hrpsys_INCLUDE_DIRS}
  )

rtmbuild_add_library(CollisionDetector2 SHARED
  CollisionDetector2.cpp
  CollisionDetector2Service_impl.cpp
  ../SoftErrorLimiter2/beep.cpp
  )
target_link_libraries(CollisionDetector2
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  HRPDataTypesSkel HRPDataTypesStub
  CollisionDetectorServiceSkel CollisionDetectorServiceStub
  )
set_target_properties(CollisionDetector2 PROPERTIES PREFIX "") # libCollisionDetector2.so -> CollisionDetector2.so
#add_dependencies(CollisionDetector2 RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(CollisionDetector2Comp CollisionDetector2Comp.cpp)
target_link_libraries(CollisionDetector2Comp CollisionDetector2)
