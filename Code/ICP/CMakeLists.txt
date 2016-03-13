
add_executable(ICP_main main.cpp PointCloud.cpp utils.cpp)
target_link_libraries(ICP_main ${Boost_LIBRARIES})

add_executable(doPointBasedReg main_PointBasedReg.cpp PointCloud.cpp utils.cpp ICPRegistration.cpp)
target_link_libraries(doPointBasedReg ${Boost_LIBRARIES})

add_executable(doSurfaceBasedReg main_SurfaceBasedReg.cpp PointCloud.cpp utils.cpp ICPRegistration.cpp)
target_link_libraries(doSurfaceBasedReg ${Boost_LIBRARIES})
