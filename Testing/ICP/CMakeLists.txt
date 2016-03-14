
add_executable(test_reg main_Tests.cc PointBasedTests.cc SurfaceBasedTests.cc ../../Code/ICP/PointCloud.cpp ../../Code/ICP/utils.cpp ../../Code/ICP/ICPRegistration.cpp ../../Code/ICP/ExceptionIcp.cpp)
target_link_libraries(test_reg ${Boost_LIBRARIES})
