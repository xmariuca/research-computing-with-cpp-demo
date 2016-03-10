#/*============================================================================
#
#  research-computing-with-cpp-demo: CMake based demo code.
#
#  Copyright (c) University College London (UCL). All rights reserved.
#
#  This software is distributed WITHOUT ANY WARRANTY; without even
#  the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
#  PURPOSE.
#
#  See LICENSE.txt in the top level directory for details.
#
#============================================================================*/

add_executable(ICP_main main.cpp PointCloud.cpp utils.cpp)
target_link_libraries(ICP_main ${Boost_LIBRARIES})

add_executable(doPointBasedReg main_PointBasedReg.cpp PointCloud.cpp utils.cpp ICPRegistration.cpp)
target_link_libraries(doPointBasedReg ${Boost_LIBRARIES})

add_executable(doSurfaceBasedReg main_SurfaceBasedReg.cpp PointCloud.cpp utils.cpp ICPRegistration.cpp)
target_link_libraries(doSurfaceBasedReg ${Boost_LIBRARIES})
