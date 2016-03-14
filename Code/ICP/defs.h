#ifndef _H_DEF
#define _H_DEF
#include <string>

namespace ICP_MPHYG02
{
    #define POINT_BASED_FLAG false
    #define SURFACE_BASED_FLAG true
    #define SUCCESS 0
    #define ERROR_CMD_LINE 1
    #define UNHANDLED_EXCEPTION 2


    // POINT BASED
    static std::string PATH_IN_FIXED_PTS = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/fixed.txt";
    static std::string PATH_EMPTY_PTS = "emptyInputFile.txt";
    static std::string PATH_IN_MOVING_PTS = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/moving.txt";
    static std::string  PATH_GT_TRANSF_PTS = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/matrix4x4.txt";
    static std::string PATH_OUTPUT_PTS = "../../../research-computing-with-cpp-demo/Testing/PointBasedRegistrationData/outputTest.txt";

    // SURFACE BASED
    static std::string PATH_IN_FIXED_SURF = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/fran_cut.txt";
    static std::string PATH_IN_MOVING_SURF = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/fran_cut_transformed.txt";
    static std::string PATH_IN_FIXED_SURF_SMALL = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/testRead.txt";
    static std::string PATH_IN_MOVING_SURF_SMALL = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/testRead_transformed.txt";
    static std::string  PATH_GT_TRANSF_SURF = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/matrix4x4.txt";
    static std::string PATH_OUTPUT_SURF = "../../../research-computing-with-cpp-demo/Testing/SurfaceBasedRegistrationData/outputTest.txt";

}//namespace ICP_MPHYG02
#endif
