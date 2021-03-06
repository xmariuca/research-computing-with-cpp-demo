// #include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include "ExceptionIcp.h"
#include <exception>
#include "boost/program_options.hpp"

#include "PointCloud.h"
#include "ICPRegistration.h"
#include "utils.h"
#include "defs.h"

using namespace ICP_MPHYG02;
namespace po = boost::program_options;

void printExampleCall()
{
    std::cout << "*** Example of a program call:\n";
    std::cout << "./doPointBasedReg --inputFile1 " << PATH_IN_FIXED_PTS << " --inputFile2 " << PATH_IN_MOVING_PTS << " --outputFile finalTransf.txt" << std::endl;
}

int main(int argc, char** argv)
{
    try
    {
        po::options_description description("PointBasedRegistration Options");
        std::string version("0.0.1");
        std::string inputPathFixed;
        std::string inputPathMoving;
        std::string outputPath;

        description.add_options()
        ("help,h", "Help message")
        ("example,e", "Show an example")
        ("version,v", "Program version")
        ("inputFile1", po::value<std::string>(), "Input file with fixed 3D points")
        ("inputFile2", po::value<std::string>(), "Input file with moving 3D points")
        ("outputFile", po::value<std::string>()->default_value("output.txt")->implicit_value("output.txt"), "Output file for the final transformation matrix");

        po::variables_map vm;
        try
        {
            po::store(po::command_line_parser(argc, argv).options(description).run(), vm);

            if(vm.count("help"))
            {
                std::cout << description << std::endl;
            }
            po::notify(vm);
        }
        catch(po::error& e)
        {
            std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
            std::cerr << description << std::endl;
            return ERROR_CMD_LINE;
        }
        if(vm.count("version"))
        {
            std::cout << version << std::endl;
        }
        if(vm.count("example"))
        {
            printExampleCall();
        }
        if(vm.count("inputFile1"))
        {
            inputPathFixed = vm["inputFile1"].as<std::string>();
            std::cout << "***inputFile1 = " << inputPathFixed << std::endl;
        }
        if(vm.count("inputFile2"))
        {
            inputPathMoving = vm["inputFile2"].as<std::string>();
            std::cout << "***inputFile2 = " << inputPathMoving << std::endl;
        }
        if(vm.count("outputFile"))
        {
            outputPath = vm["outputFile"].as<std::string>();
            std::cout << "***outputFile = " << outputPath << std::endl;
        }

        if( !(inputPathFixed.empty()) && !(inputPathMoving.empty()))
        {
            printMessage("***");
            try
            {
                PointCloud pPCD(inputPathFixed, POINT_BASED_FLAG);
                PointCloud qPCD(inputPathMoving, POINT_BASED_FLAG);
                ICPRegistration PointRegistration(POINT_BASED_FLAG);
                double RMS = 0;
                Eigen::Matrix4d finalTransf;
                PointRegistration.solve(pPCD, qPCD, RMS, finalTransf, outputPath);
            }
            // catch(ExceptionIcp& err)
            // {
            //     std::cout << "********************************\n";
            //     std::cout << err.what();
            //     std::cout << "********************************\n";
            // }
            catch(std::exception& err)
            {
                std::cout << "********************************\n";
                std::cout << err.what();
                std::cout << "********************************\n";
                return UNHANDLED_EXCEPTION;
            }

        }
        // else
        // {
        //     // printMessage("Please insert the correct paths!");
        // }

    }
    catch(std::exception& e)
    {
        std::cout << "********************************\n" << e.what() << std::endl;
        return UNHANDLED_EXCEPTION;
    }

    return SUCCESS;
}
