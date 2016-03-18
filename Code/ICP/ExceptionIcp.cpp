#include "ExceptionIcp.h"
#include "defs.h"
#include <string>

namespace ICP_MPHYG02
{
    ExceptionIcp::ExceptionIcp(const char* errMsg, const char* filename):m_errorMsg(errMsg),m_filename(filename)
    {
    }
    const char* ExceptionIcp::what() const throw()
    {
        char* result= new char[300];
        strcpy(result,"Error: ");
        strcat(result,m_errorMsg);
        strcat(result,"\nFile: ");
        strcat(result,m_filename);
        strcat(result,"\n");
        return result;
    }
}
