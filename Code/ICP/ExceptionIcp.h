#ifndef _H_ICP_EXCEPTION
#define _H_ICP_EXCEPTION

#include <iostream>
#include <exception>

namespace ICP_MPHYG02
{
    class ExceptionIcp:public std::exception
    {
    public:
      ExceptionIcp(const char* errMsg, const char* filename);
      virtual ~ExceptionIcp() throw(){}

      virtual const char* what() const throw();

     private:
      const char* m_errorMsg;
      const char* m_filename;
    };
}//namespace ICP_MPHYG02
#endif
