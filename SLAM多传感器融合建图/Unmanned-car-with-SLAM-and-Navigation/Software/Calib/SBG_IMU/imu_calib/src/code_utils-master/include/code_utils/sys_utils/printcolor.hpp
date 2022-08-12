#ifndef PRINTCOLOR_HPP
#define PRINTCOLOR_HPP

#include <iostream>

namespace sys_utils
{

namespace print_color
{

inline void
PrintWarning( std::string str )
{
    std::cout << "\033[33;40;1m" << str << "\033[0m" << std::endl;
}

inline void
PrintError( std::string str )
{
    std::cout << "\033[31;47;1m" << str << "\033[0m" << std::endl;
}

inline void
PrintInfo( std::string str )
{
    std::cout << "\033[32;40;1m" << str << "\033[0m" << std::endl;
}
}
}
#endif // PRINTCOLOR_HPP
