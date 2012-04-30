#ifndef CppHelper_h
#define CppHelper_h

/*
 * This header includes the helper functions for easing daily C++ coding works!
 *
 * Li Dong - dongli@lasg.iap.ac.cn, 2012-04-19
 */

#include <string>

template <class T>
std::string to_string(T t, const char *format)
{
    char tmp[225];
    sprintf(tmp, format, t);
    return std::string(tmp);
}

#endif
