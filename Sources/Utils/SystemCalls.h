#ifndef _SystemCalls_h_
#define _SystemCalls_h_

#include <string>
#include <vector>

class SystemCalls
{
public:
    SystemCalls();
    virtual ~SystemCalls();

    static int getNumFile(const std::string &dir);
    static int getNumFile(const std::string &dir,
                          const std::string &filePattern);
    static void getFiles(const std::string &dir,
                         const std::string &filePattern,
                         std::vector<std::string> &fileNames);
};

#endif
