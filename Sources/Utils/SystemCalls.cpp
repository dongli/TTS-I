#include "SystemCalls.hpp"
#include "ReportMacros.hpp"
#include <cstdlib>

int SystemCalls::getNumFile(const std::string &dir)
{
    std::string cmd = "ls "+dir;

    FILE *pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        std::string message = "Failed to call system command \""+cmd+"\".";
        REPORT_ERROR(message)
    }

    char buffer[128];
    std::string result;

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    pclose(pipe);

    int numFile = 0;
    int pos = 0;
    while (pos < result.length()) {
        pos = static_cast<int>(result.find("\n", pos))+1;
        numFile++;
    }

    return numFile;
}

int SystemCalls::getNumFile(const std::string &dir,
                            const std::string &filePattern)
{
    std::string cmd = "find "+dir+" -name \""+filePattern+"\"";

    FILE *pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        std::string message = "Failed to call system command \""+cmd+"\".";
        REPORT_ERROR(message)
    }

    char buffer[128];
    std::string result;

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    pclose(pipe);

    int numFile = 0;
    int pos = 0;
    while (pos < result.length()) {
        pos = static_cast<int>(result.find("\n", pos))+1;
        numFile++;
    }

    return numFile;
}

void SystemCalls::getFiles(const std::string &dir,
                           const std::string &filePattern,
                           std::vector<std::string> &fileNames)
{
    std::string cmd = "find "+dir+" -name \""+filePattern+"\"";

    FILE *pipe = popen(cmd.c_str(), "r");

    if (!pipe) {
        std::string message = "Failed to call system command \""+cmd+"\".";
        REPORT_ERROR(message)
    }

    char buffer[128];
    std::string result;

    while (!feof(pipe))
        if (fgets(buffer, 128, pipe) != NULL)
            result += buffer;

    pclose(pipe);

    int pos = 0, prev_pos;
    while (pos < result.length()) {
        prev_pos = pos;
        pos = static_cast<int>(result.find("\n", pos))+1;
        fileNames.push_back(result.substr(prev_pos, pos-1-prev_pos));
    }
}
