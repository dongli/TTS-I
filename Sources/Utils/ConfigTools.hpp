#ifndef ConfigTools_h
#define ConfigTools_h

#include "ReportMacros.hpp"
#include <string>
#include <map>
#include <fstream>

using std::string;
using std::map;
using std::ifstream;
using std::istringstream;

class ConfigTools
{
public:
    ConfigTools() {}
    ~ConfigTools() {}

    static void parse(const string &fileName);

    template <typename VALUETYPE>
    static void read(const string &key, VALUETYPE &value);

    template <typename VALUETYPE>
    static void read(const string &key, int numValue, VALUETYPE *values);

private:
    static string fileName;
    static map<string, string> content;
};

// Note: Due to the use of template and static member function, we can not
//       include the implementation of "parse" into this header file.
template <typename VALUETYPE>
void ConfigTools::read(const string &key, VALUETYPE &value)
{
    if (content.count(key) == 0) {
        Message message;
        message << "Not found entry " << key << " in " << fileName;
        REPORT_ERROR(message.str());
    }
    istringstream iss(content[key]);
    iss >> value;
}

template <typename VALUETYPE>
void ConfigTools::read(const string &key, int numValue, VALUETYPE *values)
{
    if (content.count(key) == 0) {
        Message message;
        message << "Not found entry " << key << " in " << fileName;
        REPORT_ERROR(message.str());
    }
    istringstream iss1(content[key]);
    string value;
    int counter = 0;
    while (getline(iss1, value, ',')) {
        if (counter >= numValue) {
            Message message;
            message << "Size of array entry mismatches (expected: ";
            message << numValue << ")";
            REPORT_ERROR(message.str());
        }
        istringstream iss2(value);
        iss2 >> values[counter++];
    }
}

#endif