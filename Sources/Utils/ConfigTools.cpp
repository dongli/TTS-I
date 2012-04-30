#include "ConfigTools.hpp"

string ConfigTools::fileName;
map<string, string> ConfigTools::content;

void ConfigTools::parse(const string &fileName_)
{
    fileName = fileName_;
    ifstream file(fileName.c_str(), std::ios::in);
    if (!file.good()) {
        REPORT_ERROR("Failed to open "+fileName);
    }
    // -------------------------------------------------------------------------
    // parse the configuration file
    int lineNumber = 0, idx, i;
    string line, key, value;
    while (true) {
        lineNumber++;
        getline(file, line);
        if (file.eof())
            break;
        if (line.at(0) == '#')
            continue;
        idx = static_cast<int>(line.find_first_of("="));
        if (idx == string::npos) {
            Message message;
            message << "Invalid line " << lineNumber << " in " << fileName;
            REPORT_ERROR(message.str());
        }
        key.clear();
        for (i = 0; i < idx; ++i)
            if (line.at(i) != ' ') {
                key.push_back(line.at(i));
            } else {
                break;
            }
        for (i = idx+1; i < line.size(); ++i)
            if (line.at(i) == ' ') {
                continue;
            } else {
                value = line.substr(i);
                break;
            }
        content[key] = value;
    }
    // -------------------------------------------------------------------------
    file.close();
}