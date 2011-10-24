#ifndef _ReportMacros_h_
#define _ReportMacros_h_

#include <iostream>
#include <sstream>

using std::cout;
using std::endl;
using std::ostringstream;

typedef ostringstream Message;

#define REPORT_ERROR(MESSAGE) \
{ \
    cout << "[Error]: " << __FILE__ << ":" << __LINE__ << ": " << \
            MESSAGE << endl; \
    exit(1); \
}

#define REPORT_WARNING(MESSAGE) \
{ \
    cout << "[Warning]: " << __FILE__ << ":" << __LINE__ << ": " << \
            MESSAGE << endl; \
}

#define REPORT_DEBUG \
{ \
    cout << "[Debug]: " << __FILE__ << ":" << __LINE__ << endl; \
}

#define NOTICE(CALLER, MESSAGE) \
{ \
    cout << "[Notice]: " << CALLER << ": " << MESSAGE << endl; \
}

#define REPORT_ONLINE(CLASS) \
{ \
    cout << "[Notice]: " << CLASS << " is online." << endl; \
}

#define REPORT_OFFLINE(CLASS) \
{ \
    cout << "[Notice]: " << CLASS << " is offline." << endl; \
}

#define SEPERATOR \
{ \
    cout << "-------------------------------------------------------" << endl; \
}

#endif
