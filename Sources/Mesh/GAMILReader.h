#ifndef _GAMILReader_h_
#define _GAMILReader_h_

#include "MeshManager.h"
#include "FlowManager.h"

class GAMILReader
{
public:
    GAMILReader();
    virtual ~GAMILReader();

    void init(const string &dir, const string &filePattern);

    void getVelocityField();

#ifdef DEBUG
    void checkVelocityField();
#endif

    MeshManager meshManager;
    FlowManager flowManager;

private:
    vector<string> fileNames;
};

#endif
