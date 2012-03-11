#ifndef _GAMILReader_h_
#define _GAMILReader_h_

#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "TracerManager.h"

class GAMILReader
{
public:
    GAMILReader();
    virtual ~GAMILReader();

    void init(const string &dir, const string &filePattern);
    void getTracerField(TracerManager &);
    void getVelocityField();

#ifdef DEBUG
    void checkVelocityField();
#endif

    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;

private:
    vector<string> fileNames;
};

#endif
