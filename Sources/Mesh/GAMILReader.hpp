#ifndef GAMILReader_h
#define GAMILReader_h

#include "MeshManager.hpp"
#include "MeshAdaptor.hpp"
#include "FlowManager.hpp"
#include "TracerManager.hpp"

class GAMILReader
{
public:
    GAMILReader();
    virtual ~GAMILReader();

    void init();
    void getTracerField(TracerManager &);
    void getVelocityField();

#ifdef DEBUG
    void checkVelocityField();
#endif

    MeshManager meshManager;
    MeshAdaptor meshAdaptor;
    FlowManager flowManager;

private:
    string dataRoot;
    vector<string> fileNames;
};

#endif
