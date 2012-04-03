#ifndef TTS_h
#define TTS_h

#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "TracerManager.h"

class TTS
{
public:
    TTS();
    virtual ~TTS();

    void init();

    void advect(MeshManager &meshManager,
                MeshAdaptor &meshAdaptor,
                const FlowManager &flowManager,
                TracerManager &tracerManager);

    static void track(MeshManager &, const FlowManager &, Point *);
};

#endif
