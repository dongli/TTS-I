#ifndef _TTS_h_
#define _TTS_h_

#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "TracerManager.h"
#include <list>

class TTS
{
public:
    TTS();
    virtual ~TTS();

    void advect(MeshManager &meshManager,
                MeshAdaptor &meshAdaptor,
                const FlowManager &flowManager,
                TracerManager &tracerManager);

    static void track(MeshManager &, const FlowManager &, Point *);
};

#endif
