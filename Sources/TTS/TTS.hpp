#ifndef TTS_h
#define TTS_h

#include "MeshManager.hpp"
#include "MeshAdaptor.hpp"
#include "FlowManager.hpp"
#include "TracerManager.hpp"

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
