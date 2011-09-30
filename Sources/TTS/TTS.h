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

    static void track(MeshManager &, const FlowManager &, Point *,
                      bool isCount = false);

    enum TaskType {
        UpdateAngle
    };
    static void resetTasks();
    static void recordTask(TaskType, EdgePointer *);
    static void deleteTask(TaskType, EdgePointer *);
    static void doTask(TaskType, bool debug = false);
    static void dumpTask(TaskType);

private:
    static std::list<EdgePointer *> needUpdateAngles;
};

#endif