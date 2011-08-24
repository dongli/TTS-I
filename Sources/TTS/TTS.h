#ifndef _TTS_h_
#define _TTS_h_

#include "MeshManager.h"
#include "FlowManager.h"
#include "ParcelManager.h"
#include <list>

using std::list;

class TTS
{
public:
    TTS();
    virtual ~TTS();

    void init();
    void advect(MeshManager &, const FlowManager &, ParcelManager &);

    static void track(MeshManager &, const FlowManager &, Point *,
                      bool isCount = false);

    enum TaskType {
        UpdateAngle
    };
    static void resetTasks();
    static void recordTask(TaskType, EdgePointer *);
    static void deleteTask(TaskType, EdgePointer *);
    static void doTask(TaskType, bool debug = false);

private:
    double angleThreshold(Edge *edge);
    double angleThreshold(Edge *edge1, Edge *edge2);

    void guardCurvature(MeshManager &, const FlowManager &, PolygonManager &);
    bool splitEdge(MeshManager &, const FlowManager &, PolygonManager &, Edge *);
    bool mergeEdge(MeshManager &, const FlowManager &, PolygonManager &, Polygon *);
    bool handleWrongAngle(MeshManager &, const FlowManager &, PolygonManager &, Polygon *);
    bool splitPolygon(MeshManager &, const FlowManager &, PolygonManager &);

    double A0, A1, dA;
    double L0, L1, dL;

    static list<EdgePointer *> needUpdateAngles;
};

#endif
