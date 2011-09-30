#ifndef guard_h
#define guard_h

#include "CurvatureGuard.h"
#include "TTS.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "TimeManager.h"
#include "ApproachDetector.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

void CurvatureGuard::guard(MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager)
{
    bool flag = false;
    
    // -------------------------------------------------------------------------
    // some operations at the first step
    if (TimeManager::isFirstStep()) {
        // check the location of each edge's test point
        Edge *edge = polygonManager.edges.front();
        for (int i = 0; i < polygonManager.edges.size(); ++i) {
            Vertex *testPoint = edge->getTestPoint();
            Location loc;
            meshManager.checkLocation(testPoint->getCoordinate(), loc);
            testPoint->setLocation(loc);
            edge = edge->next;
        }
    }
    
    // -------------------------------------------------------------------------
    // advect test points
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        TTS::track(meshManager, flowManager, edge->getTestPoint());
        edge = edge->next;
    }
#ifdef DEBUG
    DebugTools::dump_watchers();
#endif

    // -------------------------------------------------------------------------
    if (splitEdge(meshManager, flowManager, polygonManager)) flag = true;
#ifdef DEBUG
    DebugTools::dump_watchers();
#endif

    // -------------------------------------------------------------------------
    if (mergeEdge(meshManager, flowManager, polygonManager)) flag = true;
#ifdef DEBUG
    DebugTools::dump_watchers();
#endif

    // -------------------------------------------------------------------------
    ApproachDetector::detect(meshManager, flowManager, polygonManager);

    // -------------------------------------------------------------------------
    if (splitPolygon(meshManager, flowManager, polygonManager)) flag = true;
#ifdef DEBUG
    DebugTools::dump_watchers();
#endif

    // -------------------------------------------------------------------------
    ApproachDetector::reset(polygonManager);

    // -------------------------------------------------------------------------
    // reindex the vertices and edges for outputting
    if (flag) {
        polygonManager.vertices.reindex();
        polygonManager.edges.reindex();
    }
}

#endif
