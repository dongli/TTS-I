#include "CurvatureGuard.hpp"
#include "MeshManager.hpp"
#include "FlowManager.hpp"
#include "PolygonManager.hpp"
#include "TimeManager.hpp"
#include "ApproachDetector.hpp"
#include "TTS.hpp"
#include "CommonTasks.hpp"
#ifdef DEBUG
#include "DebugTools.hpp"
#endif

#include "AngleThreshold.hpp"
#include "splitEdge.hpp"
#include "mergeEdge.hpp"
#include "splitPolygon.hpp"

#include <ctime>

void CurvatureGuard::init()
{
    AngleThreshold::init();
}

void CurvatureGuard::guard(MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager)
{
    bool flag = false;
    clock_t start, end;
    // -------------------------------------------------------------------------
    // advect test points
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        TTS::track(meshManager, flowManager, edge->getTestPoint());
        edge = edge->next;
    }
    // -------------------------------------------------------------------------
    start = clock();
    if (splitEdges(meshManager, flowManager, polygonManager)) flag = true;
    // -------------------------------------------------------------------------
    end = clock();
    cout << "[Timing]: CurvatureGuard::splitEdges: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl;
    start = clock();
    if (mergeEdges(meshManager, flowManager, polygonManager)) flag = true;
    end = clock();
    cout << "[Timing]: CurvatureGuard::mergeEdges: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl;
    // -------------------------------------------------------------------------
    start = clock();
    ApproachDetector::detectPolygons(meshManager, flowManager, polygonManager);
    end = clock();
    cout << "[Timing]: CurvatureGuard::detectPolygons: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl;
#ifdef TTS_CGA_SPLIT_POLYGONS
    // -------------------------------------------------------------------------
    start = clock();
    if (splitPolygons(meshManager, flowManager, polygonManager)) flag = true;
    end = clock();
    cout << "[Timing]: CurvatureGuard::splitPolygons: ";
    cout << setprecision(5) << (double)(end-start)/CLOCKS_PER_SEC << " seconds." << endl;
#endif
    // -------------------------------------------------------------------------
    ApproachDetector::reset(polygonManager);
    CommonTasks::resetTasks();
#ifdef TTS_OUTPUT
    // -------------------------------------------------------------------------
    // reindex the vertices and edges for outputting
    if (flag) {
        polygonManager.vertices.reindex();
        polygonManager.edges.reindex();
    }
#endif
}