#include "PolygonRezoner.hpp"
#include "MeshManager.hpp"
#include "MeshAdaptor.hpp"
#include "FlowManager.hpp"
#include "TracerManager.hpp"
#include "DelaunayDriver.hpp"
#include "SCVT.hpp"
#include "TimeManager.hpp"
#include "ApproachDetector.hpp"
#include "CommonTasks.hpp"
#include "ConfigTools.hpp"
#include "CppHelper.hpp"
#include <netcdfcpp.h>

namespace PolygonRezoner {
    // running controls
    int frequency;
    // SCVT controls
    int numGenerator;
    int maxIteration;
    double minRho;
    //
    DelaunayDriver driver;
}

void PolygonRezoner::init()
{
    ConfigTools::read("rezone_num_generator", numGenerator);
    ConfigTools::read("rezone_frequency", frequency);
    ConfigTools::read("rezone_min_rho", minRho);
    ConfigTools::read("rezone_max_iteration", maxIteration);
    TimeManager::setAlarm("polygon rezoning", frequency);
}

void PolygonRezoner::rezone(MeshManager &meshManager,
                            MeshAdaptor &meshAdaptor,
                            const FlowManager &flowManager,
                            TracerManager &tracerManager)
{
    string ID = to_string(TimeManager::getSteps(), "%5.5d");
    PolygonManager &polygonManager = tracerManager.polygonManager;
    // -------------------------------------------------------------------------
    // 0. Initialize SCVT
    static bool isFirstCall = true;
    if (isFirstCall) {
        const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
        SCVT::init(meshBnd.getNumLon(), meshBnd.getNumLat(),
                   meshBnd.lon.data(),  meshBnd.lat.data(),
                   maxIteration);
        isFirstCall = false;
    }
    // -------------------------------------------------------------------------
    // 1. Generate density function
    // =========================================================================
    // 1.1. Use tracer density difference as a guide of density function setting
    int idx = tracerManager.getTracerId("test tracer 0");
    Array<double, 2> &rho = SCVT::getDensityFunction();
    rho = 0.0;
    const Field &q = tracerManager.getTracerDensityField(idx);
    const RLLMesh &mesh = q.getMesh();
    for (int i = 1; i < mesh.getNumLon()-1; ++i)
        for (int j = 1; j < mesh.getNumLat()-1; ++j) {
            int k = i-1;
            rho(k, j) =                 fabs(q.values(i, j).getNew()-q.values(i-1, j-1).getNew());
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i-1, j  ).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i-1, j+1).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i,   j-1).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i,   j+1).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i+1, j-1).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i+1, j  ).getNew()));
            rho(k, j) = fmax(rho(k, j), fabs(q.values(i, j).getNew()-q.values(i+1, j+1).getNew()));
        }
    rho /= max(rho);
    rho = where(rho < minRho, minRho, rho);
    SCVT::outputDensityFunction(ID);
    // -------------------------------------------------------------------------
    // 2. Generate SCVT according to the previous density function
    SCVT::run(numGenerator, driver, ID);
    // -------------------------------------------------------------------------
    // 3. Replace the polygons with SCVT
    polygonManager.reinit();
    polygonManager.init(driver);
    Vertex *vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        Location loc;
        meshManager.checkLocation(vertex->getCoordinate(), loc, vertex);
        vertex->setLocation(loc);
        vertex = vertex->next;
    }
    Edge *edge = polygonManager.edges.front();
    for (int i = 0; i < polygonManager.edges.size(); ++i) {
        Vertex *testPoint = edge->getTestPoint();
        Location loc;
        meshManager.checkLocation(testPoint->getCoordinate(), loc);
        testPoint->setLocation(loc);
        edge = edge->next;
    }
    ApproachDetector::detectPolygons(meshManager, flowManager, polygonManager);
#ifdef TTS_CGA_SPLIT_POLYGONS
    ApproachDetector::ApproachingVertices::vertices.clear();
#endif
    ApproachDetector::reset(polygonManager);
    CommonTasks::resetTasks();
    // -------------------------------------------------------------------------
#ifdef TTS_REMAP
    meshAdaptor.adapt(tracerManager, meshManager);
    for (int i = 0; i < tracerManager.getTracerNum(); ++i)
        meshAdaptor.remap(tracerManager.getTracerName(i),
                          tracerManager.getTracerDensityField(i),
                          tracerManager);
#endif
}
