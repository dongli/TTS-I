#include "PolygonRezoner.h"
#include "MeshManager.h"
#include "MeshAdaptor.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "DelaunayDriver.h"
#include "SCVT.h"
#include "TimeManager.h"
#include <netcdfcpp.h>

void PolygonRezoner::rezone(MeshManager &meshManager,
                            const MeshAdaptor &meshAdaptor,
                            const FlowManager &flowManager,
                            PolygonManager &polygonManager)
{
    const RLLMesh &meshCnt = meshManager.getMesh(PointCounter::Center);
    const RLLMesh &meshBnd = meshManager.getMesh(PointCounter::Bound);
    // -------------------------------------------------------------------------
    // 0. Initialize SCVT
    static bool isFirstCall = true;
    if (isFirstCall) {
        SCVT::init(meshBnd.getNumLon(), meshBnd.getNumLat(),
                   meshBnd.lon.data(),  meshBnd.lat.data());
        isFirstCall = false;
    }
    // -------------------------------------------------------------------------
    // 1. Generate density function
    // =========================================================================
    // 1.1. Use tracer density difference as a guide of density function setting
    double maxDiff = 0.0;
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        polygon->tracerDiff = 0.0;
        Polygon *prevPolygon = NULL;
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            if (prevPolygon != edgePointer->getPolygon(OrientRight)) {
                prevPolygon = edgePointer->getPolygon(OrientRight);
                double diff = fabs(polygon->tracers[1].getDensity()-
                                   prevPolygon->tracers[1].getDensity());
                polygon->tracerDiff = fmax(diff, polygon->tracerDiff);
                maxDiff = fmax(diff, maxDiff);
            }
            edgePointer = edgePointer->next;
        }
        polygon = polygon->next;
    }
    Array<double, 2> &rho = SCVT::getDensityFunction();
    const Array<list<OverlapArea>, 3> &overlapAreaList = meshAdaptor.getOverlapAreaList();
    rho = 0.0;
    for (int i = 0; i < overlapAreaList.extent(0); ++i)
        for (int j = 0; j < overlapAreaList.extent(1); ++j) {
            list<OverlapArea>::const_iterator it;
            for (it = overlapAreaList(i, j, 0).begin();
                 it != overlapAreaList(i, j, 0).end(); ++it) {
                rho(i, j) = fmax(rho(i, j), (*it).polygon->tracerDiff)/maxDiff;
            }
            rho(i, j) = fmin(1.0, fmax(0.1, rho(i, j)));
        }
    // =========================================================================
    // 1.2. Smooth the density function
    Array<double, 2> smoothedRho(rho.shape());
    double p = 0.5, q = 0.25;
    for (int i = 0; i < rho.extent(0); ++i) {
        for (int j = 1; j <= rho.extent(1)-2; ++j) {
            int im1 = i != 0 ? i-1 : rho.extent(0)-1;
            int ip1 = i != rho.extent(0)-1 ? i+1 : 0;
            int jm1 = j-1, jp1 = j+1;
            smoothedRho(i, j) = (1-p-q)*rho(i, j)+
            p*0.25*(rho(im1, j)+rho(i, jm1)+rho(ip1, j)+rho(i, jp1))+
            q*0.25*(rho(im1, jm1)+rho(im1, jp1)+rho(ip1, jp1)+rho(ip1, jm1));
        }
        // north pole
        
        // south pole
    }
    rho = smoothedRho;
    cout << min(rho) << endl;
    SCVT::outputDensityFunction("scvt_rho.nc");
    // -------------------------------------------------------------------------
    // 2. Generate SCVT according to the previous density function
    int numPoint = 3000;
    DelaunayDriver driver;
    SCVT::run(numPoint, driver);
    exit(0);
}
