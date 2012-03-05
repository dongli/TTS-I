#include "PolygonRezoner.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "ApproachDetector.h"
#include "PotentialCrossDetector.h"
#include "SpecialPolygons.h"
#include "TTS.h"
#include "CommonTasks.h"
#include "DelaunayDriver.h"
#include "PointManager.h"
#include "TimeManager.h"

using namespace ApproachDetector;
using namespace PotentialCrossDetector;

void PolygonRezoner::rezone(MeshManager &meshManager,
                            const FlowManager &flowManager,
                            PolygonManager &polygonManager)
{
#ifdef DEBUG
    char fileName[100];
    sprintf(fileName, "before_rezone_%d.nc", TimeManager::getSteps());
    polygonManager.output(fileName);
#endif
    // -------------------------------------------------------------------------
    Polygon *polygon;
    EdgePointer *edgePointer;
    // -------------------------------------------------------------------------
    // extract the centroids of each polygon for later Voronoi diagram
    int numPoint = polygonManager.polygons.size();
    // =========================================================================
    // skip small polygons
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        double minArea = 1.0e34, maxArea = -1.0e34, avgArea = 0.0;
        edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            if (minArea > edgePointer->getPolygon(OrientRight)->getArea())
                minArea = edgePointer->getPolygon(OrientRight)->getArea();
            if (maxArea < edgePointer->getPolygon(OrientRight)->getArea())
                maxArea = edgePointer->getPolygon(OrientRight)->getArea();
            avgArea += edgePointer->getPolygon(OrientRight)->getArea();
            edgePointer = edgePointer->next;
        }
        avgArea /= polygon->edgePointers.size();
        if (polygon->getArea()/avgArea < 0.1) {
            numPoint--;
            polygon = polygon->next;
            continue;
        }
        // calculate the centroid
        polygon->calcCentroid();
        polygon = polygon->next;
    }
    // =========================================================================
    // =========================================================================
    // split large polygons
    int k = -1, maxNumPoint = numPoint+numPoint/10;
    double *lon = new double[maxNumPoint];
    double *lat = new double[maxNumPoint];
    static const double largeArea = 1.0/Rad2Deg*Sphere::radius2;
    polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        if (!polygon->getCentroid().isSet()) {
            polygon = polygon->next;
            continue;
        }
        k++;
        lon[k] = polygon->getCentroid().getLon();
        lat[k] = polygon->getCentroid().getLat();
        if (polygon->getArea() > largeArea) {
            edgePointer = polygon->edgePointers.front();
            Polygon *polygon1 = NULL;
            for (int j = 0; j < polygon->edgePointers.size(); ++j) {
                Polygon *polygon2 = edgePointer->getPolygon(OrientRight);
                if (polygon1 != polygon2 && polygon2->getCentroid().isSet()) {
                    k++;
                    numPoint++;
                    assert(numPoint <= maxNumPoint);
                    Coordinate x;
                    Sphere::calcMiddlePoint(polygon->getCentroid(),
                                            polygon2->getCentroid(), x);
                    lon[k] = x.getLon();
                    lat[k] = x.getLat();
                }
                polygon1 = polygon2;
                edgePointer = edgePointer->next;
            }
            polygon->getCentroid().reinit();
        }
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    // 
    PointManager pointManager;
    DelaunayDriver driver;
    pointManager.init(numPoint, lon, lat);
    delete [] lon;
    delete [] lat;
    driver.linkPoint(pointManager);
    driver.init();
    driver.calcCircumcenter();
    // -------------------------------------------------------------------------
    // clear the polygon manager for new polygons
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
    // -------------------------------------------------------------------------
    ApproachDetector::detectPolygons(meshManager, flowManager, polygonManager);
    ApproachingVertices::vertices.clear();
    // -------------------------------------------------------------------------
    ApproachDetector::reset(polygonManager);
    CommonTasks::resetTasks();
}