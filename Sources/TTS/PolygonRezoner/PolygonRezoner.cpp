#include "PolygonRezoner.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "ApproachDetector.h"
#include "PotentialCrossDetector.h"
#include "TTS.h"
#include "CommonTasks.h"
#include "DelaunayDriver.h"
#include "PointManager.h"
#include "process_cocircularDT.h"
#include "TimeManager.h"

using namespace ApproachDetector;
using namespace PotentialCrossDetector;

void handleBentPolygons(MeshManager &meshManager,
                        const FlowManager &flowManager,
                        PolygonManager &polygonManager)
{
    static const double a0 = 200.0/Rad2Deg;
    std::list<Projection>::const_iterator itPrj;
    std::list<Vertex *>::const_iterator itVtx;
    Vertex *vertex1;
    Edge *markEdge, *newEdge;
    EdgePointer *edgePointer;
    EdgePointer *markEdgePointer1, *markEdgePointer2;
    EdgePointer *newEdgePointer1, *newEdgePointer2;
    const Projection *projection;
    Vector vector1, vector2;
    double distance, angle1, angle2, angle3, angle4;
    bool isConnectOk1, isConnectOk2;
    int mode;
    Location loc;
    // -------------------------------------------------------------------------
    Polygon *newPolygon;
    Polygon *polygon = polygonManager.polygons.front();
    while (polygon != NULL) {
    rehandle_polygon:
        markEdgePointer1 = polygon->edgePointers.front();
        do {
            markEdgePointer2 = NULL; distance = 1.0e33;
            // -----------------------------------------------------------------
            // find out the vertex that is penetrating into polygon
            if (markEdgePointer1->getAngle() > a0) {
                vertex1 = markEdgePointer1->getEndPoint(FirstPoint);
                for (itPrj = vertex1->detectAgent.getProjections().begin();
                     itPrj != vertex1->detectAgent.getProjections().end(); ++itPrj) {
                    projection = &*itPrj;
                    // Note: Find the most close edge to split the polygon
                    if (distance < projection->getDistance(NewTimeLevel))
                        continue;
                    markEdge = projection->getEdge();
                    if (markEdge != markEdgePointer1->prev->prev->edge &&
                        markEdge != markEdgePointer1->next->edge) {
                        if (markEdge->getPolygon(OrientLeft) == polygon) {
                            markEdgePointer2 = markEdge->getEdgePointer(OrientLeft);
                            distance = projection->getDistance(NewTimeLevel);
                        } else if (markEdge->getPolygon(OrientRight) == polygon) {
                            markEdgePointer2 = markEdge->getEdgePointer(OrientRight);
                            distance = projection->getDistance(NewTimeLevel);
                        } else
                            continue;
                        if (projection->getOrient() != markEdgePointer2->orient) {
                            markEdgePointer2 = NULL; distance = 1.0e33;
                            continue;
                        }
                        ;
                        if (detectAddConnection(polygon, markEdgePointer1,
                                                markEdgePointer2, isConnectOk1,
                                                isConnectOk2, vector1,
                                                vector2) == Cross) {
                            markEdgePointer2 = NULL; distance = 1.0e33;
                        }
                    }
                }
            }
            // -----------------------------------------------------------------
            if (markEdgePointer2 != NULL) {
                // choose which vertex to connect
                if (isConnectOk1 && isConnectOk2) {
                    angle1 = EdgePointer::calcAngle
                    (markEdgePointer1->prev->getNormVector(), -vector1, *vertex1);
                    angle2 = EdgePointer::calcAngle
                    (vector1, markEdgePointer1->getNormVector(), *vertex1);
                    angle3 = EdgePointer::calcAngle
                    (markEdgePointer1->prev->getNormVector(), -vector2, *vertex1);
                    angle4 = EdgePointer::calcAngle
                    (vector2, markEdgePointer1->getNormVector(), *vertex1);
#ifndef DEBUG
                    assert(fabs(angle1+angle2-angle3-angle4) < EPS);
                    assert(fabs(angle1+angle2-markEdgePointer1->getAngle()) < EPS);
#endif
                    if (fmax(angle1, angle2) < fmax(angle3, angle4)) {
                        mode = 1;
                    } else {
                        mode = 2;
                    }
                } else {
                    if (isConnectOk1)
                        mode = 1;
                    if (isConnectOk2)
                        mode = 2;
                }
                // create new polygon
                polygonManager.polygons.append(&newPolygon);
                if (mode == 2)
                    markEdgePointer2 = markEdgePointer2->next;
                edgePointer = markEdgePointer1;
                while (edgePointer != markEdgePointer2) {
                    newPolygon->edgePointers.append(&newEdgePointer2);
                    newEdgePointer2->replace(edgePointer);
                    newEdgePointer2->edge->setPolygon(newEdgePointer2->orient,
                                                      newPolygon);
                    edgePointer = edgePointer->next;
                    polygon->edgePointers.remove(edgePointer->prev);
                }
                // create new edge
                polygonManager.edges.append(&newEdge);
                newEdge->linkEndPoint(FirstPoint, vertex1);
                newEdge->linkEndPoint(SecondPoint,
                                      markEdgePointer2->getEndPoint(FirstPoint));
                newEdge->calcNormVector();
                newEdge->calcLength();
                Vertex *testPoint = newEdge->getTestPoint();
                meshManager.checkLocation(testPoint->getCoordinate(), loc);
                testPoint->setLocation(loc);
                TTS::track(meshManager, flowManager, testPoint);
                newEdge->setPolygon(OrientLeft, polygon);
                newEdge->setPolygon(OrientRight, newPolygon);
                polygon->edgePointers.insert(&newEdgePointer1, markEdgePointer2);
                newPolygon->edgePointers.append(&newEdgePointer2);
                newPolygon->edgePointers.ring();
                newEdge->setEdgePointer(OrientLeft, newEdgePointer1);
                newEdge->setEdgePointer(OrientRight, newEdgePointer2);
                // update angles
                CommonTasks::doTask(CommonTasks::UpdateAngle);
                // hand over tracer mass
                polygon->calcArea();
                newPolygon->calcArea();
                double percent = newPolygon->getArea()/(polygon->getArea()+newPolygon->getArea());
                polygon->handoverTracers(newPolygon, percent);
                // detect the two polygons
                detectPolygon(meshManager, flowManager, polygonManager, polygon);
                detectPolygon(meshManager, flowManager, polygonManager, newPolygon);
#ifndef DEBUG
                cout << "Polygon " << polygon->getID() << " is split with new ";
                cout << "polygon " << newPolygon->getID() << endl;
#endif
                goto rehandle_polygon;
            } else
                markEdgePointer1 = markEdgePointer1->next;
        } while (markEdgePointer1 != polygon->edgePointers.back()->next);
        polygon = polygon->next;
    }
}

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
    // split the bent polygons to ensure the centroids within the polygons
    handleBentPolygons(meshManager, flowManager, polygonManager);
    // -------------------------------------------------------------------------
    // extract the centroids of each polygon for later Voronoi diagram
    int numPoint = polygonManager.polygons.size();
    double *lon = new double[numPoint];
    double *lat = new double[numPoint];
    Polygon *polygon = polygonManager.polygons.front();
    for (int i = 0; i < polygonManager.polygons.size(); ++i) {
        lon[i] = 0.0;
        lat[i] = 0.0;
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int j = 0; j < polygon->edgePointers.size(); ++j) {
            lon[i] += edgePointer->getEndPoint(FirstPoint)->getCoordinate().getLon();
            lat[i] += edgePointer->getEndPoint(FirstPoint)->getCoordinate().getLat();
            edgePointer = edgePointer->next;
        }
        lon[i] /= polygon->edgePointers.size();
        lat[i] /= polygon->edgePointers.size();
        polygon = polygon->next;
    }
    // -------------------------------------------------------------------------
    PointManager pointManager;
    DelaunayDriver driver;
    pointManager.init(numPoint, lon, lat);
    driver.linkPoint(pointManager);
    driver.init();
    driver.calcircum();
    process_cocircularDT(driver);
    // -------------------------------------------------------------------------
    // clear the polygon manager for new polygons
    polygonManager.reinit();
    polygonManager.init(driver);
    Vertex *vertex = polygonManager.vertices.front();
    for (int i = 0; i < polygonManager.vertices.size(); ++i) {
        Location loc;
        meshManager.checkLocation(vertex->getCoordinate(), loc);
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