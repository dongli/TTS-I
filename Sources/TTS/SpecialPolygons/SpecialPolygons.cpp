#include "SpecialPolygons.h"
#include "PolygonManager.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "ApproachDetector.h"
#include "PotentialCrossDetector.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#include "CommonTasks.h"
#include <set>

using namespace SpecialPolygons;
using namespace ApproachDetector;
using namespace PotentialCrossDetector;
using namespace CurvatureGuard;

void SpecialPolygons::handleLinePolygon(PolygonManager &polygonManager,
                                        Polygon *polygon, Vertex *keepVertex,
                                        bool isKeepMass)
{
    // -------------------------------------------------------------------------
    // hand over tracer mass
    if (!isKeepMass)
        polygon->handoverTracers();
    // -------------------------------------------------------------------------
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2 = polygon->edgePointers.back();
    EdgePointer *edgePointer3, *edgePointer4;
    Edge *edge1 = edgePointer1->edge;
    Edge *edge2 = edgePointer2->edge;
#ifdef DEBUG
    assert(((edge1->getEndPoint(FirstPoint) ==
             edge2->getEndPoint(FirstPoint)) &&
            (edge1->getEndPoint(SecondPoint) ==
             edge2->getEndPoint(SecondPoint))) ||
           ((edge1->getEndPoint(FirstPoint) ==
             edge2->getEndPoint(SecondPoint)) &&
            (edge1->getEndPoint(SecondPoint) ==
             edge2->getEndPoint(FirstPoint))));
#endif
    edgePointer3 = edgePointer1->getNeighborEdgePointer();
    edgePointer4 = edgePointer2->getNeighborEdgePointer();
    Polygon *polygon1 = edge1->getPolygon(edgePointer3->orient);
    Polygon *polygon2 = edge2->getPolygon(edgePointer4->orient);
    if (polygon1 == polygon2) {
        CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer1);
        CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer2);
        if (edgePointer3->next == edgePointer4) {
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer3);
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer4);
            CommonTasks::recordTask(CommonTasks::UpdateAngle, edgePointer4->next); 
            polygon1->edgePointers.remove(edgePointer3);
            polygon1->edgePointers.remove(edgePointer4);
            polygonManager.vertices.remove(edgePointer4->getEndPoint(FirstPoint));
        } else if (edgePointer4->next == edgePointer3) {
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer3);
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer4); 
            CommonTasks::recordTask(CommonTasks::UpdateAngle, edgePointer3->next);
            polygon1->edgePointers.remove(edgePointer3);
            polygon1->edgePointers.remove(edgePointer4);
            polygonManager.vertices.remove(edgePointer3->getEndPoint(FirstPoint));
        } else {
            assert(keepVertex != NULL);
            EdgePointer *edgePointer5, *edgePointer6;
            Polygon *polygon5, *polygon6;
            if (edgePointer3->getEndPoint(SecondPoint) == keepVertex) {
                edgePointer5 = edgePointer3;
                edgePointer3 = edgePointer4;
                edgePointer4 = edgePointer5;
            }
            std::set<Polygon *> badPolygons;
            edgePointer3 = edgePointer3->next;
            while (edgePointer3 != edgePointer4) {
                badPolygons.insert(edgePointer3->getPolygon(OrientRight));
                edgePointer3 = edgePointer3->next;
            }
            std::set<Polygon *>::iterator it;
            for (it = badPolygons.begin(); it != badPolygons.end(); ++it) {
                polygon5 = *it;
                edgePointer5 = polygon5->edgePointers.front();
                for (int i = 0; i < polygon5->edgePointers.size(); ++i) {
                    polygon6 = edgePointer5->getPolygon(OrientRight);
                    edgePointer6 = edgePointer5->getNeighborEdgePointer();
                    polygonManager.vertices.remove(edgePointer6->getEndPoint(SecondPoint));
                    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer5);
                    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer6);
                    polygonManager.edges.remove(edgePointer6->edge);
                    polygon6->edgePointers.remove(edgePointer6);
                    edgePointer5 = edgePointer5->next;
                }
                // TODO: handle over the tracer mass
                polygonManager.polygons.remove(polygon5);
            }
            CommonTasks::recordTask(CommonTasks::UpdateAngle, edgePointer4->next);
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer4->prev);
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer4);
            polygon1->edgePointers.remove(edgePointer4->prev);
            polygon1->edgePointers.remove(edgePointer4);
        }
        polygonManager.edges.remove(edge1);
        polygonManager.edges.remove(edge2);
    } else {
        edge1->setPolygon(edgePointer1->orient, polygon2);
        edge1->setEdgePointer(edgePointer1->orient, edgePointer4);
        edge2->detectAgent.handoverVertices(edge1);
        CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer1);
        CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer2);
        polygonManager.edges.remove(edge2);
    }
    polygonManager.polygons.remove(polygon);
}

void SpecialPolygons::handlePointPolygon(PolygonManager &polygonManager,
                                         Polygon *polygon, bool isKeepMass)
{
#ifdef DEBUG
    assert(polygon->edgePointers.size() == 1);
#endif
    // -------------------------------------------------------------------------
    // hand over tracer mass
    if (!isKeepMass)
        polygon->handoverTracers();
    // -------------------------------------------------------------------------
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2;
    Polygon *polygon2;
    edgePointer2 = edgePointer1->getNeighborEdgePointer();
    polygon2 = edgePointer1->getPolygon(OrientRight);
    polygon2->edgePointers.remove(edgePointer2);
    if (polygon2->edgePointers.size() == 2)
        handleLinePolygon(polygonManager, polygon2);
#ifdef DEBUG
    assert(edgePointer1->edge->getEndPoint(FirstPoint) ==
           edgePointer1->edge->getEndPoint(SecondPoint));
#endif
    polygonManager.polygons.remove(polygon);
    polygonManager.edges.remove(edgePointer1->edge);
    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer1);
    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer2);
}

void SpecialPolygons::handleEnclosedPolygons(EdgePointer *edgePointer1,
                                             EdgePointer *edgePointer2,
                                             PolygonManager &polygonManager)
{
#ifdef DEBUG
    assert(CommonTasks::getTaskNumber(CommonTasks::RemoveObject) == 0);
#endif
    std::set<Polygon *> enclosedPolygons;
    std::set<Polygon *>::iterator it;
    Polygon *enclosingPolygon;
    Vertex *keepVertex;
    EdgePointer *edgePointer;
    // -------------------------------------------------------------------------
    enclosingPolygon = edgePointer1->next->getPolygon(OrientLeft);
    keepVertex = edgePointer1->next->getEndPoint(FirstPoint);
    // -------------------------------------------------------------------------
    // find out the enclosed polygons
    // record periphery enclosed polygon
    edgePointer = edgePointer1->next;
    while (edgePointer != edgePointer2) {
        enclosedPolygons.insert(edgePointer->getPolygon(OrientRight));
        edgePointer = edgePointer->next;
    }
    // record inner enclosed polygons
    for (it = enclosedPolygons.begin(); it != enclosedPolygons.end(); ++it) {
        edgePointer = (*it)->edgePointers.front();
        for (int i = 0; i < (*it)->edgePointers.size(); ++i) {
            if (edgePointer->getPolygon(OrientRight) != enclosingPolygon)
                enclosedPolygons.insert(edgePointer->getPolygon(OrientRight));
            edgePointer = edgePointer->next;
        }
    }
    // -------------------------------------------------------------------------
    // destroy the enclosed polygons
    for (it = enclosedPolygons.begin(); it != enclosedPolygons.end(); ++it) {
        (*it)->destroy();
        (*it)->handoverTracers();
        polygonManager.polygons.remove(*it);
    }
    CommonTasks::deleteTask(CommonTasks::RemoveObject, keepVertex);
    CommonTasks::recordTask(CommonTasks::UpdateAngle, edgePointer2);
    edgePointer = edgePointer1->next;
    while (edgePointer != edgePointer2) {
        enclosingPolygon->edgePointers.remove(edgePointer);
        CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer);
        edgePointer = edgePointer1->next;
    }
    CommonTasks::doTask(CommonTasks::RemoveObject, polygonManager);
}
