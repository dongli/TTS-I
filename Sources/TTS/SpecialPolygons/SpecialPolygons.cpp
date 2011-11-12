#include "SpecialPolygons.h"
#include "PolygonManager.h"
#include "TTS.h"

using namespace SpecialPolygons;

void SpecialPolygons::handleLinePolygon(PolygonManager &polygonManager,
                                        Polygon *polygon)
{
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2 = polygon->edgePointers.back();
    EdgePointer *edgePointer3, *edgePointer4;
    Edge *edge1 = edgePointer1->edge;
    Edge *edge2 = edgePointer2->edge;
    polygonManager.polygons.remove(polygon);
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
    if (edgePointer1->orient == OrientLeft) {
        edgePointer3 = edge1->getEdgePointer(OrientRight);
    } else {
        edgePointer3 = edge1->getEdgePointer(OrientLeft);
    }
    if (edgePointer2->orient == OrientLeft) {
        edgePointer4 = edge2->getEdgePointer(OrientRight);
    } else {
        edgePointer4 = edge2->getEdgePointer(OrientLeft);
    }
    Polygon *polygon1 = edge1->getPolygon(edgePointer3->orient);
    Polygon *polygon2 = edge2->getPolygon(edgePointer4->orient);
    if (polygon1 == polygon2) {
        TTS::deleteTask(TTS::UpdateAngle, edgePointer1);
        TTS::deleteTask(TTS::UpdateAngle, edgePointer2);
        TTS::deleteTask(TTS::UpdateAngle, edgePointer3);
        TTS::deleteTask(TTS::UpdateAngle, edgePointer4);
#ifdef DEBUG
        assert(edgePointer3->next == edgePointer4 ||
               edgePointer4->next == edgePointer3);
#endif
        Vertex *vertex;
        if (edgePointer3->next == edgePointer4) {
            vertex = edgePointer4->getEndPoint(FirstPoint);
            TTS::recordTask(TTS::UpdateAngle, edgePointer4->next);
        } else if (edgePointer4->next == edgePointer3) {
            vertex = edgePointer3->getEndPoint(FirstPoint);
            TTS::recordTask(TTS::UpdateAngle, edgePointer3->next);
        } else
            REPORT_ERROR("Unhandled branches!");
        polygon1->edgePointers.remove(edgePointer3);
        polygon1->edgePointers.remove(edgePointer4);
        polygonManager.edges.remove(edge1);
        polygonManager.edges.remove(edge2);
        polygonManager.vertices.remove(vertex);
    } else {
        edge1->setPolygon(edgePointer1->orient, polygon2);
        edge1->setEdgePointer(edgePointer1->orient, edgePointer4);
        edge2->detectAgent.handoverVertices(edge1);
        TTS::deleteTask(TTS::UpdateAngle, edgePointer1);
        TTS::deleteTask(TTS::UpdateAngle, edgePointer2);
        polygonManager.edges.remove(edge2);
    }
}

void SpecialPolygons::handlePointPolygon(PolygonManager &polygonManager,
                                         Polygon *polygon)
{
#ifdef DEBUG
    assert(polygon->edgePointers.size() == 1);
#endif
    EdgePointer *edgePointer1 = polygon->edgePointers.front();
    EdgePointer *edgePointer2;
    Polygon *polygon2;
    if (edgePointer1->orient == OrientLeft) {
        edgePointer2 = edgePointer1->edge->getEdgePointer(OrientRight);
        polygon2 = edgePointer1->edge->getPolygon(OrientRight);
    } else {
        edgePointer2 = edgePointer1->edge->getEdgePointer(OrientLeft);
        polygon2 = edgePointer1->edge->getPolygon(OrientLeft);
    }
    polygon2->edgePointers.remove(edgePointer2);
#ifdef DEBUG
    assert(edgePointer1->edge->getEndPoint(FirstPoint) ==
           edgePointer1->edge->getEndPoint(SecondPoint));
#endif
    polygonManager.polygons.remove(polygon);
    polygonManager.edges.remove(edgePointer1->edge);
    TTS::deleteTask(TTS::UpdateAngle, edgePointer1);
    TTS::deleteTask(TTS::UpdateAngle, edgePointer2);
}

void SpecialPolygons::handleEnclosedPolygons(PolygonManager &polygonManager,
                                             Polygon *polygon1,
                                             EdgePointer *edgePointer11,
                                             EdgePointer *edgePointer12,
                                             Polygon *polygon)
{
    EdgePointer *edgePointer2;
    EdgePointer *edgePointer;
    Vertex *vertex;
    Polygon *polygon2;
    if (polygon == NULL) {
        edgePointer2 = edgePointer12->getNeighborEdgePointer();
        polygon2 = edgePointer2->getPolygon(OrientLeft);
        // ---------------------------------------------------------------------
        Polygon *polygon3;
        edgePointer = edgePointer2;
        do {
            polygon3 = edgePointer->getPolygon(OrientRight);
            if (polygon3 != polygon1)
                handleEnclosedPolygons(polygonManager, polygon1,
                                       edgePointer, NULL, polygon2);
            if (edgePointer->next != edgePointer2) {
                vertex = edgePointer->getEndPoint(SecondPoint);
                polygonManager.vertices.remove(vertex);
            }
            polygonManager.edges.remove(edgePointer->edge);
            edgePointer = edgePointer->next;
        } while (edgePointer != edgePointer2);
        // ---------------------------------------------------------------------
        EdgePointer *edgePointer13 = edgePointer12->next;
        edgePointer = edgePointer11;
        while (true) {
            polygon1->edgePointers.remove(edgePointer);
            if (edgePointer == edgePointer12) break;
            edgePointer = edgePointer->next;
        }
        edgePointer13->calcAngle();
    } else {
        edgePointer2 = edgePointer11->getNeighborEdgePointer();
        polygon2 = edgePointer2->getPolygon(OrientLeft);
        edgePointer = edgePointer2->next;
        do {
            if (edgePointer->next != edgePointer2) {
                vertex = edgePointer->getEndPoint(SecondPoint);
#ifdef DEBUG
                if (vertex->linkedEdges.size() != 2)
                    REPORT_ERROR("Unhandled case!");
#endif
                polygonManager.vertices.remove(vertex);
            }
            polygonManager.edges.remove(edgePointer->edge);
            edgePointer = edgePointer->next;
        } while (edgePointer != edgePointer2);
    }
    polygonManager.polygons.remove(polygon2);
}