#ifndef splitEdge_h
#define splitEdge_h

#include "MeshManager.hpp"
#include "FlowManager.hpp"
#include "PolygonManager.hpp"
#include "PotentialCrossDetector.hpp"
#include "CurvatureGuard.hpp"
#include "TTS.hpp"
#include "CommonTasks.hpp"
#ifdef DEBUG
#include "DebugTools.hpp"
#endif

using namespace PotentialCrossDetector;

bool CurvatureGuard::splitEdge(MeshManager &meshManager,
                               const FlowManager &flowManager,
                               PolygonManager &polygonManager,
                               Edge *edge, bool isChecked, bool isMustSplit)
{
    Vertex *vertex1, *vertex2;    // two end points of the old edge
    Vertex *newVertex;
    TestPoint *testPoint;
    Polygon *polygon1, *polygon2; // two incident polygons of the old edge
    Edge *newEdge1, *newEdge2;
    EdgePointer *edgePointer1;
    EdgePointer *edgePointer2;
    EdgePointer *newEdgePointer1, *newEdgePointer2;
    bool isUpdateAngles;
    Location loc;
    double a0;

#ifdef DEBUG
    assert(edge->getLength() != 0.0);
#endif

    if (edge->tags.isSet(SplitChecked))
        return false;

    vertex1 = edge->getEndPoint(FirstPoint);
    testPoint = edge->getTestPoint();
    vertex2 = edge->getEndPoint(SecondPoint);

    if (!isMustSplit) {
        testPoint->calcAngle();
        testPoint->calcOrient();
        CurvatureGuard::AngleThreshold::calc(edge, a0);
    }

    if (PI-testPoint->getAngle() > a0 || isMustSplit) {
        // ---------------------------------------------------------------------
        // record the polygons and edge pointers, and reset the tasks
        polygon1 = edge->getPolygon(OrientLeft);
        polygon2 = edge->getPolygon(OrientRight);
        edgePointer1 = edge->getEdgePointer(OrientLeft);
        edgePointer2 = edge->getEdgePointer(OrientRight);
        // Note: We only reset TTS tasks when NOT forcely split the edge.
        if (CommonTasks::getTaskNumber(CommonTasks::UpdateAngle) != 0)
            isUpdateAngles = false;
        else
            isUpdateAngles = true;
        // ---------------------------------------------------------------------
        // several checks if the splitting is ok
        if (!isChecked)
            if (detectInsertVertexOnEdge
                (meshManager, flowManager, polygonManager,
                 edge, edge->getTestPoint()) != NoCross) {
                // TEST: Reset the test point when it cross any other edge.
                testPoint->reset(meshManager);
                goto return_no_split;
            }
        // ---------------------------------------------------------------------
        // add the new vertex
        polygonManager.vertices.append(&newVertex);
        *newVertex = *testPoint; // test point becomes real vertex
        // count the new vertex
        meshManager.checkLocation(newVertex->getCoordinate(), loc, newVertex);
        newVertex->setLocation(loc);
        // ---------------------------------------------------------------------
        // add the first new edge
        polygonManager.edges.append(&newEdge1);
        newEdge1->linkEndPoint(FirstPoint, vertex1);
        newEdge1->linkEndPoint(SecondPoint, newVertex);
        newEdge1->calcNormVector();
        newEdge1->calcLength();
        if (polygon1 != NULL) {
            newEdge1->setPolygon(OrientLeft, polygon1);
            newEdge1->setEdgePointer(OrientLeft, edgePointer1);
        }
        if (polygon2 != NULL) {
            newEdge1->setPolygon(OrientRight, polygon2);
            newEdge1->setEdgePointer(OrientRight, edgePointer2);
        }
        edge->detectAgent.handoverVertices(newEdge1);
        testPoint = newEdge1->getTestPoint();
        meshManager.checkLocation(testPoint->getCoordinate(), loc);
        testPoint->setLocation(loc);
        TTS::track(meshManager, flowManager, testPoint);
        // ---------------------------------------------------------------------
        // add the second new edge
        polygonManager.edges.append(&newEdge2);
        newEdge2->linkEndPoint(FirstPoint, newVertex);
        newEdge2->linkEndPoint(SecondPoint, vertex2);
        newEdge2->calcNormVector();
        newEdge2->calcLength();
        if (polygon1 != NULL) {
            polygon1->edgePointers.insert(edgePointer1, &newEdgePointer1);
            newEdge2->setPolygon(OrientLeft, polygon1);
            newEdge2->setEdgePointer(OrientLeft, newEdgePointer1);
        }
        if (polygon2 != NULL) {
            polygon2->edgePointers.insert(&newEdgePointer2, edgePointer2);
            newEdge2->setPolygon(OrientRight, polygon2);
            newEdge2->setEdgePointer(OrientRight, newEdgePointer2);
        }
        edge->detectAgent.handoverVertices(newEdge2);
        testPoint = newEdge2->getTestPoint();
        meshManager.checkLocation(testPoint->getCoordinate(), loc);
        testPoint->setLocation(loc);
        TTS::track(meshManager, flowManager, testPoint);
        // ---------------------------------------------------------------------
        polygonManager.edges.remove(edge);
        if (isUpdateAngles)
            CommonTasks::doTask(CommonTasks::UpdateAngle);
        return true;
    }
return_no_split:
    edge->tags.set(SplitChecked);
    return false;
}

bool CurvatureGuard::splitEdges(MeshManager &meshManager,
                                const FlowManager &flowManager,
                                PolygonManager &polygonManager)
{
    bool isSplit = false;
    Edge *edge = polygonManager.edges.front(), *nextEdge;
    while (edge != NULL) {
        nextEdge = edge->next;
        if (splitEdge(meshManager, flowManager, polygonManager, edge))
            isSplit = true;
        edge = nextEdge;
    }
    return isSplit;
}

#endif