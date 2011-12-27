#ifndef splitEdge_h
#define splitEdge_h

#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "PotentialCrossDetector.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#include "CommonTasks.h"
#ifdef DEBUG_TTS
#include "DebugTools.h"
#endif

using namespace PotentialCrossDetector;

bool CurvatureGuard::splitEdge(MeshManager &meshManager,
                               const FlowManager &flowManager,
                               PolygonManager &polygonManager,
                               Edge *edge, bool isChecked, bool isMustSplit)
{
    Polygon *polygon1;
    Polygon *polygon2;
    EdgePointer *edgePointer1;
    EdgePointer *edgePointer2;
    EdgePointer *newEdgePointer1, *newEdgePointer2, *newEdgePointer;
    bool isUpdateAngles;
    Location loc;
    double a0;
    Edge *newEdge;

    if (edge->tags.isSet(SplitChecked))
        return false;

    // TEST: Reset the test point of small edges.
    if (edge->getLength() < 0.01/Rad2Deg) {
        edge->getTestPoint()->reset(meshManager);
//        REPORT_DEBUG;
    }

    Vertex *vertex1 = edge->getEndPoint(FirstPoint);
    TestPoint *testPoint = edge->getTestPoint();
    Vertex *vertex2 = edge->getEndPoint(SecondPoint);

//    if (TimeManager::getSteps() >= 95 && edge->getID() == 305357) {
//        edge->getPolygon(OrientLeft)->dump("polygon1");
//        edge->getPolygon(OrientRight)->dump("polygon2");
//        REPORT_DEBUG;
//    }

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
        // add test point into the main data structure
        Vertex *newVertex;
        polygonManager.vertices.append(&newVertex);
        *newVertex = *testPoint;
        // count the new vertex
        meshManager.checkLocation(newVertex->getCoordinate(), loc, newVertex);
        newVertex->setLocation(loc);
        // ---------------------------------------------------------------------
        Edge edge1, edge2; // temporary edges
        // ---------------------------------------------------------------------
        // link end points
        edge1.linkEndPoint(FirstPoint, vertex1);
        edge1.linkEndPoint(SecondPoint, newVertex);
        edge2.linkEndPoint(FirstPoint, newVertex);
        edge2.linkEndPoint(SecondPoint, vertex2);
        // ---------------------------------------------------------------------
        // advect new test points
        Vertex *testPoint1 = edge1.getTestPoint();
        Vertex *testPoint2 = edge2.getTestPoint();
        meshManager.checkLocation(testPoint1->getCoordinate(), loc);
        testPoint1->setLocation(loc);
        meshManager.checkLocation(testPoint2->getCoordinate(), loc);
        testPoint2->setLocation(loc);
        TTS::track(meshManager, flowManager, testPoint1);
        TTS::track(meshManager, flowManager, testPoint2);
        // ---------------------------------------------------------------------
        // edge1
        // the edge has not been splitted, add the edge into the main
        // data structure
        polygonManager.edges.append(&newEdge);
        *newEdge = edge1;
        vertex1->linkEdge(newEdge);
        newVertex->linkEdge(newEdge);
        newEdge->calcNormVector();
        newEdge->calcLength();
        // left polygon:
        if (polygon1 != NULL) {
            polygon1->edgePointers.insert(edgePointer1, &newEdgePointer);
            polygon1->edgePointers.remove(edgePointer1);
            edgePointer1 = NULL;
            newEdge->setPolygon(OrientLeft, polygon1);
            newEdge->setEdgePointer(OrientLeft, newEdgePointer);
            newEdgePointer1 = newEdgePointer;
        }
        // right polygon:
        if (polygon2 != NULL) {
            polygon2->edgePointers.insert(&newEdgePointer, edgePointer2);
            polygon2->edgePointers.remove(edgePointer2);
            edgePointer2 = NULL;
            newEdge->setPolygon(OrientRight, polygon2);
            newEdge->setEdgePointer(OrientRight, newEdgePointer);
            newEdgePointer2 = newEdgePointer;
        }
        edge->detectAgent.handoverVertices(newEdge);
        // ---------------------------------------------------------------------
        // edge2
        // the edge has not been splitted, add the edge into the main
        // data structure
        polygonManager.edges.append(&newEdge);
        *newEdge = edge2;
        vertex2->linkEdge(newEdge);
        newVertex->linkEdge(newEdge);
        newEdge->calcNormVector();
        newEdge->calcLength();
        // left polygon:
        if (polygon1 != NULL) {
            polygon1->edgePointers.insert(newEdgePointer1, &newEdgePointer);
            newEdge->setPolygon(OrientLeft, polygon1);
            newEdge->setEdgePointer(OrientLeft, newEdgePointer);
        }
        // right polygon:
        if (polygon2 != NULL) {
            polygon2->edgePointers.insert(&newEdgePointer, newEdgePointer2);
            newEdge->setPolygon(OrientRight, polygon2);
            newEdge->setEdgePointer(OrientRight, newEdgePointer);
        }
        edge->detectAgent.handoverVertices(newEdge);
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