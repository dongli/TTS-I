#ifndef mergeEdge_h
#define mergeEdge_h

#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "PotentialCrossDetector.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#include "CommonTasks.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

using namespace PotentialCrossDetector;

inline bool mergeEdge(MeshManager &meshManager, const FlowManager &flowManager,
                      PolygonManager &polygonManager, Polygon *polygon)
{
    bool isMerged = false;
    EdgePointer *edgePointer;
    Edge *edge1, *edge2;
    Vertex *vertex1, *vertex2, *vertex3;  // vertex2 is to be deleted
    Polygon *polygon1, *polygon2;
    EdgePointer *edgePointer1, *edgePointer2;
    double a0;

#ifdef DEBUG
    assert(CommonTasks::getTaskNumber(CommonTasks::UpdateAngle) == 0);
#endif

    // TODO: Handle the slim triangle!
    if (polygon->edgePointers.size() == 3)
        return isMerged;

    polygon->edgePointers.startLoop(edgePointer);
    do {
        if (edgePointer->prev->getPolygon(OrientRight) != NULL &&
            edgePointer->prev->getPolygon(OrientRight)->getID() < polygon->getID())
            goto next_edge;
        edge1 = edgePointer->prev->edge; // to be deleted if necessary
        edge2 = edgePointer->edge;
        CurvatureGuard::AngleThreshold::calc(edge1, edge2, a0);
        CurvatureGuard::AngleThreshold::relax(edge1, edge2, a0);
        adjustMergeEdgeAngleThreshold(edge1, edge2, a0);
        if (fabs(edgePointer->getAngle(OldTimeLevel)-PI) < a0 &&
            fabs(edgePointer->getAngle(NewTimeLevel)-PI) < a0) {
            isMerged = true;
            // -----------------------------------------------------------------
            vertex1 = edgePointer->prev->getEndPoint(FirstPoint);
            vertex2 = edgePointer->getEndPoint(FirstPoint);
            vertex3 = edgePointer->getEndPoint(SecondPoint);
            polygon1 = polygon;
            polygon2 = edgePointer->getPolygon(OrientRight);
            if (vertex1 == edge1->getEndPoint(FirstPoint)) {
                edgePointer1 = edge1->getEdgePointer(OrientLeft);
                edgePointer2 = edge1->getEdgePointer(OrientRight);
            } else {
                edgePointer1 = edge1->getEdgePointer(OrientRight);
                edgePointer2 = edge1->getEdgePointer(OrientLeft);
            }
            // -----------------------------------------------------------------
            // Note: If the vertex is connected by more than two edges, that is
            //       a joint, then do not merge the edges.
            if (vertex2->isJoint())
                goto next_edge;
            // -----------------------------------------------------------------
            // avoid the line polygon in neighbor polygons
            if (edgePointer->getPolygon(OrientRight) != NULL &&
                edgePointer->getPolygon(OrientRight)->edgePointers.size() == 3)
                return false;
            // -----------------------------------------------------------------
            // new test point waiting for check
            Vertex testPoint;
            Coordinate x; Location loc;
            Sphere::calcMiddlePoint(vertex1->getCoordinate(OldTimeLevel),
                                    vertex3->getCoordinate(OldTimeLevel), x);
            meshManager.checkLocation(x, loc);
            testPoint.setCoordinate(x, NewTimeLevel);
            testPoint.setLocation(loc);
            TTS::track(meshManager, flowManager, &testPoint);
            // -----------------------------------------------------------------
            // check whether the edge-merging operation will cause edge-crossing
            // event or not
            if (detectRemoveVertexOnEdges(meshManager, edgePointer,
                                          &testPoint, polygon) != NoCross)
                goto next_edge;
            // -----------------------------------------------------------------
            // remove one edge pointer in each incident polygon
            polygon1->edgePointers.remove(edgePointer1->next);
            if (polygon2 != NULL)
                polygon2->edgePointers.remove(edgePointer2->prev);
            // -----------------------------------------------------------------
            // remove the shared vertex of the two old edges
            polygonManager.vertices.remove(vertex2);
            // -----------------------------------------------------------------
            // adjust edges
            if (vertex1 == edge1->getEndPoint(FirstPoint))
                edge1->changeEndPoint(SecondPoint, vertex3, &testPoint);
            else
                edge1->changeEndPoint(FirstPoint, vertex3, &testPoint);
            edge1->detectAgent.updateVertexProjections(meshManager);
            edge2->detectAgent.handoverVertices(edge1);
            polygonManager.edges.remove(edge2);
            // -----------------------------------------------------------------
            CommonTasks::doTask(CommonTasks::UpdateAngle);
        }
    next_edge: edgePointer = polygon->edgePointers.getNextElem();
    } while (!polygon->edgePointers.isLoopEnd());
    return isMerged;
}

bool CurvatureGuard::mergeEdges(MeshManager &meshManager,
                                const FlowManager &flowManager,
                                PolygonManager &polygonManager)
{
    bool isMerged = false;
    Polygon *polygon = polygonManager.polygons.front();
    while (polygon != NULL) {
        if (mergeEdge(meshManager, flowManager, polygonManager, polygon))
            isMerged = true;
        polygon = polygon->next;
    }
    return isMerged;
}

#endif