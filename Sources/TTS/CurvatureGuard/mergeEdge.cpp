#ifndef mergeEdge_h
#define mergeEdge_h

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

inline bool mergeEdge(MeshManager &meshManager, const FlowManager &flowManager,
                      PolygonManager &polygonManager, Polygon *polygon)
{
    bool isMerged = false;

    Edge *edge1, *edge2;
    double a0;

    assert(CommonTasks::getTaskNumber(CommonTasks::UpdateAngle) == 0);

//    if (TimeManager::getSteps() >= 420 && (polygon->getID() == 124665)) {
//        polygon->dump("polygon");
//        REPORT_DEBUG;
//    }

    if (polygon->edgePointers.size() == 3)
        return isMerged;

    EdgePointer *edgePointer = polygon->edgePointers.front();
    EdgePointer *nextEdgePointer;
    while (edgePointer != NULL && edgePointer != polygon->edgePointers.back()) {
        nextEdgePointer = edgePointer->next;
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
            Vertex *vertex1, *vertex2, *vertex3;  // vertex2 is to be deleted
            vertex1 = edgePointer->prev->getEndPoint(FirstPoint);
            vertex2 = edgePointer->getEndPoint(FirstPoint);
            vertex3 = edgePointer->getEndPoint(SecondPoint);
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
            EdgePointer *oldEdgePointer1, *oldEdgePointer2;
            if (vertex2 == edge2->getEndPoint(FirstPoint)) {
                oldEdgePointer1 = edge2->getEdgePointer(OrientLeft);
                oldEdgePointer2 = edge2->getEdgePointer(OrientRight);
            } else {
                oldEdgePointer1 = edge2->getEdgePointer(OrientRight);
                oldEdgePointer2 = edge2->getEdgePointer(OrientLeft);
            }            
#ifdef DEBUG
            assert(oldEdgePointer1 == edgePointer);
#endif
            // -----------------------------------------------------------------
            Polygon *polygon1, *polygon2;
            if (vertex1 == edge1->getEndPoint(FirstPoint)) {
                polygon1 = edge1->getPolygon(OrientLeft);
                polygon2 = edge1->getPolygon(OrientRight);
            } else {
                polygon1 = edge1->getPolygon(OrientRight);
                polygon2 = edge1->getPolygon(OrientLeft);
            }
#ifdef DEBUG
            assert(polygon1 == polygon);
#endif
            // -----------------------------------------------------------------
            EdgePointer *newEdgePointer1, *newEdgePointer2;
            polygon1->edgePointers.insert(oldEdgePointer1, &newEdgePointer1);
            polygon1->edgePointers.remove(oldEdgePointer1->prev);
            polygon1->edgePointers.remove(oldEdgePointer1);
            if (polygon2 != NULL) {
                polygon2->edgePointers.insert(&newEdgePointer2, oldEdgePointer2);
                polygon2->edgePointers.remove(oldEdgePointer2->next);
                polygon2->edgePointers.remove(oldEdgePointer2);
            } else {
                newEdgePointer2 = NULL;
            }
            // -----------------------------------------------------------------
            // adjust vertices
            polygonManager.vertices.remove(vertex2);
            // -----------------------------------------------------------------
            // adjust edges
            if (vertex1 == edge1->getEndPoint(FirstPoint)) {
                edge1->setEdgePointer(OrientLeft, newEdgePointer1);
                edge1->changeEndPoint(SecondPoint, vertex3, &testPoint,
                                      meshManager, flowManager);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(OrientRight, newEdgePointer2);
            } else {
                edge1->setEdgePointer(OrientRight, newEdgePointer1);
                edge1->changeEndPoint(FirstPoint, vertex3, &testPoint,
                                      meshManager, flowManager);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(OrientLeft, newEdgePointer2);
            }
            edge1->detectAgent.updateVertexProjections(meshManager);
            edge2->detectAgent.handoverVertices(edge1);
            polygonManager.edges.remove(edge2);
            // -----------------------------------------------------------------
            CommonTasks::doTask(CommonTasks::UpdateAngle);
        }
    next_edge: edgePointer = nextEdgePointer;
    }
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