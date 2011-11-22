#ifndef mergeEdge_h
#define mergeEdge_h

#include "CurvatureGuard.h"
#include "TTS.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "PotentialCrossDetector.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

using namespace PotentialCrossDetector;

inline double relaxThreshold(Edge *edge1, Edge *edge2)
{
    static const double R0 = 0.5;
    static const double R1 = 0.9;
    static const double L0 = 0.001/Rad2Deg*Sphere::radius;
    static const double L1 = 4.0/Rad2Deg*Sphere::radius;
    static const double dR = R1-R0;
    static const double dL = L1-L0;
    double length = fmin(edge1->getLength(), edge2->getLength());
    if (length > L0 && length < L1) {
        double t = 1.0-(length-L0)/dL;
        return dR*(4.0-3.0*t)*pow(t, 3.0)+R0;
    } else if (length <= L0) {
        return R1;
    } else {
        return R0;
    }
}

inline bool mergeEdge(MeshManager &meshManager, const FlowManager &flowManager,
                      PolygonManager &polygonManager, Polygon *polygon)
{
    static const double smallAngle = 20.0/Rad2Deg;
    bool isMerged = false;

    TTS::resetTasks();

//    if (TimeManager::getSteps() >= 74 && polygon->getID() == 8132) {
//        polygon->dump("polygon");
//        REPORT_DEBUG;
//    }

    EdgePointer *edgePointer = polygon->edgePointers.front();
    EdgePointer *nextEdgePointer;
    while (edgePointer != NULL && edgePointer != polygon->edgePointers.back()) {
        nextEdgePointer = edgePointer->next;
        Edge *edge1 = edgePointer->prev->edge; // to be deleted if necessary
        Edge *edge2 = edgePointer->edge;
        double a0 = CurvatureGuard::angleThreshold(edge1, edge2)*
        relaxThreshold(edge1, edge2);
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
                return false;
            // -----------------------------------------------------------------
            // check whether the edge-merging operation will cause edge-crossing
            // event or not
            if (detect3(edge1, edge2) != NoCross)
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
            if (oldEdgePointer1->next->getAngle() < smallAngle) {
                edgePointer = edgePointer->next;
                continue;
            }
            if (oldEdgePointer1->prev->getAngle() < smallAngle) {
                edgePointer = edgePointer->next;
                continue;
            }
            if (oldEdgePointer2->getAngle() < smallAngle) {
                edgePointer = edgePointer->next;
                continue;
            }
            if (oldEdgePointer2->next->next->getAngle() < smallAngle) {
                edgePointer = edgePointer->next;
                continue;
            }
            // -----------------------------------------------------------------
            if (oldEdgePointer1->isTangly() ||
                oldEdgePointer2->next->isTangly()) {
                edgePointer = edgePointer->next;
                continue;
            }
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
                edge1->changeEndPoint(SecondPoint, vertex3,
                                      meshManager, flowManager);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(OrientRight, newEdgePointer2);
            } else {
                edge1->setEdgePointer(OrientRight, newEdgePointer1);
                edge1->changeEndPoint(FirstPoint, vertex3,
                                      meshManager, flowManager);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(OrientLeft, newEdgePointer2);
            }
            edge1->detectAgent.updateVertexProjections();
            edge2->detectAgent.handoverVertices(edge1);
            polygonManager.edges.remove(edge2);
            // -----------------------------------------------------------------
            TTS::doTask(TTS::UpdateAngle);
        }
    next_edge: edgePointer = nextEdgePointer;
    }
    return isMerged;
}

bool CurvatureGuard::mergeEdge(MeshManager &meshManager,
                               const FlowManager &flowManager,
                               PolygonManager &polygonManager)
{
    bool isMerged = false;
    Polygon *endPolygon = polygonManager.polygons.back()->next;
    Polygon *polygon = polygonManager.polygons.front();
    while (polygon != endPolygon) {
        if (mergeEdge(meshManager, flowManager, polygonManager, polygon))
            isMerged = true;
        polygon = polygon->next;
    }
    return isMerged;
}

#endif