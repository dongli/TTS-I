#include "SpecialPolygons.h"
#include "PolygonManager.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "ApproachDetector.h"
#include "PotentialCrossDetector.h"
#include "CurvatureGuard.h"
#include "TTS.h"
#include "CommonTasks.h"

using namespace SpecialPolygons;
using namespace ApproachDetector;
using namespace PotentialCrossDetector;
using namespace CurvatureGuard;

void SpecialPolygons::handleLinePolygon(PolygonManager &polygonManager,
                                        Polygon *polygon, bool isKeepMass)
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
            polygonManager.edges.remove(edge1);
            polygonManager.edges.remove(edge2);
            polygonManager.vertices.remove(edgePointer4->getEndPoint(FirstPoint));
        } else if (edgePointer4->next == edgePointer3) {
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer3);
            CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer4); 
            CommonTasks::recordTask(CommonTasks::UpdateAngle, edgePointer3->next);
            polygon1->edgePointers.remove(edgePointer3);
            polygon1->edgePointers.remove(edgePointer4);
            polygonManager.edges.remove(edge1);
            polygonManager.edges.remove(edge2);
            polygonManager.vertices.remove(edgePointer3->getEndPoint(FirstPoint));
        } else {
            REPORT_ERROR("Line polygon is enclosed and there is "
                         "another polygon on the other end!");
        }
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
#ifdef DEBUG
    assert(edgePointer1->edge->getEndPoint(FirstPoint) ==
           edgePointer1->edge->getEndPoint(SecondPoint));
#endif
    polygonManager.polygons.remove(polygon);
    polygonManager.edges.remove(edgePointer1->edge);
    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer1);
    CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer2);
}

// TODO: In this function, we call "splitPolygon" where the tracer masses will
//       be handed over, but the tracer masses of the polygon may should be
//       preserved, because the caller of this function may need to hand over
//       them too!!!
bool SpecialPolygons::handleSlimPolygon(MeshManager &meshManager,
                                        const FlowManager &flowManager,
                                        PolygonManager &polygonManager,
                                        Polygon *polygon, bool isKeepMass)
{
    static const double smallAngle = 10.0/Rad2Deg;
    // Note: Currently, we only deal with triangles.
    if (polygon->edgePointers.size() == 3) {
        // 1. calculate angles if necessary
        EdgePointer *edgePointer = polygon->edgePointers.front();
        for (int i = 0; i < polygon->edgePointers.size(); ++i) {
            if (edgePointer->getAngle() == UNSET_ANGLE) {
                edgePointer->calcAngle();
                CommonTasks::deleteTask(CommonTasks::UpdateAngle, edgePointer);
            }
            edgePointer = edgePointer->next;
        }
        // 2. set up the edge pointers
        EdgePointer *edgePointer1 = NULL;
        EdgePointer *edgePointer2 = NULL;
        EdgePointer *edgePointer3 = NULL;
        edgePointer = polygon->edgePointers.front();
        for (int i = 0; i < polygon->edgePointers.size()+1; ++i) {
            if (edgePointer->getAngle() < smallAngle &&
                fabs(edgePointer->prev->getAngle()+
                     edgePointer->next->getAngle()-PI) < 1.0/Rad2Deg) {
                edgePointer1 = edgePointer;
                edgePointer2 = edgePointer->next;
                edgePointer3 = edgePointer->prev;
                break;
            }
            edgePointer = edgePointer->next;
        }
        if (edgePointer1 != NULL) {
            // 3. detect the triangle
            detectPolygon(meshManager, flowManager, polygonManager, polygon);
            // 4. set up the help pointers
            Polygon *polygon1 = edgePointer1->getPolygon(OrientRight);
            Polygon *polygon2 = edgePointer2->getPolygon(OrientRight);
            Polygon *polygon3 = edgePointer3->getPolygon(OrientRight);
            Vertex *vertex2 = edgePointer2->getEndPoint(FirstPoint);
            Vertex *vertex3 = edgePointer3->getEndPoint(FirstPoint);
            Edge *edge1 = edgePointer1->edge;
            Edge *edge3 = edgePointer3->edge;
            // 5. detect the neighbor polygons to avoid potential edge-crossing
            detectPolygon(meshManager, flowManager, polygonManager, polygon1);
            detectPolygon(meshManager, flowManager, polygonManager, polygon2);
            detectPolygon(meshManager, flowManager, polygonManager, polygon3);
            // 6. split the triangle
            Projection *projection2, *projection3;
            int mode;
            projection2 = vertex2->detectAgent.getProjection(edge3);
            projection3 = vertex3->detectAgent.getProjection(edge1);
            if (projection2 != NULL) {
                vertex2->tags.set(MayCrossEdge);
                mode = ApproachDetector::chooseMode(edgePointer3, vertex2,
                                                    projection2);
                assert(mode != -1);
                if (!projection2->isApproaching()) {
                    projection2->setApproach(true);
                    ApproachingVertices::recordVertex(vertex2);
                }
                splitPolygon(meshManager, flowManager, polygonManager, polygon,
                             edgePointer3, edgePointer1, vertex2, mode);
            } else if (projection3 != NULL) {
                vertex3->tags.set(MayCrossEdge);
                mode = ApproachDetector::chooseMode(edgePointer1, vertex3,
                                                    projection3);
                assert(mode != -1);
                if (!projection3->isApproaching()) {
                    projection3->setApproach(true);
                    ApproachingVertices::recordVertex(vertex3);
                }
                splitPolygon(meshManager, flowManager, polygonManager, polygon,
                             edgePointer1, edgePointer2, vertex3, mode);
            } else {
                REPORT_ERROR("Unexpected branch!");
            }
            return true;
        }
    }
    return false;
}