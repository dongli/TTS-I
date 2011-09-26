/*
 * splitPolygon
 *
 * Description:
 *   Due to the deformation caused by the background flow, the polygons of
 *   tracer will be deformed severely at some time. The computation may even be
 *   disrupted by the wild tangling of edges. Additionally, the incrementally
 *   inserted edges will cost huge amount of memory, which will be impractical
 *   for real applications. Therefore, we should detect the situations when
 *   problems will occur, and fix them.
 *
 *   Potential situations:
 *   (1) Some vertices approach some edges and may across them;
 *
 */

#ifndef splitPolygon_h
#define splitPolygon_h

#include "CurvatureGuard.h"
#include "TTS.h"
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "ApproachDetector.h"
#include "PotentialCrossDetector.h"

using namespace ApproachDetector;
using namespace PotentialCrossDetector;

bool CurvatureGuard::splitPolygon(MeshManager &meshManager,
                                  const FlowManager &flowManager,
                                  PolygonManager &polygonManager)
{
    bool isSplit = false;
    static const double smallDistance = 0.05/Rad2Deg*Sphere::radius;

    TTS::resetTasks();

    while (!ApproachingVertices::isEmpty()) {
        Vertex *vertex3 = ApproachingVertices::vertices.front();
        // ---------------------------------------------------------------------
        Projection *projection = vertex3->detectAgent.getActiveProjection();
        // Note: The approaching status of vertex may change during splitPolygon.
        if (projection == NULL) {
            ApproachingVertices::removeVertex(vertex3);
            continue;
        }
        // ---------------------------------------------------------------------
        Edge *edge1 = projection->getEdge();
        Vertex *vertex1, *vertex2;
        Polygon *polygon1, *polygon2;
        EdgePointer *edgePointer1, *edgePointer2 = NULL;
        if (projection->getOrient() == OrientLeft) {
            polygon1 = edge1->getPolygon(OrientLeft);
            polygon2 = edge1->getPolygon(OrientRight);
            edgePointer1 = edge1->getEdgePointer(OrientLeft);
        } else {
            polygon1 = edge1->getPolygon(OrientRight);
            polygon2 = edge1->getPolygon(OrientLeft);
            edgePointer1 = edge1->getEdgePointer(OrientRight);
        }
        EdgePointer *linkedEdge = vertex3->linkedEdges.front();
        for (int i = 0; i < vertex3->linkedEdges.size(); ++i) {
            if (linkedEdge->edge->getPolygon(OrientLeft) == polygon1 &&
                linkedEdge->edge->getEndPoint(SecondPoint) == vertex3) {
                edgePointer2 = linkedEdge->edge->getEdgePointer(OrientLeft);
                break;
            } else if (linkedEdge->edge->getPolygon(OrientRight) == polygon1 &&
                       linkedEdge->edge->getEndPoint(FirstPoint) == vertex3) {
                edgePointer2 = linkedEdge->edge->getEdgePointer(OrientRight);
                break;
            }
            linkedEdge = linkedEdge->next;
        }
#ifdef DEBUG
        if (edgePointer2 == NULL) {
            polygon1->dump("polygon");
        }
        assert(edgePointer2 != NULL);
#endif
        vertex1 = edgePointer1->getEndPoint(FirstPoint);
        vertex2 = edgePointer1->getEndPoint(SecondPoint);
#ifdef DEBUG
        if (edgePointer2->getPolygon(OrientLeft) != polygon1) {
            edgePointer2->getPolygon(OrientLeft)->dump("polygon");
            ostringstream message;
            message << "Edge-crossing event has occurred at polygon ";
            message << edgePointer2->getPolygon(OrientLeft)->getID() << "!" << endl;
            REPORT_ERROR(message.str())
        }
#endif
        // ---------------------------------------------------------------------
#ifdef DEBUG
        SEPERATOR
        ApproachingVertices::dump();
        cout << "***** current approaching vertex ID: ";
        cout << vertex3->getID() << endl;
        cout << "      Polygon ID: "<< polygon1->getID() << endl;
        cout << "      Paired edge ID: " << edge1->getID() << endl;
        cout << "      End point IDs: ";
        cout << setw(8) << edge1->getEndPoint(FirstPoint)->getID();
        cout << setw(8) << edge1->getEndPoint(SecondPoint)->getID();
        cout << endl;
        cout << "      Linked edge ID: ";
        cout << setw(8) << edgePointer2->edge->getID() << endl;
        cout << "      End point IDs: ";
        cout << setw(8) << edgePointer2->getEndPoint(FirstPoint)->getID();
        cout << setw(8) << edgePointer2->getEndPoint(SecondPoint)->getID();
        cout << endl;
        cout << "      Distance: ";
        cout << projection->getDistance(NewTimeLevel) << endl;
        cout << "      Orientation: ";
        if (projection->getOrient() == OrientLeft)
            cout << "left" << endl;
        else if (projection->getOrient() == OrientRight)
            cout << "right" << endl;
        polygon1->dump("polygon");
        DebugTools::assert_consistent_projection(projection);
        assert(edgePointer2->getEndPoint(SecondPoint) == vertex3);
        REPORT_DEBUG
#endif
        isSplit = true;
        // ---------------------------------------------------------------------
        double d1 = Sphere::calcDistance(vertex1->getCoordinate(),
                                         vertex3->getCoordinate());
        double d2 = Sphere::calcDistance(vertex2->getCoordinate(),
                                         vertex3->getCoordinate());
        int option;
        if (d1 <= d2 && d1 < smallDistance) {
            option = 1;
        } else if (d2 < d1 && d2 < smallDistance) {
            option = 2;
        } else if (projection->getDistance(NewTimeLevel) > smallDistance) {
            option = 3;
        } else {
            option = 4;
        }
        // ---------------------------------------------------------------------
        Vertex *testVertex;
        Location loc;
        OrientStatus orient;
        if (option == 1) {
            testVertex = vertex1;
        } else if (option == 2) {
            testVertex = vertex2;
        } else if (option >= 3) {
            testVertex = new Vertex;
            testVertex->setCoordinate(projection->getCoordinate(OldTimeLevel));
            meshManager.checkLocation(testVertex->getCoordinate(), loc);
            testVertex->setLocation(loc);
            TTS::track(meshManager, flowManager, testVertex);
        }
        // ---------------------------------------------------------------------
        if (option != 3) {
            if (detect1(vertex3, testVertex, edge1) != NoCross)
                continue;
            if (detect2(testVertex, edge1, polygon2) == Cross) {
                testVertex->setCoordinate
                (projection->getCoordinate(NewTimeLevel), NewTimeLevel);
                testVertex->setCoordinate
                (projection->getCoordinate(OldTimeLevel), OldTimeLevel);
                meshManager.checkLocation(testVertex->getCoordinate(), loc);
                testVertex->setLocation(loc);
            }
        }
        // ---------------------------------------------------------------------
        ApproachDetector::AgentPair::unpair(vertex3, edge1);
        ApproachingVertices::removeVertex(vertex3);
        // ---------------------------------------------------------------------
        // splitPolygon may affect the neightbor polygon, so record it for later
        // processing
        Polygon *polygon4 = NULL;
        EdgePointer *edgePointer4 = NULL;
        if (option == 1 && edgePointer1 == edgePointer2->next->next)
            if (edgePointer2->next->getPolygon(OrientLeft) == polygon1) {
                polygon4 = edgePointer2->next->getPolygon(OrientRight);
                edgePointer4 = edgePointer2->next->edge->getEdgePointer(OrientRight);
            } else {
                polygon4 = edgePointer2->next->getPolygon(OrientLeft);
                edgePointer4 = edgePointer2->next->edge->getEdgePointer(OrientLeft);
            }
        else if (option == 2 && edgePointer1 == edgePointer2->prev)
            if (edgePointer2->getPolygon(OrientLeft) == polygon1) {
                polygon4 = edgePointer2->getPolygon(OrientRight);
                edgePointer4 = edgePointer2->edge->getEdgePointer(OrientRight);
            } else {
                polygon4 = edgePointer2->getPolygon(OrientLeft);
                edgePointer4 = edgePointer2->edge->getEdgePointer(OrientLeft);
            }
        // ---------------------------------------------------------------------
        // create a new polygon
        Polygon *polygon3;
        polygonManager.polygons.append(&polygon3);
        EdgePointer *edgePointer3 = edgePointer2->next;
        EdgePointer *endEdgePointer;
        Vertex *newVertex;
        if (option == 1) {
            endEdgePointer = edgePointer1;
            newVertex = vertex1;
        } else if (option == 2) {
            endEdgePointer = edgePointer1->next;
            newVertex = vertex2;
        } else {
            endEdgePointer = edgePointer1;
        }
        while (edgePointer3 != endEdgePointer) {
            EdgePointer *edgePointer;
            polygon3->edgePointers.append(&edgePointer);
            edgePointer->replace(edgePointer3);
            edgePointer->edge->setPolygon(edgePointer->orient, polygon3);
            edgePointer3 = edgePointer3->next;
            polygon1->edgePointers.remove(edgePointer3->prev);
        }
        // ---------------------------------------------------------------------
        Edge *newEdge1 = NULL;
        // check if the projection of vertex on the
        // edge 1 is too close to vertex 1 or 2
        // if true, just use the 1 or 2
        // if not, use the new one
        if (option < 3) {
            polygon3->edgePointers.ring();
        } else {
            polygon3->edgePointers.append(&edgePointer3);
            polygon3->edgePointers.ring();
            // create a new vertex on the edge 1
            polygonManager.vertices.append(&newVertex);
            *newVertex = *testVertex;
            meshManager.countPoint(newVertex);
            // split the edge pointed by edge 1
            polygonManager.edges.append(&newEdge1);
            newEdge1->linkEndPoint(FirstPoint, vertex1);
            newEdge1->linkEndPoint(SecondPoint, newVertex);
            newEdge1->setPolygon(OrientLeft, polygon3);
            newEdge1->setEdgePointer(OrientLeft, edgePointer3);
            newEdge1->setPolygon(OrientRight, polygon2);
            EdgePointer *edgePointer5, *edgePointer6;
            if (edgePointer1->orient == OrientLeft) {
                edgePointer5 = edgePointer1->edge->getEdgePointer(OrientRight);
            } else {
                edgePointer5 = edgePointer1->edge->getEdgePointer(OrientLeft);
            }
            polygon2->edgePointers.insert(edgePointer5, &edgePointer6);
            newEdge1->setEdgePointer(OrientRight, edgePointer6);
            newEdge1->calcNormVector();
            newEdge1->calcLength();
            Vertex *testPoint = newEdge1->getTestPoint();
            meshManager.checkLocation(testPoint->getCoordinate(), loc);
            testPoint->setLocation(loc);
            TTS::track(meshManager, flowManager, testPoint);
        }
        // ---------------------------------------------------------------------
        // judge whether we need to add another new edge to connect
        // the new vertex and vertex
        Edge *newEdge2 = NULL;
        if (option == 3) {
            polygonManager.edges.append(&newEdge2);
            newEdge2->linkEndPoint(FirstPoint, vertex3);
            newEdge2->linkEndPoint(SecondPoint, newVertex);
            newEdge2->setPolygon(OrientLeft, polygon1);
            EdgePointer *edgePointer7;
            polygon1->edgePointers.insert(edgePointer2, &edgePointer7);
            newEdge2->setEdgePointer(OrientLeft, edgePointer7);
            newEdge2->setPolygon(OrientRight, polygon3);
            EdgePointer *edgePointer8;
            polygon3->edgePointers.insert(edgePointer3, &edgePointer8);
            newEdge2->setEdgePointer(OrientRight, edgePointer8);
            newEdge2->calcNormVector();
            newEdge2->calcLength();
            Vertex *testPoint = newEdge2->getTestPoint();
            Location loc;
            meshManager.checkLocation(testPoint->getCoordinate(), loc);
            testPoint->setLocation(loc);
            TTS::track(meshManager, flowManager, testPoint);
        }
        // ---------------------------------------------------------------------
        // modify the old edges
        if (newEdge1 != NULL) {
            if (edgePointer1->orient == OrientLeft) {
                edge1->changeEndPoint(FirstPoint, newVertex,
                                      meshManager, flowManager);
            } else {
                edge1->changeEndPoint(SecondPoint, newVertex,
                                      meshManager, flowManager);
            }
            if (newEdge1 != NULL && polygon3 != NULL)
                edge1->detectAgent.handoverVertices(newEdge1);
            edge1->detectAgent.updateVertexProjections();
        }
        if (newEdge2 == NULL) {
            vertex3->handoverEdges(newVertex, meshManager,
                                   flowManager, polygonManager);
            polygonManager.vertices.remove(vertex3);
            vertex3 = NULL;
        } else
            vertex3->detectAgent.clean();
        // ---------------------------------------------------------------------
        if (polygon1->edgePointers.size() == 1) {
            Polygon::handlePointPolygon(polygonManager, polygon1);
            polygon1 = NULL;
        }
        if (polygon3->edgePointers.size() == 1) {
            Polygon::handlePointPolygon(polygonManager, polygon3);
            polygon3 = NULL;
        }
        if (polygon1 != NULL && polygon1->edgePointers.size() == 2) {
            Polygon::handleLinePolygon(polygonManager, polygon1);
            polygon1 = NULL;
        }
        if (polygon3 != NULL && polygon3->edgePointers.size() == 2) {
            Polygon::handleLinePolygon(polygonManager, polygon3);
            polygon3 = NULL;
        }
        if (polygon4 != NULL) {
            assert(edgePointer4->getEndPoint(FirstPoint) ==
                   edgePointer4->getEndPoint(SecondPoint));
            if (polygon4->edgePointers.size() == 2)
                Polygon::handleLinePolygon(polygonManager, polygon4);
        }
        // ---------------------------------------------------------------------
        TTS::doTask(TTS::UpdateAngle);
        // ---------------------------------------------------------------------
        // detect the new vertex for approaching
        if (option >= 3) {
            EdgePointer *linkedEdge = newVertex->linkedEdges.front();
            for (int i = 0; i < newVertex->linkedEdges.size(); ++i) {
                Edge *edge = linkedEdge->edge;
                if (edge->getEndPoint(FirstPoint) == newVertex)
                    orient = OrientLeft;
                else if (edge->getEndPoint(SecondPoint) == newVertex)
                    orient = OrientRight;
#ifdef DEBUG
                cout << "Detecting polygon ";
                cout << edge->getPolygon(orient)->getID() << endl;
#endif
                ApproachDetector::detect(meshManager, flowManager,
                                         edge->getPolygon(orient));
                linkedEdge = linkedEdge->next;
            }
#ifdef DEBUG
            cout << "New vertex " << newVertex->getID() << " is linked to ";
            cout << newVertex->linkedEdges.size() << " edges." << endl;
            REPORT_DEBUG
#endif
            delete testVertex;
        }
        // ---------------------------------------------------------------------
#ifdef DEBUG
        if (polygon1 != NULL) {
            cout << "Splitted polygon 1 ID: " << polygon1->getID() << endl;
            polygon1->dump("polygon1");
            REPORT_DEBUG
        }
        if (polygon3 != NULL) {
            cout << "Splitted polygon 3 ID: " << polygon3->getID() << endl;
            polygon3->dump("polygon3");
            REPORT_DEBUG
        }
#endif
    }
    
    return isSplit;
}

#endif