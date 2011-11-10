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
    // major internal variables
    // vertex3 is the vertex which is approaching to edge1
    // vertex1 and vertex2 are the end points of the edge1
    // polygon1 is the polygon which will be split
    // polygon2 is the neighbor of polygon1 at edge1
    // polygon3 is the new polygon which is split from polygon1
    Vertex *vertex1, *vertex2, *vertex3;
    Edge *edge1;
    EdgePointer *edgePointer1, *edgePointer2, *edgePointer3;
    Polygon *polygon1, *polygon2, *polygon3, *polygon4;
    // other internal variables
    int i, j;
    EdgePointer *edgePointer, *linkedEdge;

    TTS::resetTasks();

    while (!ApproachingVertices::isEmpty()) {
        vertex3 = ApproachingVertices::vertices.front();
        // ---------------------------------------------------------------------
        Projection *projection = vertex3->detectAgent.getActiveProjection();
        // Note: The approaching status of vertex may change during splitPolygon.
        if (projection == NULL) {
            ApproachingVertices::removeVertex(vertex3);
            continue;
        }
        // ---------------------------------------------------------------------
        edge1 = projection->getEdge();
        // ---------------------------------------------------------------------
        // check if there is another approaching vertex whose projection
        // distance is smaller than vertex3
        bool hasAnotherVertex = false;
        list<Vertex *>::const_iterator it = edge1->detectAgent.vertices.begin();
        for (; it != edge1->detectAgent.vertices.end(); ++it) {
            Projection *projection1 = (*it)->detectAgent.getActiveProjection();
            if (projection1 != NULL) {
                if (projection1->getDistance(NewTimeLevel) <
                    projection->getDistance(NewTimeLevel)) {
                    ApproachingVertices::jumpVertex(vertex3, *it);
                    hasAnotherVertex = true;
                    break;
                }
            }
        }
        if (hasAnotherVertex) continue;
        // ---------------------------------------------------------------------
        edgePointer2 = NULL;
        switch (projection->getOrient()) {
            case OrientLeft:
                polygon1 = edge1->getPolygon(OrientLeft);
                polygon2 = edge1->getPolygon(OrientRight);
                edgePointer1 = edge1->getEdgePointer(OrientLeft);
                break;
            case OrientRight:
                polygon1 = edge1->getPolygon(OrientRight);
                polygon2 = edge1->getPolygon(OrientLeft);
                edgePointer1 = edge1->getEdgePointer(OrientRight);
                break;
            default:
                REPORT_ERROR("Unknown orientation!");
        }
        linkedEdge = vertex3->linkedEdges.front();
        for (i = 0; i < vertex3->linkedEdges.size(); ++i) {
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
        if (TimeManager::getSteps() >= 69 && polygon1->getID() == 3328)
            REPORT_DEBUG;
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
            if (detect1(vertex3, testVertex, edge1) != NoCross) {
                // when option is 1/2/4, vertex3 will be eliminated, and this
                // may cause edge-crossing. If this happens, shift to option 5
                // by making testVertex be vertex3.
                if (option >= 3) delete testVertex;
                testVertex = vertex3; option = 5;
            }
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
        polygon4 = NULL;
        if (option == 1 && edgePointer1 == edgePointer2->next->next)
            polygon4 = edgePointer2->next->getPolygon(OrientRight);
        else if (option == 2 && edgePointer1 == edgePointer2->prev)
            polygon4 = edgePointer2->getPolygon(OrientRight);
        // ---------------------------------------------------------------------
        int numTracer = static_cast<int>(polygon1->tracers.size());
        double mass[numTracer];
        for (i = 0; i < numTracer; ++i)
            mass[i] = polygon1->tracers[i].getMass();
        // polygon1 and new polygon3 may be degenerated simultaneously, the mass
        // of original polygon1 should be assigned to its neighbors
        int numNeighbor = polygon1->edgePointers.size();
        Polygon *neighbors[numNeighbor];
        edgePointer = polygon1->edgePointers.front();
        if (polygon4 == NULL)
            for (i = 0; i < numNeighbor; ++i) {
                neighbors[i] = edgePointer->getPolygon(OrientRight);
                edgePointer = edgePointer->next;
            }
        else {
            // put polygon4 into the last neighbor
            j = 0;
            for (i = 0; i < numNeighbor; ++i) {
                if (edgePointer->getPolygon(OrientRight) != polygon4)
                    neighbors[j++] = edgePointer->getPolygon(OrientRight);
                edgePointer = edgePointer->next;
            }
            neighbors[j] = polygon4;
        }
        // ---------------------------------------------------------------------
        // create a new polygon
        polygonManager.polygons.append(&polygon3);
        Vertex *newVertex;
        edgePointer3 = edgePointer2->next;
        EdgePointer *endEdgePointer;
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
            if (option != 5) {
                polygonManager.vertices.append(&newVertex);
                *newVertex = *testVertex;
                meshManager.countPoint(newVertex);
            } else
                newVertex = testVertex;
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
            if (option != 5) {
                vertex3->handoverEdges(newVertex, meshManager,
                                       flowManager, polygonManager);
                polygonManager.vertices.remove(vertex3);
                vertex3 = NULL;
            }
        } else
            vertex3->detectAgent.clean();
        // ---------------------------------------------------------------------
        // handle degenerate polygons
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
        if (polygon4 != NULL)
            if (polygon4->edgePointers.size() == 2) {
                Polygon *polygon41 = polygon4->edgePointers.front()->getPolygon(OrientRight);
                Polygon *polygon42 = polygon4->edgePointers.back()->getPolygon(OrientRight);
                for (i = 0; i < numTracer; ++i) {
                    double mass = polygon4->tracers[i].getMass()*0.5;
                    polygon41->tracers[i].addMass(mass);
                    polygon42->tracers[i].addMass(mass);
                }
                numNeighbor--;
                Polygon::handleLinePolygon(polygonManager, polygon4);
            }
        // ---------------------------------------------------------------------
        TTS::doTask(TTS::UpdateAngle);
        // ---------------------------------------------------------------------
        // assign mass to new parcels or neighbors
        if (polygon1 != NULL && polygon3 != NULL) {
            polygon1->calcArea(); polygon3->calcArea();
            double totalArea = polygon1->getArea()+polygon3->getArea();
            double weight1 = polygon1->getArea()/totalArea;
            double weight3 = polygon3->getArea()/totalArea;
            polygon3->tracers.resize(numTracer);
            for (i = 0; i < numTracer; ++i) {
                polygon1->tracers[i].setMass(mass[i]*weight1);
                polygon3->tracers[i].setMass(mass[i]*weight3);
            }
        } else if (polygon1 == NULL && polygon3 != NULL) {
            polygon3->tracers.resize(numTracer);
            for (i = 0; i < numTracer; ++i)
                polygon3->tracers[i].setMass(mass[i]);
        } else if (polygon1 == NULL && polygon3 == NULL) {
            for (i = 0; i < numTracer; ++i)
                mass[i] /= numNeighbor;
            for (j = 0; j < numNeighbor; ++j)
                for (i = 0; i < numTracer; ++i)
                    neighbors[j]->tracers[i].addMass(mass[i]);
        }
#ifdef DEBUG
        //        static double targetMass = 10.9682466106844085379;
        //        double totalMass = 0.0;
        //        polygon2 = polygonManager.polygons.front();
        //        for (i = 0; i < polygonManager.polygons.size(); ++i) {
        //            totalMass += polygon2->tracers[2].getMass();
        //            polygon2 = polygon2->next;
        //        }
        //        if (fabs(totalMass-targetMass) > 1.0e-9)
        //            REPORT_DEBUG;
#endif
        // ---------------------------------------------------------------------
        // check if polygon1 or polygon3 has been contained by another polygon
        if (polygon1 != NULL) {
            polygon2 = polygon1->edgePointers.front()->getPolygon(OrientRight);
            edgePointer1 = polygon1->edgePointers.front();
            for (i = 0; i < polygon1->edgePointers.size(); ++i) {
                if (edgePointer1->getPolygon(OrientRight) != polygon2)
                    break;
                edgePointer1 = edgePointer1->next;
            }
            if (edgePointer1 == polygon1->edgePointers.front()) {
                edgePointer2 = NULL; edgePointer3 = NULL;
                edgePointer1 = polygon1->edgePointers.front();
                for (i = 0; i < polygon1->edgePointers.size(); ++i) {
                    if (edgePointer1->getEndPoint(FirstPoint)->linkedEdges.size() > 2) {
                        edgePointer2 = edgePointer1->prev->getNeighborEdgePointer();
                        edgePointer3 = edgePointer1->getNeighborEdgePointer();
                        break;
                    }
                    edgePointer1 = edgePointer1->next;
                }
                edgePointer = edgePointer2;
                while (edgePointer != edgePointer3->next) {
                    polygon2->edgePointers.remove(edgePointer);
                    if (edgePointer != edgePointer2)
                        polygonManager.vertices.remove
                        (edgePointer->getEndPoint(FirstPoint));
                    polygonManager.edges.remove(edgePointer->edge);
                    edgePointer = edgePointer->next;
                }
                edgePointer->calcAngle();
                polygonManager.polygons.remove(polygon1);
            }
        }
        if (polygon3 != NULL) {
            polygon2 = polygon3->edgePointers.front()->getPolygon(OrientRight);
            edgePointer1 = polygon3->edgePointers.front();
            for (i = 0; i < polygon3->edgePointers.size(); ++i) {
                if (edgePointer1->getPolygon(OrientRight) != polygon2)
                    break;
                edgePointer1 = edgePointer1->next;
            }
            if (edgePointer1 == polygon3->edgePointers.front()) {
                edgePointer2 = NULL; edgePointer3 = NULL;
                edgePointer1 = polygon3->edgePointers.front();
                for (i = 0; i < polygon3->edgePointers.size(); ++i) {
                    if (edgePointer1->getEndPoint(FirstPoint)->linkedEdges.size() > 2) {
                        edgePointer2 = edgePointer1->prev->getNeighborEdgePointer();
                        edgePointer3 = edgePointer1->getNeighborEdgePointer();
                        break;
                    }
                    edgePointer1 = edgePointer1->next;
                }
                edgePointer = edgePointer2;
                while (edgePointer != edgePointer3->next) {
                    polygon2->edgePointers.remove(edgePointer);
                    if (edgePointer != edgePointer2)
                        polygonManager.vertices.remove
                        (edgePointer->getEndPoint(FirstPoint));
                    polygonManager.edges.remove(edgePointer->edge);
                    edgePointer = edgePointer->next;
                }
                edgePointer->calcAngle();
                polygonManager.polygons.remove(polygon3);
            }
        }
        // ---------------------------------------------------------------------
        // detect the new vertex for approaching
        if (option >= 3) {
            linkedEdge = newVertex->linkedEdges.front();
            for (i = 0; i < newVertex->linkedEdges.size(); ++i) {
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
        }
        // ---------------------------------------------------------------------
        if (option == 3 || option == 4)
            delete testVertex;
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