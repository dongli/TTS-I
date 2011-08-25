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

using namespace ApproachDetector;

bool TTS::splitPolygon(MeshManager &meshManager, const FlowManager &flowManager,
                       PolygonManager &polygonManager)
{
    bool isSplitted = false;
    static const double smallDistance = 0.05/Rad2Deg*Sphere::radius;

    resetTasks();
    
    while (!ApproachingVertices::isEmpty()) {
        Vertex *vertex3 = ApproachingVertices::vertexPointers.front()->vertex;
        if (TimeManager::getSteps() >= 23 && vertex3->getID() == 339732)
            REPORT_DEBUG
        if (vertex3->detectAgent.isApproaching) {
            Edge *edge1 = vertex3->detectAgent.edge;
#ifdef DEBUG
            assert(edge1 != NULL);
#endif
            Vertex *vertex1, *vertex2;
            Polygon *polygon1, *polygon2;
            EdgePointer *edgePointer1, *edgePointer2;
            if (vertex3->detectAgent.edgePointer->edge != edge1) {
                Edge *edge2 = vertex3->detectAgent.edgePointer->edge;
                if (vertex3->detectAgent.edgePointer->orient == OrientLeft) {
                    polygon1 = edge2->getPolygon(OrientLeft);
                    edgePointer2 = edge2->getEdgePointer(OrientLeft);
                } else if (vertex3->detectAgent.edgePointer->orient == OrientRight) {
                    polygon1 = edge2->getPolygon(OrientRight);
                    edgePointer2 = edge2->getEdgePointer(OrientRight);
                } else {
                    REPORT_ERROR("Unknown orientation!")
                }
                if (edge1->getPolygon(OrientLeft) == polygon1) {
                    polygon2 = edge1->getPolygon(OrientRight);
                    edgePointer1 = edge1->getEdgePointer(OrientLeft);
                } else if (edge1->getPolygon(OrientRight) == polygon1) {
                    polygon2 = edge1->getPolygon(OrientLeft);
                    edgePointer1 = edge1->getEdgePointer(OrientRight);
                }
#ifdef DEBUG
                assert(edgePointer2 == vertex3->detectAgent.edgePointer);
                assert(edgePointer2->getPolygon(OrientLeft) == polygon1 ||
                       edgePointer2->getPolygon(OrientRight) == polygon1);
#endif
            } else {
                if (vertex3->detectAgent.edgePointer->orient == OrientLeft) {
                    polygon1 = edge1->getPolygon(OrientLeft);
                    polygon2 = edge1->getPolygon(OrientRight);
                    edgePointer1 = edge1->getEdgePointer(OrientLeft);
                } else if (vertex3->detectAgent.edgePointer->orient == OrientRight) {
                    polygon1 = edge1->getPolygon(OrientRight);
                    polygon2 = edge1->getPolygon(OrientLeft);
                    edgePointer1 = edge1->getEdgePointer(OrientRight);
                } else {
                    REPORT_ERROR("Unknown orientation!")
                }
                edgePointer2 = edgePointer1->next;
                while (edgePointer2->getEndPoint(SecondPoint) != vertex3)
                    edgePointer2 = edgePointer2->next;
            }
            vertex1 = edgePointer1->getEndPoint(FirstPoint);
            vertex2 = edgePointer1->getEndPoint(SecondPoint);
#ifdef DEBUG
            assert(edgePointer2->getEndPoint(SecondPoint) == vertex3);
#endif
            // -----------------------------------------------------------------
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
            cout << "      Distance: ";
            cout << vertex3->detectAgent.distance << endl;
            cout << "      Orientation: ";
            if (vertex3->detectAgent.orient == OrientLeft)
                cout << "left" << endl;
            else if (vertex3->detectAgent.orient == OrientRight)
                cout << "right" << endl;
            polygon1->dump("polygon");
            const Coordinate &x1o = vertex1->getCoordinate(OldTimeLevel);
            const Coordinate &x2o = vertex2->getCoordinate(OldTimeLevel);
            const Coordinate &x1n = vertex1->getCoordinate();
            const Coordinate &x2n = vertex2->getCoordinate();
            const Coordinate &x3 = vertex3->detectAgent.projection;
            Coordinate x2or, x2nr, x3or, x3nr;
            Sphere::rotate(x1o, x2o, x2or);
            Sphere::rotate(x1n, x2n, x2nr);
            Sphere::rotate(x1o, x3, x3or);
            Sphere::rotate(x1n, x3, x3nr);
            if (vertex3->detectAgent.timeLevel == OldTimeLevel) {
                if (fabs(x2or.getLon()-x3or.getLon()) > EPS) {
                    vertex1->getCoordinate(OldTimeLevel).dump();
                    vertex3->detectAgent.projection.dump();
                    vertex2->getCoordinate(OldTimeLevel).dump();
                    REPORT_ERROR("Invalid projection!")
                }
            } else {
                if (fabs(x2nr.getLon()-x3nr.getLon()) > EPS) {
                    vertex1->getCoordinate().dump();
                    vertex3->detectAgent.projection.dump();
                    vertex2->getCoordinate().dump();
                    REPORT_ERROR("Invalid projection!")
                }   
            }
            REPORT_DEBUG
#endif
            isSplitted = true;
            // -------------------------------------------------------------
            double d1 = Sphere::calcDistance(vertex1->getCoordinate(),
                                             vertex3->getCoordinate());
            double d2 = Sphere::calcDistance(vertex2->getCoordinate(),
                                             vertex3->getCoordinate());
            int option;
            if (d1 <= d2 && d1 < smallDistance) {
                option = 1;
            } else if (d2 < d1 && d2 < smallDistance) {
                option = 2;
            } else if (vertex3->detectAgent.distance > smallDistance) {
                option = 3;
            } else {
                option = 4;
            }
            // -------------------------------------------------------------
            ApproachingVertices::remove(vertex3);
            ApproachDetector::AgentPairs::unpair(vertex3, edge1);
            // -----------------------------------------------------------------
            Vertex testVertex;
            Location loc;
            OrientStatus orient;
            if (option >= 3) {
                testVertex.setCoordinate(vertex3->detectAgent.projection);
                meshManager.checkLocation(testVertex.getCoordinate(), loc);
                testVertex.setLocation(loc);
                track(meshManager, flowManager, &testVertex, true);
            }
            if (option == 4) {
                // test if the coming creation of new vertex on edge 1 will not
                // cause edge-crossing
                EdgePointer *linkedEdge = vertex3->linkedEdges.front();
                for (int i = 0; i < vertex3->linkedEdges.size(); ++i) {
                    Edge *edge = linkedEdge->edge;
                    VertexPointer *vertexPointer = edge->detectAgent.vertexPointers.front();
                    int n = 0;
                    for (int j = 0; j < edge->detectAgent.vertexPointers.size(); ++j) {
                        if (edge->getEndPoint(FirstPoint) == vertex3) {
                            orient = Sphere::orient(testVertex.getCoordinate(),
                                                    edge->getEndPoint(SecondPoint)->getCoordinate(),
                                                    vertexPointer->vertex->getCoordinate());
                        } else if (edge->getEndPoint(SecondPoint) == vertex3) {
                            orient = Sphere::orient(edge->getEndPoint(FirstPoint)->getCoordinate(),
                                                    testVertex.getCoordinate(),
                                                    vertexPointer->vertex->getCoordinate());
                        }
                        if (orient != vertexPointer->vertex->detectAgent.orient) {
                            REPORT_ERROR("Edge-crossing has occurred!")
                            
                        }
                        n++;
                        vertexPointer = vertexPointer->next;
                    }
#ifdef DEBUG
                    cout << "Linked edge " << edge->getID() << " has " << n;
                    cout << " approaching vertices." << endl;
#endif
                    linkedEdge = linkedEdge->next;
                }
            }
            // -----------------------------------------------------------------
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
            // -------------------------------------------------------------
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
                *(newVertex) = testVertex;
                // split the edge pointed by edge 1
                polygonManager.edges.append(&newEdge1);
                vertex1->dislinkEdge(edgePointer1->edge);
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
                track(meshManager, flowManager, testPoint);
            }
            // -------------------------------------------------------------
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
                track(meshManager, flowManager, testPoint);
            }
            // -------------------------------------------------------------
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
                    edge1->detectAgent.handover(newEdge1);
                edge1->detectAgent.update();
            }
            if (newEdge2 == NULL) {
                vertex3->handoverEdges(newVertex, meshManager,
                                       flowManager, polygonManager);
                polygonManager.vertices.remove(vertex3);
                vertex3 = NULL;
            } else
                vertex3->detectAgent.clean();
            // -------------------------------------------------------------
            if (polygon1->edgePointers.size() == 2) {
                Polygon::handleLinePolygon(polygonManager, polygon1);
                polygon1 = NULL;
            } else if (polygon1->edgePointers.size() == 1) {
                Polygon::handlePointPolygon(polygonManager, polygon1);
                polygon1 = NULL;
                REPORT_DEBUG
            }
            if (polygon3->edgePointers.size() == 2) {
                Polygon::handleLinePolygon(polygonManager, polygon3);
                polygon3 = NULL;
            } else if (polygon3->edgePointers.size() == 1) {
                Polygon::handlePointPolygon(polygonManager, polygon3);
                polygon3 = NULL;
                REPORT_DEBUG
            }
            // -------------------------------------------------------------
            doTask(UpdateAngle);
            // -------------------------------------------------------------
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
                cout << "New vertex " << newVertex->getID() << " is linked to ";
                cout << newVertex->linkedEdges.size() << " edges." << endl;
                REPORT_DEBUG
            }
#ifdef DEBUG
            if (polygon1 != NULL) {
                cout << "Splitted polygon 1 ID: " << polygon1->getID() << endl;
                polygon1->dump("polygon1");
                //REPORT_DEBUG
            }
            if (polygon3 != NULL) {
                cout << "Splitted polygon 3 ID: " << polygon3->getID() << endl;
                polygon3->dump("polygon3");
                //REPORT_DEBUG
            }
#endif
        }
    }
    
    return isSplitted;
}
