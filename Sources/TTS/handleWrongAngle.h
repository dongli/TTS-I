bool TTS::handleWrongAngle(MeshManager &meshManager, const FlowManager &flowManager,
                           PolygonManager &polygonManager, Polygon *polygon)
{
    bool isHandled = false;
    double angleDiff;
    Location loc;
    Vertex *testPoint;

    EdgePointer *edgePointer = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        double oldAngle = edgePointer->getAngle(OldTimeLevel);
        if (edgePointer->isWrongAngle()) {
            isHandled = true;
            // -----------------------------------------------------------------
            // collect information
            EdgePointer *edgePointer1 = edgePointer->prev;
            EdgePointer *edgePointer2 = edgePointer;
            EdgePointer *edgePointer3 = edgePointer1->prev;
            EdgePointer *edgePointer4 = edgePointer2->next;
            Edge *edge1 = edgePointer1->edge;
            Edge *edge2 = edgePointer2->edge;
            Edge *edge3 = edgePointer3->edge;
            Edge *edge4 = edgePointer4->edge;
            EdgePointer *edgePointer5 = polygon == edge1->getPolygon(EdgeLeft) ?
            edge1->getEdgePointer(EdgeRight) : edge1->getEdgePointer(EdgeLeft);
            EdgePointer *edgePointer6 = polygon == edge2->getPolygon(EdgeLeft) ?
            edge2->getEdgePointer(EdgeRight) : edge2->getEdgePointer(EdgeLeft);
            EdgePointer *edgePointer7 = polygon == edge3->getPolygon(EdgeLeft) ?
            edge3->getEdgePointer(EdgeRight) : edge3->getEdgePointer(EdgeLeft);
            EdgePointer *edgePointer8 = polygon == edge4->getPolygon(EdgeLeft) ?
            edge4->getEdgePointer(EdgeRight) : edge4->getEdgePointer(EdgeLeft);
            Vertex *vertex3 = edgePointer1->getEndPoint(FirstPoint);
            Vertex *vertex1 = edgePointer2->getEndPoint(FirstPoint);
            Vertex *vertex2 = edgePointer4->getEndPoint(FirstPoint);
            // Note: Due to the significance of edge length, we calculate the
            //       length here to ensure the validity of it.
            REPORT_DEBUG
            edge1->calcLength();
            edge2->calcLength();
            double length1 = edge1->getLength();
            double length2 = edge2->getLength();
            Polygon *polygon1, *polygon2, *polygon3, *polygon4;
            if (oldAngle < PI) {
                polygon1 = polygon == edge1->getPolygon(EdgeLeft) ?
                edge1->getPolygon(EdgeRight) : edge1->getPolygon(EdgeLeft);
                polygon2 = polygon == edge2->getPolygon(EdgeLeft) ?
                edge2->getPolygon(EdgeRight) : edge2->getPolygon(EdgeLeft);
                polygon3 = polygon == edge3->getPolygon(EdgeLeft) ?
                edge3->getPolygon(EdgeRight) : edge3->getPolygon(EdgeLeft);
                polygon4 = polygon == edge4->getPolygon(EdgeLeft) ?
                edge4->getPolygon(EdgeRight) : edge4->getPolygon(EdgeLeft);
            } else {
                polygon1 = polygon == edge1->getPolygon(EdgeLeft) ?
                edge1->getPolygon(EdgeRight) : edge1->getPolygon(EdgeLeft);
                polygon2 = polygon == edge4->getPolygon(EdgeLeft) ?
                edge4->getPolygon(EdgeRight) : edge4->getPolygon(EdgeLeft);
                polygon3 = polygon == edge3->getPolygon(EdgeLeft) ?
                edge3->getPolygon(EdgeRight) : edge3->getPolygon(EdgeLeft);
                polygon4 = NULL;
            }
#ifdef DEBUG
            cout << "Handle wrong angle:" << endl;
            cout << "  Old angle:   " << oldAngle*Rad2Deg << endl;
            cout << "  New angle:   " << edgePointer->getAngle()*Rad2Deg << endl;
            cout << "  Polygon ID:  " << polygon->getID() << endl;
            cout << "  Edge 1 ID:   " << edge1->getID() << endl;
            cout << "  Edge 2 ID:   " << edge2->getID() << endl;
            cout << "  Edge 3 ID:   " << edge3->getID() << endl;
            cout << "  Edge 4 ID:   " << edge4->getID() << endl;
            cout << "  Vertex 1 ID: " << vertex1->getID() << endl;
            cout << "  Vertex 2 ID: " << vertex2->getID() << endl;
            cout << "  Vertex 3 ID: " << vertex3->getID() << endl;
            cout << "  Edge 1 length: " << length1 << endl;
            cout << "  Edge 2 length: " << length2 << endl;
            cout << "  Polygon 1 ID: ";
            if (polygon1 == NULL)
                cout << "None" << endl;
            else
                cout << polygon1->getID() << endl;
            cout << "  Polygon 2 ID: ";
            if (polygon2 == NULL)
                cout << "None" << endl;
            else
                cout << polygon2->getID() << endl;
            cout << "  Polygon 3 ID: ";
            if (polygon3 == NULL)
                cout << "None" << endl;
            else
                cout << polygon3->getID() << endl;
            cout << "  Polygon 4 ID: ";
            if (polygon4 == NULL)
                cout << "None" << endl;
            else
                cout << polygon4->getID() << endl;
#endif
            assert(length1 != length2);
            // -----------------------------------------------------------------
            // create new vertex
            Vertex *newVertex;
            polygonManager.vertices.append(&newVertex);
            double ratio = fmin(length1, length2)/fmax(length1, length2);
            Coordinate x1, x2;
            if (length1 > length2)
                x1 = vertex3->getCoordinate(OldTimeLevel);
            else
                x1 = vertex2->getCoordinate(OldTimeLevel);
            x2 = vertex1->getCoordinate(OldTimeLevel);
            Coordinate xr, xo;
            Sphere::rotate(x1, x2, xr);
            double dlat = (PI05-xr.getLat())*(1.0-ratio);
            xr.set(xr.getLon(), PI05-dlat);
            Sphere::inverseRotate(x1, xo, xr);
            meshManager.checkLocation(xo, loc, newVertex->getID());
            newVertex->setCoordinate(xo, NewTimeLevel);
            newVertex->setLocation(loc);
            track(meshManager, flowManager, newVertex);
            if (length1 > length2) {
                polygonManager.vertices.remove(vertex2);
            } else {
                polygonManager.vertices.remove(vertex3);
            }
            // -----------------------------------------------------------------
            // get the wings if there is :)
            vector<EdgePointer *> edgePointer13, edgePointer24, edgePointer12;
            vector<Edge *> edge13, edge24, edge12;
            if (oldAngle < PI) {
                if (polygon1 != polygon3 && length1 < length2) {
                    if (polygon1 != NULL) {
                        EdgePointer *tmp = edgePointer5->next;
                        while (true) {
                            edgePointer13.push_back(tmp);
                            if (tmp->edge->getEndPoint(FirstPoint) == vertex3)
                                tmp = tmp->edge->getEdgePointer(EdgeRight);
                            else
                                tmp = tmp->edge->getEdgePointer(EdgeLeft);
                            edgePointer13.push_back(tmp);
                            edge13.push_back(tmp->edge);
                            if (tmp->edge->getPolygon(EdgeLeft)  == polygon3 ||
                                tmp->edge->getPolygon(EdgeRight) == polygon3)
                                break;
                            tmp = tmp->next;
                        }
                    } else {
                        EdgePointer *tmp = edgePointer7->prev;
                        while (true) {
                            edgePointer13.push_back(tmp);
                            if (tmp->edge->getEndPoint(FirstPoint) == vertex3)
                                tmp = tmp->edge->getEdgePointer(EdgeLeft);
                            else
                                tmp = tmp->edge->getEdgePointer(EdgeRight);
                            edgePointer13.push_back(tmp);
                            edge13.push_back(tmp->edge);
                            if (tmp->edge->getPolygon(EdgeLeft)  == polygon1 ||
                                tmp->edge->getPolygon(EdgeRight) == polygon1)
                                break;
                            tmp = tmp->prev;
                        }
                    }
                }
                if (polygon2 != polygon4 && length1 > length2) {
                    if (polygon2 != NULL) {
                        EdgePointer *tmp = edgePointer6->prev;
                        while (true) {
                            edgePointer24.push_back(tmp);
                            if (tmp->edge->getEndPoint(FirstPoint) == vertex2)
                                tmp = tmp->edge->getEdgePointer(EdgeLeft);
                            else
                                tmp = tmp->edge->getEdgePointer(EdgeRight);
                            edgePointer24.push_back(tmp);
                            edge24.push_back(tmp->edge);
                            if (tmp->edge->getPolygon(EdgeLeft)  == polygon4 ||
                                tmp->edge->getPolygon(EdgeRight) == polygon4)
                                break;
                            tmp = tmp->prev;
                        }
                    } else {
                        EdgePointer *tmp = edgePointer8->next;
                        while (true) {
                            edgePointer24.push_back(tmp);
                            if (tmp->edge->getEndPoint(FirstPoint) == vertex2)
                                tmp = tmp->edge->getEdgePointer(EdgeRight);
                            else
                                tmp = tmp->edge->getEdgePointer(EdgeLeft);
                            edgePointer24.push_back(tmp);
                            edge24.push_back(tmp->edge);
                            if (tmp->edge->getPolygon(EdgeLeft)  == polygon2 ||
                                tmp->edge->getPolygon(EdgeRight) == polygon2)
                                break;
                            tmp = tmp->next;
                        }
                    }
                }
#ifdef DEBUG
                cout << "Wings:" << endl;
                cout << "  Edge pointers between polygon 1 and 3:" << endl;
                cout << "    number: " << edgePointer13.size() << endl;
                cout << "  Edges between polygon 1 and 3:" << endl;
                for (int j = 0; j < edge13.size(); ++j)
                    cout << edge13[j]->getID() << "  ";
                cout << endl;
                cout << "  Edge pointers between polygon 2 and 4:" << endl;
                cout << "    number: " << edgePointer24.size() << endl;
                cout << "  Edges between polygon 2 and 4:" << endl;
                for (int j = 0; j < edge24.size(); ++j)
                    cout << edge24[j]->getID() << "  ";
                cout << endl;
#endif
            } else {
                if (polygon1 != polygon2 && length1 < length2) {
                    EdgePointer *tmp = edgePointer8->prev;
                    while (true) {
                        edgePointer12.push_back(tmp);
                        if (tmp->edge->getEndPoint(FirstPoint) == vertex2) {
                            tmp = tmp->edge->getEdgePointer(EdgeRight);
                        } else {
                            tmp = tmp->edge->getEdgePointer(EdgeLeft);
                        }
                        edgePointer12.push_back(tmp);
                        edge12.push_back(tmp->edge);
                        if (tmp->edge->getPolygon(EdgeLeft)  == polygon1 ||
                            tmp->edge->getPolygon(EdgeRight) == polygon1)
                            break;
                        tmp = tmp->prev;
                    }
                }
                if (polygon1 != polygon3 && length1 > length2) {
                    EdgePointer *tmp = edgePointer7->next;
                    while (true) {
                        edgePointer13.push_back(tmp);
                        if (tmp->edge->getEndPoint(FirstPoint) == vertex3) {
                            tmp = tmp->edge->getEdgePointer(EdgeLeft);
                        } else {
                            tmp = tmp->edge->getEdgePointer(EdgeRight);
                        }
                        edgePointer13.push_back(tmp);
                        edge13.push_back(tmp->edge);
                        if (tmp->edge->getPolygon(EdgeLeft)  == polygon1 ||
                            tmp->edge->getPolygon(EdgeRight) == polygon1)
                            break;
                        tmp = tmp->next;
                    }
                }
#ifdef DEBUG
                cout << "Wings:" << endl;
                cout << "  Edge pointers between polygon 1 and 2:" << endl;
                cout << "    number: " << edgePointer12.size() << endl;
                cout << "  Edges between polygon 1 and 2:" << endl;
                for (int j = 0; j < edge12.size(); ++j)
                    cout << edge12[j]->getID() << "  ";
                cout << endl;
                cout << "  Edge pointers between polygon 1 and 3:" << endl;
                cout << "    number: " << edgePointer13.size() << endl;
                cout << "  Edges between polygon 1 and 3:" << endl;
                for (int j = 0; j < edge13.size(); ++j)
                    cout << edge13[j]->getID() << "  ";
                cout << endl;
#endif
            }
            // -----------------------------------------------------------------
            // create new edge
            Edge *newEdge;
            EdgePointer *edgePointer9 = NULL, *edgePointer10 = NULL;
            if (oldAngle < PI) {
                if (polygon1 != NULL || polygon2 != NULL) {
                    polygonManager.edges.append(&newEdge);
                    newEdge->linkEndPoint(FirstPoint, vertex1);
                    newEdge->linkEndPoint(SecondPoint, newVertex);
                    newEdge->calcNormVector();
                    newEdge->calcLength();
                    testPoint = newEdge->getTestPoint();
                    meshManager.checkLocation(testPoint->getCoordinate(), loc);
                    testPoint->setLocation(loc);
                    track(meshManager, flowManager, testPoint);
                    if (polygon1 != NULL) {
                        newEdge->setPolygon(EdgeLeft, polygon1);
                        if (length1 > length2) {
                            polygon1->edgePointers.insert(&edgePointer9, edgePointer5);
                            newEdge->setEdgePointer(EdgeLeft, edgePointer9);
                        } else {
                            edgePointer5->changeEdge(newEdge, EdgeLeft);
                        }
                    }
                    if (polygon2 != NULL) {
                        newEdge->setPolygon(EdgeRight, polygon2);
                        if (length1 > length2) {
                            edgePointer6->changeEdge(newEdge, EdgeRight);
                        } else {
                            polygon2->edgePointers.insert(edgePointer6, &edgePointer10);
                            newEdge->setEdgePointer(EdgeRight, edgePointer10);
                        }
                    }
                } else {
                    polygonManager.vertices.remove(vertex1);
                }
            } else {
                polygonManager.vertices.remove(vertex1);
            }
            // -----------------------------------------------------------------
            // remove or modify edge1 and edge2
            if (oldAngle < PI) {
                if (length1 > length2) {
                    polygonManager.edges.remove(edge2);
                    polygon->edgePointers.remove(edgePointer2);
                    if (vertex3 == edge1->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge1, SecondPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge1, FirstPoint, newVertex,
                                           meshManager, flowManager);
                } else {
                    polygonManager.edges.remove(edge1);
                    polygon->edgePointers.remove(edgePointer1);
                    if (vertex2 == edge2->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge2, SecondPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge2, FirstPoint, newVertex,
                                           meshManager, flowManager);
                }
            } else {
                if (vertex3 == edge1->getEndPoint(FirstPoint))
                    changeEdgeEndPoint(edge1, SecondPoint, newVertex,
                                       meshManager, flowManager);
                else
                    changeEdgeEndPoint(edge1, FirstPoint, newVertex,
                                       meshManager, flowManager);
                if (vertex2 == edge2->getEndPoint(FirstPoint))
                    changeEdgeEndPoint(edge2, SecondPoint, newVertex,
                                       meshManager, flowManager);
                else
                    changeEdgeEndPoint(edge2, FirstPoint, newVertex,
                                       meshManager, flowManager);
            }
            // -----------------------------------------------------------------
            // modify edge3 and edge4
            if (length1 > length2)
                if (vertex2 == edge4->getEndPoint(FirstPoint))
                    changeEdgeEndPoint(edge4, FirstPoint, newVertex,
                                       meshManager, flowManager);
                else
                    changeEdgeEndPoint(edge4, SecondPoint, newVertex,
                                       meshManager, flowManager);
            else
                if (vertex3 == edge3->getEndPoint(FirstPoint))
                    changeEdgeEndPoint(edge3, FirstPoint, newVertex,
                                       meshManager, flowManager);
                else
                    changeEdgeEndPoint(edge3, SecondPoint, newVertex,
                                       meshManager, flowManager);
            // -----------------------------------------------------------------
            // modify wings to link newVertex
            if (oldAngle < PI) {
#ifdef DEBUG
                if (length1 > length2)
                    assert(edge13.size() == 0);
                else
                    assert(edge24.size() == 0);
#endif
                for (int j = 0; j < edge13.size(); ++j)
                    if (vertex3 == edge13[j]->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge13[j], FirstPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge13[j], SecondPoint, newVertex,
                                           meshManager, flowManager);
                for (int j = 0; j < edge24.size(); ++j)
                    if (vertex2 == edge24[j]->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge24[j], FirstPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge24[j], SecondPoint, newVertex,
                                           meshManager, flowManager);
            } else {
#ifdef DEBUG
                if (length1 > length2)
                    assert(edge13.size() == 0);
                else
                    assert(edge12.size() == 0);
#endif
                for (int j = 0; j < edge12.size(); ++j)
                    if (vertex2 == edge12[j]->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge12[j], FirstPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge12[j], SecondPoint, newVertex,
                                           meshManager, flowManager);
                for (int j = 0; j < edge13.size(); ++j)
                    if (vertex3 == edge13[j]->getEndPoint(FirstPoint))
                        changeEdgeEndPoint(edge13[j], FirstPoint, newVertex,
                                           meshManager, flowManager);
                    else
                        changeEdgeEndPoint(edge13[j], SecondPoint, newVertex,
                                           meshManager, flowManager);
            }
            // -----------------------------------------------------------------
            // update the angles
            if (oldAngle < PI) {
                // -------------------------------------------------------------
                // left wing
                if (length1 > length2) {
                    if (polygon1 != NULL) {
#ifdef DEBUG
                        assert(edgePointer9->isAngleSet == false);
                        assert(edgePointer5->isAngleSet == false);
                        assert(edgePointer5->next->isAngleSet == false);
#endif
                        edgePointer9->calcAngle();
                        edgePointer5->calcAngle();
                        edgePointer5->next->calcAngle();
                    }
                } else {
                    if (polygon1 != NULL) {
#ifdef DEBUG
                        assert(edgePointer5->isAngleSet == false);
#endif
                        edgePointer5->calcAngle();
                    }
                    for (int j = 0; j < edgePointer13.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer13[j]->isAngleSet == false);
                        assert(edgePointer13[j]->next->isAngleSet == false);
#endif
                        edgePointer13[j]->calcAngle();
                        edgePointer13[j]->next->calcAngle();
                    }
                    for (int j = 1; j < edgePointer13.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer13[j]->isAngleSet == false);
#endif
                        edgePointer13[j]->calcAngle();
                    }
                    if (polygon3 != NULL) {
#ifdef DEBUG
                        assert(edgePointer7->isAngleSet == false);
                        assert(edgePointer7->next->isAngleSet == false);
#endif
                        edgePointer7->calcAngle();
                        edgePointer7->next->calcAngle();
                    }
                }
                // -------------------------------------------------------------
                // right wing
                if (length1 > length2) {
                    if (polygon2 != NULL) {
#ifdef DEBUG
                        assert(edgePointer6->next->isAngleSet == false);
#endif
                        edgePointer6->next->calcAngle();
                    }
                    for (int j = 0; j < edgePointer24.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer24[j]->isAngleSet == false);
                        assert(edgePointer24[j]->next->isAngleSet == false);
#endif
                        edgePointer24[j]->calcAngle();
                        edgePointer24[j]->next->calcAngle();
                    }
                    for (int j = 1; j < edgePointer24.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer24[j]->next->isAngleSet == false);
#endif
                        edgePointer24[j]->next->calcAngle();
                    }
                    if (polygon4 != NULL) {
#ifdef DEBUG
                        assert(edgePointer8->isAngleSet == false);
                        assert(edgePointer8->next->isAngleSet == false);
#endif
                        edgePointer8->calcAngle();
                        edgePointer8->next->calcAngle();
                    }
                } else {
                    if (polygon2 != NULL) {
#ifdef DEBUG
                        assert(edgePointer6->isAngleSet == false);
                        assert(edgePointer10->isAngleSet == false);
                        assert(edgePointer10->next->isAngleSet == false);
#endif
                        edgePointer6->calcAngle();
                        edgePointer10->calcAngle();
                        edgePointer10->next->calcAngle();
                    }
                }
                // -------------------------------------------------------------
                if (length1 > length2) {
#ifdef DEBUG
                    assert(edgePointer1->isAngleSet == false);
                    assert(edgePointer4->isAngleSet == false);
                    assert(edgePointer4->next->isAngleSet == false);
#endif
                    edgePointer1->calcAngle();
                    edgePointer4->calcAngle();
                    angleDiff = edgePointer4->getAngle(NewTimeLevel)-oldAngle;
                    if (fabs(angleDiff) > dA)
                        edgePointer4->angle.setOld(oldAngle);
                    edgePointer4->next->calcAngle();
                } else {
#ifdef DEBUG
                    assert(edgePointer3->isAngleSet == false);
                    assert(edgePointer2->isAngleSet == false);
                    assert(edgePointer2->next->isAngleSet == false);
#endif
                    edgePointer3->calcAngle();
                    edgePointer2->calcAngle();
                    angleDiff = edgePointer2->getAngle(NewTimeLevel)-oldAngle;
                    if (fabs(angleDiff) > dA)
                        edgePointer2->angle.setOld(oldAngle);
                    edgePointer2->next->calcAngle();
                }
            } else {
                // -------------------------------------------------------------
                // left wing
                if (length1 < length2) {
                    if (polygon3 != NULL) {
#ifdef DEBUG
                        assert(edgePointer7->isAngleSet == false);
                        assert(edgePointer7->next->isAngleSet == false);
#endif
                        edgePointer7->next->calcAngle();
                        edgePointer7->calcAngle();
                    }
                    for (int j = 0; j < edgePointer13.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer13[j]->isAngleSet == false);
#endif
                        edgePointer13[j]->calcAngle();
                    }
                    for (int j = 1; j < edgePointer13.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer13[j]->isAngleSet == false);
                        assert(edgePointer13[j]->next->isAngleSet == false);
#endif
                        edgePointer13[j]->calcAngle();
                        edgePointer13[j]->next->calcAngle();
                    }
                    if (polygon1 != NULL) {
#ifdef DEBUG
                        assert(edgePointer5->isAngleSet == false);
#endif
                        edgePointer5->calcAngle();
                    }
                }
                // -------------------------------------------------------------
                // right wing
                if (length1 > length2) {
                    if (polygon2 != NULL) {
#ifdef DEBUG
                        assert(edgePointer8->isAngleSet == false);
#endif
                        edgePointer8->calcAngle();
                    }
                    for (int j = 0; j < edgePointer12.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer12[j]->isAngleSet == false);
                        assert(edgePointer12[j]->next->isAngleSet == false);
#endif
                        edgePointer12[j]->calcAngle();
                        edgePointer12[j]->next->calcAngle();
                    }
                    for (int j = 1; j < edgePointer12.size(); j += 2) {
#ifdef DEBUG
                        assert(edgePointer12[j]->isAngleSet == false);
#endif
                        edgePointer12[j]->calcAngle();
                    }
                    if (polygon1 != NULL) {
#ifdef DEBUG
                        assert(edgePointer6->isAngleSet == false);
                        assert(edgePointer5->isAngleSet == false);
#endif
                        edgePointer6->calcAngle();
                        edgePointer5->calcAngle();
                    }
                }
                // -------------------------------------------------------------
#ifdef DEBUG
                assert(edgePointer2->isAngleSet == false);
#endif
                edgePointer2->calcAngle();
                angleDiff = edgePointer2->getAngle(NewTimeLevel)-oldAngle;
                if (fabs(angleDiff) > dA)
                    edgePointer2->angle.setOld(oldAngle);
                if (length1 > length2) {
#ifdef DEBUG
                    assert(edgePointer4->isAngleSet == false);
                    assert(edgePointer4->next->isAngleSet == false);
#endif
                    edgePointer4->calcAngle();
                    edgePointer4->next->calcAngle();
                } else {
#ifdef DEBUG
                    assert(edgePointer1->isAngleSet == false);
                    assert(edgePointer3->isAngleSet == false);
#endif
                    edgePointer1->calcAngle();
                    edgePointer3->calcAngle();
                }
            }
            // -----------------------------------------------------------------
            // check the "wrong" angle again
            if (oldAngle < PI) {
                if (length1 > length2)
                    edgePointer = edgePointer1;
                else
                    edgePointer = edgePointer3;
            } else {
                if (length1 > length2)
                    edgePointer = edgePointer1;
                else
                    edgePointer = edgePointer2;
            }
        }
        edgePointer = edgePointer->next;
    }

    return isHandled;
}
