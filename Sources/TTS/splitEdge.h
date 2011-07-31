bool TTS::splitEdge(MeshManager &meshManager, const FlowManager &flowManager,
                    PolygonManager &polygonManager, Edge *edge)
{
    static int level = 0;
    static const int maxLevel = 3;
    static const double smallAngle = 20.0/Rad2Deg;
    static const double smallLength = 0.01/Rad2Deg;
    static Edge *edge0;
    static Polygon *polygon1; // for EdgeLeft polygon
    static Polygon *polygon2; // for EdgeRight polygon
    static EdgePointer *edgePointer1; // for EdgeLeft polygon
    static EdgePointer *edgePointer2; // for EdgeRight polygon
    static EdgePointer *edgePointer1_prev;
    static EdgePointer *edgePointer1_next;
    static EdgePointer *edgePointer2_prev;
    static EdgePointer *edgePointer2_next;
    static double angleCheck[4];

    level++;

    Vertex *vertex1 = edge->getEndPoint(FirstPoint);
    Vertex *testPoint = edge->getTestPoint();
    Vertex *vertex2 = edge->getEndPoint(SecondPoint);

    const Coordinate &x1 = vertex1->getCoordinate(NewTimeLevel);
    const Coordinate &x2 = testPoint->getCoordinate(NewTimeLevel);
    const Coordinate &x3 = vertex2->getCoordinate(NewTimeLevel);
    
    Vector vector1 = cross(x2.getCAR(), x1.getCAR());
    Vector vector2 = cross(x3.getCAR(), x2.getCAR());
    vector1 /= norm(vector1);
    vector2 /= norm(vector2);
    double a0 = angleThreshold(edge);
    double angle = EdgePointer::calcAngle(vector1, vector2, *(testPoint));
    if (fabs(angle-PI) > a0) {
#ifdef DEBUG_SPLITEDGE
        cout << fabs(angle-PI)*Rad2Deg << endl;
        cout << angleThreshold(edge)*Rad2Deg << endl;
        cout << "Level " << level << " to be splitted edge:" << endl;
        edge->dump();
#endif
        // ---------------------------------------------------------------------
        // record the polygons and edge pointers
        if (level == 1) {
            edge0 = edge;
            polygon1 = edge->getPolygon(EdgeLeft);
            polygon2 = edge->getPolygon(EdgeRight);
            edgePointer1 = edge->getEdgePointer(EdgeLeft);
            edgePointer2 = edge->getEdgePointer(EdgeRight);
            edgePointer1_prev = edgePointer1->prev;
            edgePointer1_next = edgePointer1->next;
            edgePointer2_prev = edgePointer2->prev;
            edgePointer2_next = edgePointer2->next;
            angleCheck[0] = edgePointer1->getAngle();
            angleCheck[1] = edgePointer1_next->getAngle();
            angleCheck[2] = edgePointer2->getAngle();
            angleCheck[3] = edgePointer2_next->getAngle();
        }

        // ---------------------------------------------------------------------
        // several checks if the splitting is ok
        if (level > maxLevel) {
            level--;
            return false;
        }
        if (vertex1->getLocation().isOnPole() ||
            vertex2->getLocation().isOnPole() ||
            testPoint->getLocation().isOnPole()) {
            level--;
            return false;
        }
        if (level == 1) {
            if (edgePointer1->isTangly() || edgePointer2->isTangly()) {
                level--;
                return false;
            }
        }
        // check if the insertion of test point causes edge tangling
        Vertex *p1[4] = {edgePointer1_prev->getEndPoint(FirstPoint),
                         edgePointer1_next->getEndPoint(FirstPoint),
                         edgePointer2_prev->getEndPoint(FirstPoint),
                         edgePointer2_next->getEndPoint(FirstPoint)};
        Vertex *p2[4] = {edgePointer1_prev->getEndPoint(SecondPoint),
                         edgePointer1_next->getEndPoint(SecondPoint),
                         edgePointer2_prev->getEndPoint(SecondPoint),
                         edgePointer2_next->getEndPoint(SecondPoint)};
        for (int i = 0; i < 4; ++i) {
            if (angleCheck[i] < smallAngle) {
                if (Sphere::orient(p1[i]->getCoordinate(OldTimeLevel),
                                   p2[i]->getCoordinate(OldTimeLevel),
                                   testPoint->getCoordinate(OldTimeLevel))
                    == OrientRight ||
                    Sphere::orient(p1[i]->getCoordinate(NewTimeLevel),
                                   p2[i]->getCoordinate(NewTimeLevel),
                                   testPoint->getCoordinate(NewTimeLevel))
                    == OrientRight) {
                    level--;
                    return false;
                }
            } else if (PI2-angleCheck[i] < smallAngle) {
                if (Sphere::orient(p1[i]->getCoordinate(OldTimeLevel),
                                   p2[i]->getCoordinate(OldTimeLevel),
                                   testPoint->getCoordinate(OldTimeLevel))
                    == OrientLeft ||
                    Sphere::orient(p1[i]->getCoordinate(NewTimeLevel),
                                   p2[i]->getCoordinate(NewTimeLevel),
                                   testPoint->getCoordinate(NewTimeLevel))
                    == OrientLeft) {
                    level--;
                    return false;
                }
            }
        }

        // ---------------------------------------------------------------------
        // add test point into the main data structure
        Vertex *newVertex;
        polygonManager.vertices.append(&newVertex);
        *newVertex = *testPoint;
#ifdef DEBUG_SPLITEDGE
        newVertex->dump();
#endif

        // ---------------------------------------------------------------------
        Edge edge1, edge2; // temporary edges

        // ---------------------------------------------------------------------
        // link end points
        vertex1->dislinkEdge(edge);
        vertex2->dislinkEdge(edge);
        edge1.linkEndPoint(FirstPoint, vertex1);
        edge1.linkEndPoint(SecondPoint, newVertex);
        edge2.linkEndPoint(FirstPoint, newVertex);
        edge2.linkEndPoint(SecondPoint, vertex2);
#ifdef DEBUG_SPLITEDGE
        newVertex->dump();
#endif

        // ---------------------------------------------------------------------
        // advect new test points
        Vertex *testPoint1 = edge1.getTestPoint();
        Vertex *testPoint2 = edge2.getTestPoint();
        Location loc;
        meshManager.checkLocation(testPoint1->getCoordinate(), loc);
        testPoint1->setLocation(loc);
        meshManager.checkLocation(testPoint2->getCoordinate(), loc);
        testPoint2->setLocation(loc);
        track(meshManager, flowManager, testPoint1);
        track(meshManager, flowManager, testPoint2);

        // ---------------------------------------------------------------------
        // check in next level
        if (!splitEdge(meshManager, flowManager, polygonManager, &edge1)) {
            // the edge has not been splitted, add the edge into the main
            // data structure
            Edge *newEdge;
            if (edge0 != NULL) {
                // when this is the first split, use the old edge
                newEdge = edge0;
            } else {
                polygonManager.edges.append(&newEdge);
            }
            *newEdge = edge1;
            vertex1->dislinkEdge(&edge1);
            vertex1->linkEdge(newEdge);
            newVertex->dislinkEdge(&edge1);
            newVertex->linkEdge(newEdge);
#ifdef DEBUG_SPLITEDGE
            newVertex->dump();
#endif
            newEdge->calcNormVector();
            newEdge->calcLength();
            EdgePointer *newEdgePointer;
            // -----------------------------------------------------------------
            // left polygon:
            if (polygon1 != NULL) {
                if (edge0 != NULL) {
                    // when this is the first split, use the old edge pointer
                    newEdgePointer = edgePointer1;
                    newEdgePointer->resetAngle();
                } else {
                    polygon1->edgePointers.insert(edgePointer1, &newEdgePointer);
                }
                newEdge->setPolygon(EdgeLeft, polygon1);
                newEdge->setEdgePointer(EdgeLeft, newEdgePointer);
                edgePointer1 = newEdgePointer;
            }
            // -----------------------------------------------------------------
            // right polygon:
            if (polygon2 != NULL) {
                if (edge0 != NULL) {
                    // when this is the first split, use the old edge pointer
                    newEdgePointer = edgePointer2;
                    newEdgePointer->resetAngle();
                } else {
                    polygon2->edgePointers.insert(&newEdgePointer, edgePointer2);
                }
                newEdge->setPolygon(EdgeRight, polygon2);
                newEdge->setEdgePointer(EdgeRight, newEdgePointer);
                edgePointer2 = newEdgePointer;
            }
            if (edge0 != NULL) edge0 = NULL;
#ifdef DEBUG_SPLITEDGE
            cout << "New edge:" << endl;
            newEdge->dump();
#endif
        }
        if (!splitEdge(meshManager, flowManager, polygonManager, &edge2)) {
            // the edge has not been splitted, add the edge into the main
            // data structure
            Edge *newEdge;
            polygonManager.edges.append(&newEdge);
            *newEdge = edge2;
            vertex2->dislinkEdge(&edge2);
            vertex2->linkEdge(newEdge);
            newVertex->dislinkEdge(&edge2);
            newVertex->linkEdge(newEdge);
#ifdef DEBUG_SPLITEDGE
            newVertex->dump();
#endif
            
            newEdge->calcNormVector();
            newEdge->calcLength();
            EdgePointer *newEdgePointer;
            // -----------------------------------------------------------------
            // left polygon:
            if (polygon1 != NULL) {
                polygon1->edgePointers.insert(edgePointer1, &newEdgePointer);
                newEdge->setPolygon(EdgeLeft, polygon1);
                newEdge->setEdgePointer(EdgeLeft, newEdgePointer);
                edgePointer1 = newEdgePointer;
            }
            // -----------------------------------------------------------------
            // right polygon:
            if (polygon2 != NULL) {
                polygon2->edgePointers.insert(&newEdgePointer, edgePointer2);
                newEdge->setPolygon(EdgeRight, polygon2);
                newEdge->setEdgePointer(EdgeRight, newEdgePointer);
                edgePointer2 = newEdgePointer;
            }
#ifdef DEBUG_SPLITEDGE
            cout << "New edge:" << endl;
            newEdge->dump();
#endif
        }
        level--;
        if (level == 0) {
            // update the angles
            EdgePointer *edgePointer;
            edgePointer = edgePointer1_prev->next;
            do {
                edgePointer->calcAngle();
                edgePointer = edgePointer->next;
            } while (edgePointer != edgePointer1_next->next);
            edgePointer = edgePointer2_prev->next;
            do {
                edgePointer->calcAngle();
                edgePointer = edgePointer->next;
            } while (edgePointer != edgePointer2_next->next);
#ifdef DEBUG_SPLITEDGE
            cout << "************** quit splitEdge" << endl;
#endif
        }
        return true;
    } else {
        if (level > 0) level--;
        return false;
    }
}