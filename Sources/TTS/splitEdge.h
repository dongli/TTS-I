bool TTS::splitEdge(MeshManager &meshManager, const FlowManager &flowManager,
                    PolygonManager &polygonManager, Edge *edge)
{
    static int level = 0;
    static const int maxLevel = 5;
    static const double smallAngle = 90.0/Rad2Deg;
    static Edge *edge0;
    static Polygon *polygon1; // for OrientLeft polygon
    static Polygon *polygon2; // for OrientRight polygon
    static EdgePointer *edgePointer1; // for OrientLeft polygon
    static EdgePointer *edgePointer2; // for OrientRight polygon
    static EdgePointer *newEdgePointer1;
    static EdgePointer *newEdgePointer2;
    static EdgePointer *edgePointer1_prev;
    static EdgePointer *edgePointer1_next;
    static EdgePointer *edgePointer2_prev;
    static EdgePointer *edgePointer2_next;
    static double angleCheck[4];
    Location loc;

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
    double angle = EdgePointer::calcAngle(vector1, vector2, *testPoint);
    if (fabs(angle-PI) > a0) {
        // ---------------------------------------------------------------------
        // record the polygons and edge pointers, and reset the tasks
        if (level == 1) {
            edge0 = edge;
            polygon1 = edge->getPolygon(OrientLeft);
            polygon2 = edge->getPolygon(OrientRight);
            edgePointer1 = edge->getEdgePointer(OrientLeft);
            edgePointer2 = edge->getEdgePointer(OrientRight);
            edgePointer1_prev = edgePointer1->prev;
            edgePointer1_next = edgePointer1->next;
            edgePointer2_prev = edgePointer2->prev;
            edgePointer2_next = edgePointer2->next;
            angleCheck[0] = edgePointer1->getAngle();
            angleCheck[1] = edgePointer1_next->getAngle();
            angleCheck[2] = edgePointer2->getAngle();
            angleCheck[3] = edgePointer2_next->getAngle();
#ifdef TEST_NEW_FEATURE
            resetTasks();
#endif
        }

        // ---------------------------------------------------------------------
        // several checks if the splitting is ok
        if (level > maxLevel) {
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
        track(meshManager, flowManager, testPoint1);
        track(meshManager, flowManager, testPoint2);

        // ---------------------------------------------------------------------
        // check in next level
        if (!splitEdge(meshManager, flowManager, polygonManager, &edge1)) {
            // the edge has not been splitted, add the edge into the main
            // data structure
            Edge *newEdge;
            polygonManager.edges.append(&newEdge);
            *newEdge = edge1;
            vertex1->dislinkEdge(&edge1);
            vertex1->linkEdge(newEdge);
            newVertex->dislinkEdge(&edge1);
            newVertex->linkEdge(newEdge);
            newEdge->calcNormVector();
            newEdge->calcLength();
            // -----------------------------------------------------------------
            EdgePointer *newEdgePointer;
            // left polygon:
            if (polygon1 != NULL) {
                if (edgePointer1 != NULL) {
                    polygon1->edgePointers.insert(edgePointer1, &newEdgePointer);
                    polygon1->edgePointers.remove(edgePointer1);
                    edgePointer1 = NULL;
                } else {
                    polygon1->edgePointers.insert(newEdgePointer1, &newEdgePointer);
                }
                newEdge->setPolygon(OrientLeft, polygon1);
                newEdge->setEdgePointer(OrientLeft, newEdgePointer);
                newEdgePointer1 = newEdgePointer;
            }
            // right polygon:
            if (polygon2 != NULL) {
                if (edgePointer2 != NULL) {
                    polygon2->edgePointers.insert(&newEdgePointer, edgePointer2);
                    polygon2->edgePointers.remove(edgePointer2);
                    edgePointer2 = NULL;
                } else {
                    polygon2->edgePointers.insert(&newEdgePointer, newEdgePointer2);
                }
                newEdge->setPolygon(OrientRight, polygon2);
                newEdge->setEdgePointer(OrientRight, newEdgePointer);
                newEdgePointer2 = newEdgePointer;
            }
            // -----------------------------------------------------------------
            edge0->detectAgent.handover(newEdge);
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
            newEdge->calcNormVector();
            newEdge->calcLength();
            // -----------------------------------------------------------------
            EdgePointer *newEdgePointer;
            // left polygon:
            if (polygon1 != NULL) {
                polygon1->edgePointers.insert(newEdgePointer1, &newEdgePointer);
                newEdge->setPolygon(OrientLeft, polygon1);
                newEdge->setEdgePointer(OrientLeft, newEdgePointer);
                newEdgePointer1 = newEdgePointer;
            }
            // right polygon:
            if (polygon2 != NULL) {
                polygon2->edgePointers.insert(&newEdgePointer, newEdgePointer2);
                newEdge->setPolygon(OrientRight, polygon2);
                newEdge->setEdgePointer(OrientRight, newEdgePointer);
                newEdgePointer2 = newEdgePointer;
            }
            // -----------------------------------------------------------------
            edge0->detectAgent.handover(newEdge);
        }
        level--;
        if (level == 0) {
            polygonManager.edges.remove(edge);
#ifdef TEST_NEW_FEATURE
            doTask(UpdateAngle);
#else
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
#endif
        }
        return true;
    } else {
        if (level > 0) level--;
        return false;
    }
}