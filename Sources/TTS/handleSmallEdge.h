bool TTS::handleSmallEdge(MeshManager &meshManager, const FlowManager &flowManager,
                          PolygonManager &polygonManager, Edge *edge)
{
    bool isHandled = false;
    static const double smallEdge = 0.01/Rad2Deg*Sphere::radius;
    static const double smallAngle = 20.0/Rad2Deg;

    if (edge->getLength() < smallEdge) {
#ifdef DEBUG
        REPORT_DEBUG
        cout << "Small edge length: " << edge->getLength()*Rad2Deg << endl;
#endif
        isHandled = true;
        Vertex *vertex1 = edge->getEndPoint(FirstPoint);
        Vertex *vertex2 = edge->getEndPoint(SecondPoint);
        Polygon *polygon1 = edge->getPolygon(EdgeLeft);
        Polygon *polygon2 = edge->getPolygon(EdgeRight);
        EdgePointer *edgePointer1 = edge->getEdgePointer(EdgeLeft);
        EdgePointer *edgePointer2 = edge->getEdgePointer(EdgeRight);
        if (edgePointer1->isTangly() || edgePointer2->isTangly())
            return false;
        EdgePointer *edgePointer3, *edgePointer4;
        Edge *edge1, *edge2;
        if (vertex1->isJoint() && vertex2->isJoint()) {
            REPORT_ERROR("How to handle this situation?")
        } else if (vertex1->isJoint()) {
            if (edgePointer1->getAngle() < smallAngle) {
                if (Sphere::orient(vertex1,
                                   edgePointer1->next->getEndPoint(SecondPoint),
                                   edgePointer1->prev->getEndPoint(FirstPoint))
                    == OrientRight)
                    return false;
            } else if (PI2-edgePointer1->getAngle() < smallAngle) {
                if (Sphere::orient(vertex1,
                                   edgePointer1->next->getEndPoint(SecondPoint),
                                   edgePointer1->prev->getEndPoint(FirstPoint))
                    == OrientLeft)
                    return false;
            }
            if (edgePointer2->next->getAngle() < smallAngle) {
                if (Sphere::orient(vertex1,
                                   edgePointer2->prev->getEndPoint(FirstPoint),
                                   edgePointer2->next->getEndPoint(SecondPoint))
                    == OrientLeft)
                    return false;
            } else if (PI2-edgePointer2->next->getAngle() < smallAngle) {
                if (Sphere::orient(vertex1,
                                   edgePointer2->prev->getEndPoint(FirstPoint),
                                   edgePointer2->next->getEndPoint(SecondPoint))
                    == OrientRight)
                    return false;
            }
            edgePointer3 = edgePointer1->next;
            edgePointer4 = edgePointer2->prev;
            polygon1->edgePointers.remove(edgePointer1);
            polygon2->edgePointers.remove(edgePointer2);
            edge1 = edgePointer3->edge;
            edge2 = edgePointer4->edge;
            vertex1->dislinkEdge(edge);
            if (vertex2 == edge1->getEndPoint(FirstPoint))
                changeEdgeEndPoint(edge1, FirstPoint, vertex1,
                                   meshManager, flowManager);
            else
                changeEdgeEndPoint(edge1, SecondPoint, vertex1,
                                   meshManager, flowManager);
            polygonManager.vertices.remove(vertex2);
        } else {
            if (edgePointer2->getAngle() < smallAngle) {
                if (Sphere::orient(vertex2,
                                   edgePointer2->next->getEndPoint(SecondPoint),
                                   edgePointer2->prev->getEndPoint(FirstPoint))
                    == OrientRight)
                    return false;
            } else if (PI2-edgePointer2->getAngle() < smallAngle) {
                if (Sphere::orient(vertex2,
                                   edgePointer2->next->getEndPoint(SecondPoint),
                                   edgePointer2->prev->getEndPoint(FirstPoint))
                    == OrientLeft)
                    return false;
            }
            if (edgePointer1->next->getAngle() < smallAngle) {
                if (Sphere::orient(vertex2,
                                   edgePointer1->prev->getEndPoint(FirstPoint),
                                   edgePointer1->next->getEndPoint(SecondPoint))
                    == OrientLeft)
                    return false;
            } else if (PI2-edgePointer1->next->getAngle() < smallAngle) {
                if (Sphere::orient(vertex2,
                                   edgePointer1->prev->getEndPoint(FirstPoint),
                                   edgePointer1->next->getEndPoint(SecondPoint))
                    == OrientRight)
                    return false;
            }
            edgePointer3 = edgePointer1->prev;
            edgePointer4 = edgePointer2->next;
            polygon1->edgePointers.remove(edgePointer1);
            polygon2->edgePointers.remove(edgePointer2);
            edge1 = edgePointer3->edge;
            edge2 = edgePointer4->edge;
            vertex2->dislinkEdge(edge);
            if (vertex1 == edge1->getEndPoint(FirstPoint))
                changeEdgeEndPoint(edge1, FirstPoint, vertex2,
                                   meshManager, flowManager);
            else
                changeEdgeEndPoint(edge1, SecondPoint, vertex2,
                                   meshManager, flowManager);
            polygonManager.vertices.remove(vertex1);
        }
#ifdef DEBUG
        assert(edge1 == edge2);
#endif
        // ---------------------------------------------------------------------
        polygonManager.edges.remove(edge);
        // ---------------------------------------------------------------------
        // update the angles
        edgePointer3->calcAngle();
        edgePointer3->next->calcAngle();
        edgePointer4->calcAngle();
        edgePointer4->next->calcAngle();
    }

    return isHandled;
}