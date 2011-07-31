bool TTS::mergeEdge(MeshManager &meshManager, const FlowManager &flowManager,
                    PolygonManager &polygonManager, Polygon *polygon)
{
    static const double angleCheck = 20.0/Rad2Deg;
    bool isMerged = false;

    EdgePointer *edgePointer = polygon->edgePointers.front();
    for (int i = 0; i < polygon->edgePointers.size(); ++i) {
        Edge *edge1 = edgePointer->prev->edge; // to be deleted if necessary
        Edge *edge2 = edgePointer->edge;
        double a0 = angleThreshold(edge1, edge2)*0.95;
        if (fabs(edgePointer->getAngle(OldTimeLevel)-PI) < a0 &&
            fabs(edgePointer->getAngle(NewTimeLevel)-PI) < a0) {
            isMerged = true;
#ifdef DEBUG_MERGEEDGE
            cout << fabs(edgePointer->getAngle(OldTimeLevel)-PI)*Rad2Deg << endl;
            cout << fabs(edgePointer->getAngle(NewTimeLevel)-PI)*Rad2Deg << endl;
            cout << a0*Rad2Deg << endl;
            edgePointer->dump();
            edgePointer->prev->dump();
#endif
            // -----------------------------------------------------------------
            Vertex *vertex1, *vertex2, *vertex3;  // vertex2 is to be deleted
            vertex1 = edgePointer->prev->getEndPoint(FirstPoint);
            vertex2 = edgePointer->getEndPoint(FirstPoint);
            vertex3 = edgePointer->getEndPoint(SecondPoint);
#ifdef DEBUG_MERGEEDGE
            vertex2->dump();
#endif
            // Note: If the vertex is connected by more than two edges, that is
            //       a joint, then do not merge the edges.
            if (vertex2->isJoint())
                return false;
#ifdef DEBUG_MERGEEDGE
            cout << "vertex 1: " << vertex1->getID() << endl;
            cout << "vertex 2: " << vertex2->getID() << endl;
            cout << "vertex 3: " << vertex3->getID() << endl;
#endif
            // -----------------------------------------------------------------
            EdgePointer *oldEdgePointer1, *oldEdgePointer2;
            if (vertex2 == edge2->getEndPoint(FirstPoint)) {
                oldEdgePointer1 = edge2->getEdgePointer(EdgeLeft);
                oldEdgePointer2 = edge2->getEdgePointer(EdgeRight);
            } else {
                oldEdgePointer1 = edge2->getEdgePointer(EdgeRight);
                oldEdgePointer2 = edge2->getEdgePointer(EdgeLeft);
            }            
#ifdef DEBUG
            assert(oldEdgePointer1 == edgePointer);
#endif
            // -----------------------------------------------------------------
            if (oldEdgePointer1->next->getAngle() < angleCheck)
                return false;
            if (oldEdgePointer1->prev->getAngle() < angleCheck)
                return false;
            if (oldEdgePointer2->getAngle() < angleCheck)
                return false;
            if (oldEdgePointer2->next->next->getAngle() < angleCheck)
                return false;
            // -----------------------------------------------------------------
            if (oldEdgePointer1->isTangly() ||
                oldEdgePointer2->next->isTangly()) {
                edgePointer = edgePointer->next;
                continue;
            }
            // -----------------------------------------------------------------
            Polygon *polygon1, *polygon2;
            if (vertex1 == edge1->getEndPoint(FirstPoint)) {
                polygon1 = edge1->getPolygon(EdgeLeft);
                polygon2 = edge1->getPolygon(EdgeRight);
            } else {
                polygon1 = edge1->getPolygon(EdgeRight);
                polygon2 = edge1->getPolygon(EdgeLeft);
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
            vertex3->dislinkEdge(edge2);
            polygonManager.vertices.remove(vertex2);
            // -----------------------------------------------------------------
            // adjust edges
            if (vertex1 == edge1->getEndPoint(FirstPoint)) {
                changeEdgeEndPoint(edge1, SecondPoint, vertex3,
                                   meshManager, flowManager);
                edge1->setEdgePointer(EdgeLeft, newEdgePointer1);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(EdgeRight, newEdgePointer2);
            } else {
                changeEdgeEndPoint(edge1, FirstPoint, vertex3,
                                   meshManager, flowManager);
                edge1->setEdgePointer(EdgeRight, newEdgePointer1);
                if (newEdgePointer2 != NULL)
                    edge1->setEdgePointer(EdgeLeft, newEdgePointer2);
            }
            polygonManager.edges.remove(edge2);
            // -----------------------------------------------------------------
            // update the angles
            newEdgePointer1->calcAngle();
            newEdgePointer1->next->calcAngle();
            if (newEdgePointer2 != NULL) {
                newEdgePointer2->calcAngle();
                newEdgePointer2->next->calcAngle();
            }
        }
        edgePointer = edgePointer->next;
    }

    return isMerged;
}