#include "Vertex.h"
#include "Edge.h"
#include "Polygon.h"
#ifdef TTS_ONLINE
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#endif

Vertex::Vertex()
{
#ifdef TTS_ONLINE
    detectAgent.checkin(this);
#endif
    reinit();
}

Vertex::~Vertex()
{
    linkedEdges.destroy();
}

void Vertex::reinit()
{
    Point::reinit();
    linkedEdges.recycle();
#ifdef TTS_ONLINE
    detectAgent.reinit();
#endif
}

void Vertex::clean()
{
#ifdef TTS_ONLINE
    detectAgent.clean();
#endif
}

void Vertex::linkEdge(Edge *edge)
{
    linkedEdges.append();
    linkedEdges.back()->edge = edge;
}

void Vertex::dislinkEdge(Edge *edge)
{
    EdgePointer *edgePointer = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        if (edgePointer->edge == edge) {
            linkedEdges.remove(edgePointer);
            return;
        }
        edgePointer = edgePointer->next;
    }
    cout << "The linked edges by vertex " << getID() << "  " << this << endl;
    edgePointer = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        cout << i << ": " << edgePointer->edge->getID() << endl;
        edgePointer = edgePointer->next;
    }
    cout << "The wanted edge is " << edge->getID() << endl;
    REPORT_ERROR("Edge is not linked with vertex.")
}

#ifdef TTS_ONLINE
void Vertex::handoverEdges(Vertex *newVertex, MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager)
{
    EdgePointer *edgePointer = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        if (edgePointer->edge->getEndPoint(FirstPoint) == this) {
            edgePointer->edge->changeEndPoint(FirstPoint, newVertex,
                                              meshManager, flowManager);
        } else {
            edgePointer->edge->changeEndPoint(SecondPoint, newVertex,
                                              meshManager, flowManager);
        }
        if (edgePointer->edge->getEndPoint(FirstPoint) ==
            edgePointer->edge->getEndPoint(SecondPoint) &&
            edgePointer->edge->detectAgent.vertexPointers.size() != 0) {
            EdgePointer *linkedEdge = newVertex->linkedEdges.front();
            for (int j = 0; j < newVertex->linkedEdges.size(); ++j) {
                edgePointer->edge->detectAgent.handover(linkedEdge->edge);
                linkedEdge = linkedEdge->next;
            }
            if (edgePointer->edge->detectAgent.vertexPointers.size() != 0) {
                edgePointer->edge->detectAgent.clean();
                REPORT_DEBUG
            }
        }
        edgePointer->edge->detectAgent.update();
        edgePointer = edgePointer->next;
    }
}
#endif

Vertex &Vertex::operator=(const Vertex &that)
{
    if (this != &that) {
        Point::operator=(that);
    }
    return *this;
}

void Vertex::dump(int indentLevel) const
{
    cout << "Vertex " << getID() << ":" << endl;
    Point::dump(indentLevel+1);
    EdgePointer *edgePointer = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        cout << "  Linked edge " << i << ":" << endl;
        cout << "    ID:              " << edgePointer->edge->getID() << endl;
        cout << "    First point ID:  " <<
        edgePointer->edge->getEndPoint(FirstPoint)->getID() << endl;
        cout << "    Second point ID: " <<
        edgePointer->edge->getEndPoint(SecondPoint)->getID() << endl;
        cout << "    Left polygon ID: " <<
        edgePointer->edge->getPolygon(OrientLeft)->getID() << endl;
        cout << "    Right polygon ID: " <<
        edgePointer->edge->getPolygon(OrientRight)->getID() << endl;
        edgePointer = edgePointer->next;
    }
}

// -----------------------------------------------------------------------------

VertexPointer::VertexPointer()
{
    reinit();
}

VertexPointer::~VertexPointer()
{
}

void VertexPointer::reinit()
{
    vertex = NULL;
}