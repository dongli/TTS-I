#include "Vertex.h"
#include "Edge.h"

Vertex::Vertex()
{
    reinit();
}

Vertex::~Vertex()
{
}

void Vertex::reinit()
{
    Point::reinit();
    edgePointers.recycle();
}

void Vertex::linkEdge(Edge *edge)
{
    edgePointers.append();
    edgePointers.back()->edge = edge;
}

void Vertex::dislinkEdge(Edge *edge)
{
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        if (edgePointer->edge == edge) {
            edgePointers.remove(edgePointer);
            return;
        }
        edgePointer = edgePointer->next;
    }
    cout << "The linked edges by vertex " << getID() << "  " << this << endl;
    edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        cout << i << ": " << edgePointer->edge->getID() << endl;
        edgePointer = edgePointer->next;
    }
    cout << "The wanted edge is " << edge->getID() << endl;
    REPORT_ERROR("Edge is not linked with vertex.")
}

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
    EdgePointer *edgePointer = edgePointers.front();
    for (int i = 0; i < edgePointers.size(); ++i) {
        cout << "  Linked Edge " << i << ":" << endl;
        cout << "    ID:              " << edgePointer->edge->getID() << endl;
        cout << "    First Point ID:  " <<
        edgePointer->edge->getEndPoint(FirstPoint)->getID() << endl;
        cout << "    Second Point ID: " <<
        edgePointer->edge->getEndPoint(SecondPoint)->getID() << endl;
        edgePointer = edgePointer->next;
    }
}