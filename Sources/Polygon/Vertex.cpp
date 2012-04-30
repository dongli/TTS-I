#include "Vertex.hpp"
#include "Edge.hpp"
#include "Polygon.hpp"
#ifdef TTS_ONLINE
#include "MeshManager.hpp"
#include "FlowManager.hpp"
#include "PolygonManager.hpp"
#include "TimeManager.hpp"
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
    clean();
    linkedEdges.destroy();
}

void Vertex::reinit()
{
    Point::reinit();
    linkedEdges.recycle();
#ifdef TTS_ONLINE
    detectAgent.reinit();
    hostEdge = NULL;
#endif
}

void Vertex::clean()
{
    EdgePointer *linkedEdge = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        if (linkedEdge->edge->getEndPoint(FirstPoint) == this)
            linkedEdge->edge->linkEndPoint(FirstPoint, NULL);
        else if (linkedEdge->edge->getEndPoint(SecondPoint) == this)
            linkedEdge->edge->linkEndPoint(SecondPoint, NULL);
        else
            REPORT_ERROR("Unlinked edge!");
        linkedEdge = linkedEdge->next;
    }
    linkedEdges.recycle();
#ifdef TTS_ONLINE
    detectAgent.clean();
#endif
}

void Vertex::linkEdge(Edge *edge)
{
    linkedEdges.append();
    linkedEdges.back()->edge = edge;
}

void Vertex::unlinkEdge(Edge *edge)
{
    EdgePointer *linkedEdge = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        if (linkedEdge->edge == edge) {
            linkedEdges.remove(linkedEdge);
            return;
        }
        linkedEdge = linkedEdge->next;
    }
    cout << "The linked edges of vertex " << getID() << "  " << this << endl;
    linkedEdge = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        cout << i << ": " << linkedEdge->edge->getID() << endl;
        linkedEdge = linkedEdge->next;
    }
    cout << "The wanted edge is " << edge->getID() << endl;
    REPORT_ERROR("Edge is not linked with vertex.")
}

void Vertex::handoverEdges(Vertex *newVertex, PolygonManager &polygonManager)
{
    EdgePointer *linkedEdge = linkedEdges.front();
    while (linkedEdge != NULL) {
        if (linkedEdge->edge->getEndPoint(FirstPoint) == this) {
            linkedEdge->edge->changeEndPoint(FirstPoint, newVertex);
        } else if (linkedEdge->edge->getEndPoint(SecondPoint) == this) {
            linkedEdge->edge->changeEndPoint(SecondPoint, newVertex);
        } else
            REPORT_ERROR("Edge and end points are not inconsistent!");
        linkedEdge = linkedEdge->next;
    }
}

#ifdef TTS_ONLINE
void Vertex::handoverEdges(Vertex *newVertex, MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager)
{
    static std::list<Edge *> edges;
    // -------------------------------------------------------------------------
    EdgePointer *linkedEdge = linkedEdges.front();
    while (linkedEdge != NULL) {
        edges.push_back(linkedEdge->edge);
        if (linkedEdge->edge->getEndPoint(FirstPoint) == this) {
            linkedEdge->edge->changeEndPoint(FirstPoint, newVertex,
                                             meshManager, flowManager);
        } else {
            linkedEdge->edge->changeEndPoint(SecondPoint, newVertex,
                                             meshManager, flowManager);
        }
        linkedEdge = linkedEdge->next;
    }
    // -------------------------------------------------------------------------
    static std::list<Edge *>::iterator it;
    for (it = edges.begin(); it != edges.end(); ++it) {
        if ((*it)->getEndPoint(FirstPoint) ==
            (*it)->getEndPoint(SecondPoint) &&
            (*it)->detectAgent.vertices.size() != 0)
            (*it)->detectAgent.clean();
        else
            (*it)->detectAgent.updateVertexProjections(meshManager);
    }
    edges.clear();
}
#endif

Vertex &Vertex::operator=(const Vertex &that)
{
    if (this != &that) {
        Point::operator=(that);
#ifdef TTS_ONLINE
        // for detecting test points 2011-11-12
        detectAgent = that.detectAgent;
#endif
    }
    return *this;
}

void Vertex::dump(int indentLevel)
{
    cout << "Vertex " << getID() << ":" << endl;
    Point::dump(indentLevel+1);
    EdgePointer *linkedEdge = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        cout << "  Linked edge " << i << ":" << endl;
        cout << "    ID:              " << linkedEdge->edge->getID() << endl;
        cout << "    First point ID:  " <<
        linkedEdge->edge->getEndPoint(FirstPoint)->getID() << endl;
        cout << "    Second point ID: " <<
        linkedEdge->edge->getEndPoint(SecondPoint)->getID() << endl;
        cout << "    Left polygon ID: " <<
        linkedEdge->edge->getPolygon(OrientLeft)->getID() << endl;
        cout << "    Right polygon ID: " <<
        linkedEdge->edge->getPolygon(OrientRight)->getID() << endl;
        linkedEdge = linkedEdge->next;
    }
#ifdef TTS_ONLINE
    detectAgent.dump();
#endif
}