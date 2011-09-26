#include "Vertex.h"
#include "Edge.h"
#include "Polygon.h"
#ifdef TTS_ONLINE
#include "MeshManager.h"
#include "FlowManager.h"
#include "PolygonManager.h"
#include "TimeManager.h"
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
#ifdef TTS_ONLINE
    detectAgent.reinit();
#endif
}

void Vertex::clean()
{
    //if (TimeManager::getSteps() >= 45 && getID() == 185239)
    //    REPORT_DEBUG
    EdgePointer *linkedEdge = linkedEdges.front();
    for (int i = 0; i < linkedEdges.size(); ++i) {
        if (linkedEdge->edge->getEndPoint(FirstPoint) == this)
            linkedEdge->edge->linkEndPoint(FirstPoint, NULL);
        else if (linkedEdge->edge->getEndPoint(SecondPoint) == this)
            linkedEdge->edge->linkEndPoint(SecondPoint, NULL);
        else
            REPORT_ERROR("Unlinked edge!")
        linkedEdge = linkedEdge->next;
    }
    linkedEdges.recycle();
#ifdef TTS_ONLINE
    detectAgent.clean();
#endif
}

void Vertex::linkEdge(Edge *edge)
{
    //if (TimeManager::getSteps() >= 45 && getID() == 185239)
    //    REPORT_DEBUG
    linkedEdges.append();
    linkedEdges.back()->edge = edge;
}

void Vertex::unlinkEdge(Edge *edge)
{
    //if (TimeManager::getSteps() >= 45 && getID() == 185239)
    //    REPORT_DEBUG
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

#ifdef TTS_ONLINE
void Vertex::handoverEdges(Vertex *newVertex, MeshManager &meshManager,
                           const FlowManager &flowManager,
                           PolygonManager &polygonManager)
{
    EdgePointer *linkedEdge = linkedEdges.front();
    while (linkedEdge != NULL) {
        if (linkedEdge->edge->getEndPoint(FirstPoint) == this) {
            linkedEdge->edge->changeEndPoint(FirstPoint, newVertex,
                                              meshManager, flowManager);
        } else {
            linkedEdge->edge->changeEndPoint(SecondPoint, newVertex,
                                              meshManager, flowManager);
        }
        if (linkedEdge->edge->getEndPoint(FirstPoint) ==
            linkedEdge->edge->getEndPoint(SecondPoint) &&
            linkedEdge->edge->detectAgent.vertices.size() != 0) {
            EdgePointer *linkedEdge1 = newVertex->linkedEdges.front();
            for (int j = 0; j < newVertex->linkedEdges.size(); ++j) {
                linkedEdge->edge->detectAgent.handoverVertices(linkedEdge1->edge);
                linkedEdge1 = linkedEdge1->next;
            }
            if (linkedEdge->edge->detectAgent.vertices.size() != 0) {
                linkedEdge->edge->detectAgent.clean();
                REPORT_DEBUG
            }
        }
        linkedEdge->edge->detectAgent.updateVertexProjections();
        linkedEdge = linkedEdge->next;
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