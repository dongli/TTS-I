#include "ApproachingVertices.h"
#include "Vertex.h"
#include "Edge.h"

using namespace ApproachDetector;

std::list<Vertex *> ApproachingVertices::vertices;

//#define TRACK_APPROACH_VERTEX
#define APPROACH_VERTEX_ID 45804
#define APPROACH_TEST_POINT_HOST_EDGE_ID -999

ApproachingVertices::ApproachingVertices()
{
}

ApproachingVertices::~ApproachingVertices()
{
}

void ApproachingVertices::recordVertex(Vertex *vertex)
{
#ifdef TRACK_APPROACH_VERTEX
    if ((vertex->getID() == APPROACH_VERTEX_ID ||
         (vertex->getHostEdge() != NULL &&
          vertex->getHostEdge()->getID() == APPROACH_TEST_POINT_HOST_EDGE_ID))) {
        vertex->detectAgent.dump();
        REPORT_DEBUG;
    }
#endif
    if (std::find(vertices.begin(), vertices.end(), vertex) != vertices.end())
        return;
    vertices.push_back(vertex);
}

void ApproachingVertices::removeVertex(Vertex *vertex)
{
    if (std::find(vertices.begin(), vertices.end(), vertex) == vertices.end())
        return;
#ifdef TRACK_APPROACH_VERTEX
    if ((vertex->getID() == APPROACH_VERTEX_ID ||
         (vertex->getHostEdge() != NULL &&
          vertex->getHostEdge()->getID() == APPROACH_TEST_POINT_HOST_EDGE_ID))) {
        vertex->detectAgent.dump();
        REPORT_DEBUG;
    }
#endif
    vertices.remove(vertex);
}

void ApproachingVertices::jumpVertex(Vertex *vertex1, Vertex *vertex2)
{
#ifdef DEBUG
    assert(std::find(vertices.begin(), vertices.end(), vertex1) != vertices.end());
    assert(std::find(vertices.begin(), vertices.end(), vertex2) != vertices.end());
#endif
#ifdef TRACK_APPROACH_VERTEX
    if ((vertex2->getID() == APPROACH_VERTEX_ID ||
         (vertex2->getHostEdge() != NULL &&
          vertex2->getHostEdge()->getID() == APPROACH_TEST_POINT_HOST_EDGE_ID))) {
        vertex2->detectAgent.dump();
        REPORT_DEBUG;
    }
#endif
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end(); ++it)
        if (*it == vertex1)
            break;
    vertices.remove(vertex2);
    vertices.insert(it, vertex2);
}

void ApproachingVertices::dump()
{
    cout << "***** Approaching vertices list:" << endl;
    cout << "      Number: " << vertices.size() << endl;
    cout << "      Vertex IDs: ";
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end(); ++it) {
        if ((*it)->getID() != -1)
            cout << setw(15) << (*it)->getID();
        else
            cout << setw(15) << "[" << (*it)->getHostEdge()->getID() << "]";
    }
    cout << endl;
}
