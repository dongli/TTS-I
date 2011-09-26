#include "ApproachingVertices.h"
#include "Vertex.h"

using namespace ApproachDetector;

std::list<Vertex *> ApproachingVertices::vertices;

//#define TRACK_APPROACH_VERTEX
#define APPROACH_VERTEX_ID 51501

ApproachingVertices::ApproachingVertices()
{
}

ApproachingVertices::~ApproachingVertices()
{
}

void ApproachingVertices::recordVertex(Vertex *vertex)
{
#ifdef DEBUG
    if (std::find(vertices.begin(), vertices.end(), vertex) != vertices.end()) {
        vertex->detectAgent.dump();
        cout << "---> Duplicate record of vertex " << vertex->getID() << endl;
        return;
    }
#endif
#ifdef TRACK_APPROACH_VERTEX
    if (vertex->getID() == APPROACH_VERTEX_ID) {
        vertex->detectAgent.dump();
        REPORT_DEBUG
    }
#endif
    vertices.push_back(vertex);
}

void ApproachingVertices::removeVertex(Vertex *vertex)
{
#ifdef DEBUG
    assert(std::find(vertices.begin(), vertices.end(), vertex) != vertices.end());
#endif
#ifdef TRACK_APPROACH_VERTEX
    if (vertex->getID() == APPROACH_VERTEX_ID) {
        vertex->detectAgent.dump();
        REPORT_DEBUG
    }
#endif
    if (vertex->detectAgent.getActiveProjection() != NULL)
        return;
    vertices.remove(vertex);
}

void ApproachingVertices::jumpVertex(Vertex *vertex1, Vertex *vertex2)
{
#ifdef DEBUG
    assert(std::find(vertices.begin(), vertices.end(), vertex1) != vertices.end());
    assert(std::find(vertices.begin(), vertices.end(), vertex2) != vertices.end());
#endif
#ifdef TRACK_APPROACH_VERTEX
    if (vertex2->getID() == APPROACH_VERTEX_ID) {
        vertex2->detectAgent.dump();
        REPORT_DEBUG
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
    for (; it != vertices.end(); ++it)
        cout << setw(8) << (*it)->getID();
    cout << endl;
}
