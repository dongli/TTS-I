#include "AgentPair.h"
#include "Vertex.h"
#include "Edge.h"
#ifdef DEBUG
#include "DebugTools.h"
#endif

using namespace ApproachDetector;

//#define TRACK_PAIRS
#define PAIR_VERTEX_ID 620874
#define PAIR_TEST_POINT_HOST_EDGE_ID -999
#define PAIR_EDGE_ID 184552

void AgentPair::pair(Vertex *vertex, Edge *edge, Projection *projection)
{
#ifdef DEBUG
    if (vertex->detectAgent.getProjection(edge) != NULL) {
        vertex->detectAgent.dump();
        ostringstream message;
        message << "Vertex " << vertex->getID() << " has already been paired ";
        message << "with edge " << edge->getID();
        REPORT_ERROR(message.str())
    }
#endif
#ifdef TRACK_PAIRS
#ifdef PAIR_EDGE_ID
    if ((vertex->getID() == PAIR_VERTEX_ID ||
         (vertex->getHostEdge() != NULL &&
          vertex->getHostEdge()->getID() == PAIR_TEST_POINT_HOST_EDGE_ID)) &&
         edge->getID() == PAIR_EDGE_ID)
#else
    if (vertex->getID() == PAIR_VERTEX_ID)
#endif
    {
//        DebugTools::watch(vertex);
//        DebugTools::watch(edge);
        cout << endl << endl;
        cout << "*** ApproachDetector::AgentPair::pair ***" << endl;
        cout << "---> Vertex " << vertex->getID() << " is paired with ";
        cout << "edge " << edge->getID() << " " << edge << endl;
        cout << "--->   Old projection: ";
        projection->getCoordinate(OldTimeLevel).dump();
        cout << "--->   New projection: ";
        projection->getCoordinate(NewTimeLevel).dump();
        cout << "--->   Old distance: ";
        cout << projection->getDistance(OldTimeLevel) << endl;
        cout << "--->   New distance: ";
        cout << projection->getDistance(NewTimeLevel) << endl;
        cout << endl << endl;
        REPORT_DEBUG;
    }
#endif
    vertex->detectAgent.recordProjection(edge, projection);
    edge->detectAgent.recordVertex(vertex);
}

void AgentPair::unpair(Vertex *vertex, Edge *edge)
{
#ifdef TRACK_PAIRS
#ifdef PAIR_EDGE_ID
    if ((vertex->getID() == PAIR_VERTEX_ID ||
         (vertex->getHostEdge() != NULL &&
          vertex->getHostEdge()->getID() == PAIR_TEST_POINT_HOST_EDGE_ID)) &&
        edge->getID() == PAIR_EDGE_ID)
#else
    if (vertex->getID() == PAIR_VERTEX_ID)
#endif
    {
        cout << "1: ---> Vertex " << vertex->getID() << " is unpaired with ";
        cout << "edge " << edge->getID() << endl;
        REPORT_DEBUG;
    }
#endif
    Projection *projection = vertex->detectAgent.getProjection(edge);
    edge->detectAgent.removeVertex(vertex);
    vertex->detectAgent.removeProjection(projection);
    if (vertex->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex);
}


void AgentPair::unpair(std::list<Vertex *>::iterator &it, Edge *edge)
{
#ifdef TRACK_PAIRS
#ifdef PAIR_EDGE_ID
    if (((*it)->getID() == PAIR_VERTEX_ID ||
         ((*it)->getHostEdge() != NULL &&
          (*it)->getHostEdge()->getID() == PAIR_TEST_POINT_HOST_EDGE_ID)) &&
        edge->getID() == PAIR_EDGE_ID)
#else
    if ((*it)->getID() == PAIR_VERTEX_ID)
#endif
    {
        cout << "2: ---> Vertex " << (*it)->getID() << " is unpaired with ";
        cout << "edge " << edge->getID() << endl;
        REPORT_DEBUG;
    }
#endif
    Vertex *vertex = *it;
    Projection *projection = vertex->detectAgent.getProjection(edge);
    edge->detectAgent.removeVertex(it);
    vertex->detectAgent.removeProjection(projection);
    if (vertex->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex);
}

void AgentPair::unpair(std::list<Vertex *>::iterator &it,
                       Projection *projection)
{
#ifdef TRACK_PAIRS
#ifdef PAIR_EDGE_ID
    if (((*it)->getID() == PAIR_VERTEX_ID ||
         ((*it)->getHostEdge() != NULL &&
          (*it)->getHostEdge()->getID() == PAIR_TEST_POINT_HOST_EDGE_ID)) &&
        projection->getEdge()->getID() == PAIR_EDGE_ID)
#else
    if ((*it)->getID() == PAIR_VERTEX_ID)
#endif
    {
        cout << "3: ---> Vertex " << (*it)->getID() << " is unpaired with ";
        cout << "edge " << projection->getEdge()->getID() << endl;
        REPORT_DEBUG;
    }
#endif
    Vertex *vertex = *it;
    Edge *edge = projection->getEdge();
    edge->detectAgent.removeVertex(it);
    vertex->detectAgent.removeProjection(projection);
    if (vertex->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex);
}

void AgentPair::unpair(std::list<Projection>::iterator &it)
{
    Vertex *vertex = (*it).getVertex();
    Edge *edge = (*it).getEdge();
#ifdef TRACK_PAIRS
#ifdef PAIR_EDGE_ID
    if ((vertex->getID() == PAIR_VERTEX_ID ||
         (vertex->getHostEdge() != NULL &&
          vertex->getHostEdge()->getID() == PAIR_TEST_POINT_HOST_EDGE_ID)) &&
        edge->getID() == PAIR_EDGE_ID)
#else
    if (vertex->getID() == PAIR_VERTEX_ID)
#endif
    {
        cout << "4: ---> Vertex " << vertex->getID() << " is unpaired with ";
        cout << "edge " << edge->getID() << endl;
        REPORT_DEBUG;
    }
#endif
    edge->detectAgent.removeVertex(vertex);
    vertex->detectAgent.removeProjection(it);
    if (vertex->detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(vertex);
}