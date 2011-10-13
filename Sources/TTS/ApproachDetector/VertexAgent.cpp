#include "VertexAgent.h"
#include "Vertex.h"
#include "Edge.h"

using namespace ApproachDetector;

VertexAgent::VertexAgent()
{
    host = NULL;
    reinit();
}

VertexAgent::~VertexAgent()
{
}

void VertexAgent::checkin(Vertex *vertex)
{
    host = vertex;
}

void VertexAgent::reinit()
{
}

void VertexAgent::clean()
{
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if ((*it).getEdge() != NULL)
            (*it).getEdge()->detectAgent.removeVertex(host);
    }
    projections.clear();
}

void VertexAgent::recordProjection(Edge *edge, Projection *projection)
{
#ifdef DEBUG
    assert(projection->getEdge() == NULL);
    assert(projection->getVertex() == NULL);
#endif
    projection->setEdge(edge);
    projection->setVertex(host);
    projection->setCalculated();
    projections.push_back(*projection);
}

void VertexAgent::removeProjection(Projection *projection)
{
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if (&(*it) == projection) {
            projections.erase(it);
            return;
        }
    }
}

void VertexAgent::expireProjection()
{
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it)
        (*it).expire();
}

Projection *VertexAgent::getProjection(Edge *edge)
{
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if ((*it).getEdge() == edge)
            return &(*it);
    }
    return NULL;
}

Projection *VertexAgent::getActiveProjection()
{
    Projection *activeProjection = NULL;
    double distance = UndefinedDistance;
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if ((*it).isApproaching()) {
            if ((*it).getDistance(NewTimeLevel) < distance ||
                distance == UndefinedDistance) {
                distance = (*it).getDistance(NewTimeLevel);
                activeProjection = &(*it);
            }
        }
    }
    return activeProjection;
}

double VertexAgent::getShortestDistance()
{
    double distance = UndefinedDistance;
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if ((*it).getDistance(NewTimeLevel) < distance ||
            distance == UndefinedDistance) {
            distance = (*it).getDistance(NewTimeLevel);
        }
    }
    return distance;
}

void VertexAgent::dump()
{
    cout << "Paired edge projections of vertex ";
    cout << host->getID() << ":" << endl;
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        Vertex *vertex1 = (*it).getEdge()->getEndPoint(FirstPoint);
        Vertex *vertex2 = (*it).getEdge()->getEndPoint(SecondPoint);
        cout << "  ** Edge ID: ";
        cout << (*it).getEdge()->getID() << " " << (*it).getEdge() << endl;
        cout << "     Approaching?: ";
        if ((*it).isApproaching())
            cout << "yes" << endl;
        else
            cout << "no" << endl;
        cout << "     First point ID: " << setw(8) << vertex1->getID() << endl;
        vertex1->getCoordinate(OldTimeLevel).dump();
        vertex1->getCoordinate(NewTimeLevel).dump();
        cout << "     Second point ID: " << setw(8) << vertex2->getID() << endl;
        vertex2->getCoordinate(OldTimeLevel).dump();
        vertex2->getCoordinate(NewTimeLevel).dump();
        cout << "     Old projection coordinate: ";
        (*it).getCoordinate(OldTimeLevel).dump();
        cout << "     New projection coordinate: ";
        (*it).getCoordinate(NewTimeLevel).dump();
        cout << "     Old distance: ";
        cout << (*it).getDistance(OldTimeLevel) << endl;
        cout << "     New distance: ";
        cout << (*it).getDistance(NewTimeLevel) << endl;
    }
}