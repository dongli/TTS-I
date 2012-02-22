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
    ApproachingVertices::removeVertex(host);
}

void VertexAgent::recordProjection(Edge *edge, Projection *projection)
{
#ifdef DEBUG
    assert(projection->getEdge() == NULL);
    assert(projection->getVertex() == NULL);
#endif
    projection->setEdge(edge);
    projection->setVertex(host);
    projection->tags.set(Calculated);
    projections.push_back(*projection);
}

void VertexAgent::removeProjection(Projection *projection)
{
#ifdef DEBUG
    assert(projection != NULL);
#endif
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        if (&(*it) == projection) {
            projections.erase(it);
            return;
        }
    }
}

void VertexAgent::removeProjection(std::list<Projection>::iterator &it)
{
    it = projections.erase(it);
}

void VertexAgent::expireProjection()
{
    Polygon *polygon1, *polygon2, *polygon3, *polygon4;
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end();) {
        // Note: There is a possibility that the paired vertex and edge belong
        //       to different polygons after one is split, so they should be
        //       unpaired to avoid unconsistent projection.
        bool isSeperated = true;
        polygon1 = (*it).getEdge()->getPolygon(OrientLeft);
        polygon2 = (*it).getEdge()->getPolygon(OrientRight);
        if (host->getHostEdge() == NULL) {
            EdgePointer *linkedEdge = host->linkedEdges.front();
            for (int i = 0; i < host->linkedEdges.size(); ++i) {
                polygon3 = linkedEdge->edge->getPolygon(OrientLeft);
                polygon4 = linkedEdge->edge->getPolygon(OrientRight);
                if (polygon1 == polygon3 || polygon1 == polygon4 ||
                    polygon2 == polygon3 || polygon2 == polygon4) {
                    isSeperated = false;
                    break;
                }
                linkedEdge = linkedEdge->next;
            }
        } else {
            polygon3 = host->getHostEdge()->getPolygon(OrientLeft);
            polygon4 = host->getHostEdge()->getPolygon(OrientRight);
            if (polygon1 == polygon3 || polygon1 == polygon4 ||
                polygon2 == polygon3 || polygon2 == polygon4)
                isSeperated = false;
        }
        if (isSeperated) {
            AgentPair::unpair(it);
            if (getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(host);
        } else {
            (*it).expire();
            ++it;
        }
    }
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
        if ((*it).tags.isSet(Approaching)) {
            if ((*it).getDistance(NewTimeLevel) < distance ||
                distance == UndefinedDistance) {
                distance = (*it).getDistance(NewTimeLevel);
                activeProjection = &(*it);
            }
        }
    }
    return activeProjection;
}

bool VertexAgent::isCrossing()
{
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it)
        if ((*it).tags.isSet(Crossing))
            return true;
    return false;
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

VertexAgent &VertexAgent::operator=(const VertexAgent &that)
{
    if (this != &that) {
        projections = that.projections;
        std::list<Projection>::iterator it = projections.begin();
        for (; it != projections.end(); ++it) {
            if ((*it).tags.isSet(Approaching)) {
                // Note: If the host of "that" is approaching to some edge,
                //       remember to replace it with "this" host.
                ApproachingVertices::removeVertex(that.host);
                ApproachingVertices::recordVertex(host);
            }
            (*it).setVertex(host);
            (*it).getEdge()->detectAgent.removeVertex(that.host);
            (*it).getEdge()->detectAgent.recordVertex(host);
        }
    }
    return *this;
}

void VertexAgent::dump()
{
    cout << "Paired edge projections of vertex ";
    cout << host->getID() << " " << host << ":" << endl;
    std::list<Projection>::iterator it = projections.begin();
    for (; it != projections.end(); ++it) {
        Edge *edge = (*it).getEdge();
        Vertex *vertex1 = edge->getEndPoint(FirstPoint);
        Vertex *vertex2 = edge->getEndPoint(SecondPoint);
        cout << "  ** Edge ID: ";
        cout << edge->getID() << " " << edge << endl;
        cout << "     Approaching?: ";
        if ((*it).tags.isSet(Approaching))
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