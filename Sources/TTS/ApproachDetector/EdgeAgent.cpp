#include "EdgeAgent.h"
#include "Vertex.h"
#include "Edge.h"
#include "ReportMacros.h"

using namespace ApproachDetector;

EdgeAgent::EdgeAgent()
{
    host = NULL;
    vertices.clear();
    reinit();
}

EdgeAgent::~EdgeAgent()
{
}

void EdgeAgent::checkin(Edge *edge)
{
    host = edge;
}

void EdgeAgent::reinit()
{
}

void EdgeAgent::clean()
{
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end();)
        AgentPair::unpair(it, host);
}

void EdgeAgent::recordVertex(Vertex *vertex)
{
    vertices.push_back(vertex);
}

void EdgeAgent::removeVertex(Vertex *vertex)
{
    vertices.remove(vertex);
}

void EdgeAgent::removeVertex(std::list<Vertex *>::iterator &it)
{
    it = vertices.erase(it);
}

void EdgeAgent::updateVertexProjections()
{
    Vertex *vertex1 = host->getEndPoint(FirstPoint);
    Vertex *vertex2 = host->getEndPoint(SecondPoint);
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end();) {
        Vertex *vertex3 = *it;
        Projection *projection = vertex3->detectAgent.getProjection(host);
        if (vertex3 == vertex1 || vertex3 == vertex2) {
            AgentPair::unpair(it, host);
            if (projection->isApproaching() &&
                vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
        } else {
            bool isAlreadyApproaching = projection->isApproaching();
            if (projection->project(NewTimeLevel)) {
                projection->project(OldTimeLevel);
                projection->checkApproaching();
                if (projection->isApproaching()) {
                    if (!isAlreadyApproaching)
                        ApproachingVertices::recordVertex(vertex3);
                } else
                    if (isAlreadyApproaching)
                        ApproachingVertices::removeVertex(vertex3);
                ++it;
            } else
                AgentPair::unpair(it, projection);
        }
    }
}

void EdgeAgent::handoverVertices(Edge *edge)
{
    if (edge == host)
        return;
    Projection p;
    Vertex *vertex1 = edge->getEndPoint(FirstPoint);
    Vertex *vertex2 = edge->getEndPoint(SecondPoint);
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end(); ++it) {
        Vertex *vertex3 = *it;
        if (vertex3 == vertex1 || vertex3 == vertex2) {
            continue;
        }
        Projection *projection = vertex3->detectAgent.getProjection(edge);
        if (projection == NULL) {
            // -----------------------------------------------------------------
            // Scenario 1
            //   Vertex3 is not paired with edge.
            projection = &p;
            projection->reinit();
            if (projection->project(vertex3, edge, NewTimeLevel)) {
                projection->project(vertex3, edge, OldTimeLevel);
                projection->checkApproaching();
                if (projection->isApproaching()) {
#ifdef DEBUG
                    ostringstream message;
                    message << "Paired vertex " << vertex3->getID() << " is handled over.";
                    NOTICE("EdgeAgent::handoverVertices", message.str())
#endif
                    if (vertex3->detectAgent.getActiveProjection() == NULL)
                        ApproachingVertices::recordVertex(vertex3);
                }
                AgentPair::pair(vertex3, edge, projection);
            }
        } else {
            // -----------------------------------------------------------------
            // Scenario 2
            //   Vertex3 is already paired with edge.
            continue;
        }
    }
}

void EdgeAgent::dump()
{
    cout << "Paired vertices:" << endl;
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end(); ++it) {
        cout << setw(8) << (*it)->getID();
    }
    cout << endl;
}