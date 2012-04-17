#include "EdgeAgent.h"
#include "Vertex.h"
#include "Edge.h"
#include "PolygonManager.h"
#include "CurvatureGuard.h"
#include "PotentialCrossDetector.h"
#include "ReportMacros.h"

using namespace ApproachDetector;
using namespace CurvatureGuard;
using namespace PotentialCrossDetector;

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
    vertices.clear();
}

void EdgeAgent::clean()
{
    std::list<Vertex *>::iterator it;
    for (it = vertices.begin(); it != vertices.end();)
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

void EdgeAgent::updateVertexProjections(MeshManager &meshManager)
{
    Vertex *vertex1 = host->getEndPoint(FirstPoint);
    Vertex *vertex2 = host->getEndPoint(SecondPoint);
    std::list<Vertex *>::iterator it = vertices.begin();
    for (int i = 0; i < vertices.size();) {
        Vertex *vertex3 = *it;
        Projection *projection = vertex3->detectAgent.getProjection(host);
        if (vertex3 == vertex1 || vertex3 == vertex2 ||
            vertex3->getHostEdge() == host ||
            // Note: To avoid line polygons.
            (vertex3->getHostEdge() != NULL &&
             ((vertex3->getHostEdge()->getEndPoint(FirstPoint) == vertex1 &&
               vertex3->getHostEdge()->getEndPoint(SecondPoint) == vertex2) ||
              (vertex3->getHostEdge()->getEndPoint(FirstPoint) == vertex2 &&
               vertex3->getHostEdge()->getEndPoint(SecondPoint) == vertex1)))) {
            AgentPair::unpair(it, host);
            if (projection->tags.isSet(Approaching) &&
                vertex3->detectAgent.getActiveProjection() == NULL)
                ApproachingVertices::removeVertex(vertex3);
        } else {
            bool isAlreadyApproaching = projection->tags.isSet(Approaching);
            ProjectionStatus status = projection->project(NewTimeLevel);
            if (status == HasNoProjection) {
                AgentPair::unpair(it, projection);
            } else {
                projection->project(OldTimeLevel);
                if (status == CrossEdge) {
                    if (vertex3->getID() == -1) {
                        static_cast<TestPoint *>(vertex3)->reset(meshManager);
                        if (vertex3->detectAgent.getProjection(host) != NULL)
                            ++it;
                    } else {
                        projection->tags.set(Crossing);
                        Message message;
                        message << "Vertex " << vertex3->getID() << " crosses ";
                        message << "edge " << host->getID() << "!";
                        REPORT_WARNING(message.str());
                        ++it;
                    }
                } else {
                    projection->checkApproaching();
                    if (projection->tags.isSet(Approaching)) {
                        if (!isAlreadyApproaching)
                            ApproachingVertices::recordVertex(vertex3);
                    } else {
                        if (isAlreadyApproaching &&
                            vertex3->detectAgent.getActiveProjection() == NULL)
                            ApproachingVertices::removeVertex(vertex3);
                    }
                    ++it;
                }
                ++i;
            }
        }
    }
}

void EdgeAgent::handoverVertices(Edge *edge)
{
    if (edge == host)
        return;
    Projection p, *projection;
    Vertex *vertex1, *vertex2, *vertex3;
    vertex1 = edge->getEndPoint(FirstPoint);
    vertex2 = edge->getEndPoint(SecondPoint);
    std::list<Vertex *>::iterator it = vertices.begin();
    for (; it != vertices.end(); ++it) {
        vertex3 = *it;
        if (vertex3 == vertex1 || vertex3 == vertex2 ||
            vertex3->getHostEdge() == edge) {
            continue;
        }
       projection = vertex3->detectAgent.getProjection(edge);
        if (projection == NULL) {
            // -----------------------------------------------------------------
            // Scenario 1
            //   Vertex3 is not paired with edge.
            projection = &p;
            projection->reinit();
            if (projection->project(vertex3, edge, NewTimeLevel)) {
                if (vertex3->detectAgent.getProjection(host)->tags.isSet(Crossing)) {
                    Message message;
                    message << "Vertex " << vertex3->getID();
                    message << " is crossing old edge (";
                    message << host->getEndPoint(FirstPoint)->getID() << " -> ";
                    message << host->getEndPoint(SecondPoint)->getID() << ")! ";
                    message << "This should be caused by crossing vertices.";
                    REPORT_ERROR(message.str());
                }
                projection->project(vertex3, edge, OldTimeLevel);
                projection->checkApproaching();
                if (projection->tags.isSet(Approaching)) {
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
        cout << setw(12) << *it << ":";
        if ((*it)->getID() != -1)
            cout << setw(10) << (*it)->getID() << endl;
        else
            cout << setw(10) << "[" << (*it)->getHostEdge()->getID() << "]" << endl;
    }
    cout << endl;
}