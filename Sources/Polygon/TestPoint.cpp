#include "TestPoint.h"
#include "Vector.h"
#include "Edge.h"
#include "MeshManager.h"
#include "ApproachDetector.h"
#include "TTS.h"

using namespace ApproachDetector;

TestPoint::TestPoint()
{
    reinit();
}

TestPoint::~TestPoint()
{
}

void TestPoint::reinit()
{
    Point::reinit();
}

#ifdef TTS_ONLINE
void TestPoint::reset(MeshManager &meshManager)
{
    const Coordinate &x1 = hostEdge->getEndPoint(FirstPoint)->getCoordinate();
    const Coordinate &x2 = hostEdge->getEndPoint(SecondPoint)->getCoordinate();
    Coordinate x3, x4;
    Sphere::calcMiddlePoint(x1, x2, x3);
    x4 = x.getOld();
    x.setNew(x3); x.setOld(x4);
    meshManager.checkLocation(x3, loc);
    // TODO: Clarify these codes.
    // reset projections
    if (detectAgent.getActiveProjection() == NULL)
        ApproachingVertices::removeVertex(this);
    std::list<Projection>::iterator itPrj;
    for (itPrj = detectAgent.getProjections().begin();
         itPrj != detectAgent.getProjections().end(); ++itPrj) {
        (*itPrj).expire();
        if ((*itPrj).project(NewTimeLevel) != HasNoProjection) {
            (*itPrj).project(OldTimeLevel);
            (*itPrj).tags.unset(Approaching);
        } else
            AgentPair::unpair(itPrj);
    }
}

void TestPoint::calcAngle()
{
    const Coordinate &x1 = hostEdge->getEndPoint(FirstPoint)->getCoordinate();
    const Coordinate &x2 = x.getNew();
    const Coordinate &x3 = hostEdge->getEndPoint(SecondPoint)->getCoordinate();
    Vector vector1 = norm_cross(x2.getCAR(), x1.getCAR());
    Vector vector2 = norm_cross(x3.getCAR(), x2.getCAR());
    angle = Sphere::calcAngle(vector1, vector2);
}

void TestPoint::calcOrient()
{
    orient = Sphere::orient(hostEdge->getEndPoint(FirstPoint),
                            hostEdge->getEndPoint(SecondPoint),
                            this);
}

TestPoint &TestPoint::operator=(const TestPoint &that)
{
    if (this != &that) {
        Vertex::operator=(that);
        this->angle = that.angle;
        this->orient = that.orient;
    }
    return *this;
}

TestPoint &TestPoint::operator=(const Vertex &that)
{
    Vertex::operator=(that);
    return *this;
}
#endif