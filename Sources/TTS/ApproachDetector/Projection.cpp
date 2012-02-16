#include "Projection.h"
#include "Vertex.h"
#include "Edge.h"
#ifdef DEBUG
#include "TimeManager.h"
#endif

using namespace ApproachDetector;

//#define TRACK_PROJECTION
#define PROJECT_VERTEX_ID 45804
#define PROJECT_EDGE_ID 85762

Projection::Projection()
{
    x.init();
    distance.init();
    reinit();
}

Projection::Projection(Vertex *vertex, Edge *edge)
{
    reinit();
    this->vertex = vertex;
    this->edge = edge;
}

Projection::Projection(const Projection &that)
{
    x.init();
    distance.init();
    *this = that;
}
    
Projection::~Projection()
{
}

void Projection::reinit()
{
    vertex = NULL;
    edge = NULL;
    approach = false;
    calculated = false;
}

ProjectionStatus Projection::project(TimeLevel timeLevel)
{
#ifdef DEBUG
    if (distance.getOld() == UndefinedDistance &&
        distance.getNew() == UndefinedDistance) {
        vertex->detectAgent.dump();
        REPORT_ERROR("Old and new distances should not "
                     "be undefined at the same time!")
    }
#endif
#ifdef TRACK_PROJECTION
    if (vertex->getID() == PROJECT_VERTEX_ID &&
        edge->getID() == PROJECT_EDGE_ID) {
        cout << endl << endl;
        cout << "*** ApproachDetector::Projection::project ***" << endl;
        vertex->detectAgent.dump();
        cout << endl << endl;
        REPORT_DEBUG;
    }
#endif
    if (timeLevel == NewTimeLevel) {
        x.save();
        distance.save();
        setCalculated();
    }
    OrientStatus orient = this->orient;
    if (!project(vertex, edge, timeLevel))
        return HasNoProjection;
    if (this->orient != orient)
        return CrossEdge;
    return HasProjection;
}

bool Projection::project(Vertex *vertex, Edge *edge, TimeLevel timeLevel)
{
#ifdef TRACK_PROJECTION
    if (vertex->getID() == PROJECT_VERTEX_ID && edge->getID() == PROJECT_EDGE_ID) {
        cout << endl << endl;
        cout << "*** ApproachDetector::Projection::project ***" << endl;
        vertex->detectAgent.dump();
        cout << endl << endl;
        REPORT_DEBUG;
    }
#endif
    const Coordinate &x1 = edge->getEndPoint(FirstPoint)->getCoordinate(timeLevel);
    const Coordinate &x2 = edge->getEndPoint(SecondPoint)->getCoordinate(timeLevel);
    const Coordinate &x3 = vertex->getCoordinate(timeLevel);
    Coordinate x4;
    double d;
    if (Sphere::project(x1, x2, x3, x4, d)) {
        x.set(timeLevel, x4);
        distance.set(timeLevel, d);
        if (timeLevel == NewTimeLevel)
            orient = Sphere::orient(x1, x2, x3);
        if (orient == OrientOn) {
            if (dot(cross(x3.getCAR(), x1.getCAR()),
                    cross(x3.getCAR(), x2.getCAR())) > 0.0) {
                REPORT_ERROR("Vertex is on the edge!");
            } else {
                return false;
            }
        }
        return true;
    } else {
        distance.set(timeLevel, d);
        return false;
    }
}

const Coordinate &Projection::getCoordinate(TimeLevel timeLevel) const
{
    if (timeLevel == OldTimeLevel)
        return x.getOld();
    else if (timeLevel == NewTimeLevel)
        return x.getNew();
    else
        REPORT_ERROR("Unknown time level.")
}

double Projection::getDistance(TimeLevel timeLevel) const
{
    if (timeLevel == OldTimeLevel)
        return distance.getOld();
    else if (timeLevel == NewTimeLevel)
        return distance.getNew();
    else
        REPORT_ERROR("Unknown time level.");
}

void Projection::calcChangeRate()
{
    changeRate = (distance.getOld()-distance.getNew())/distance.getOld();
}

void Projection::checkApproaching()
{
    approach = ApproachDetector::isApproaching(this);
}

Projection &Projection::operator=(const Projection &that)
{
    if (this != &that) {
        this->vertex = that.vertex;
        this->edge = that.edge;
        this->x = that.x;
        this->distance = that.distance;
        this->orient = that.orient;
        this->changeRate = that.changeRate;
        this->approach = that.approach;
        this->calculated = that.calculated;
    }
    return *this;
}