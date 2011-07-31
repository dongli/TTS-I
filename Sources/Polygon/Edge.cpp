#include "Edge.h"
#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"

#ifdef DEBUG
#include "TimeManager.h"

inline void catchEdge(Edge *edge)
{
    if (edge->getEndPoint(FirstPoint) == NULL ||
        edge->getEndPoint(SecondPoint) == NULL)
        return;
    if (edge->getEndPoint(FirstPoint)->getID() == 3235 &&
        edge->getEndPoint(SecondPoint)->getID() == 36465)
        REPORT_DEBUG
    if (edge->getPolygon(EdgeLeft) == NULL ||
        edge->getPolygon(EdgeRight) == NULL)    
        return;
    if (edge->getEndPoint(FirstPoint)->getID() == 3235 &&
        edge->getEndPoint(SecondPoint)->getID() == 36465 &&
        edge->getPolygon(EdgeLeft)->getID() == 3235 &&
        edge->getPolygon(EdgeRight)->getID() == 3363)
        REPORT_DEBUG
}
#endif

Edge::Edge()
{
    normVector.init();
    reinit();
}

Edge::~Edge()
{
    delete testPoint;
}

void Edge::reinit()
{
    testPoint = new Vertex;
    for (int i = 0; i < 2; ++i) {
        endPoints[i] = NULL;
        polygons[i] = NULL;
        edgePointers[i] = NULL;
    }
    isNormVectorSet = false;
}

void Edge::linkEndPoint(PointOrder order, Vertex *point)
{
    isNormVectorSet = false;
    endPoints[order] = point;
    point->linkEdge(this);

    // -------------------------------------------------------------------------
    // set test point
    // Note: Here use the coordinates of end points at old time level to
    //       calculate the coordinate of test point at new time level, because
    //       the test point will be advected following, so the new coordinate
    //       will be copied to old coordinate.
    if (endPoints[0] != NULL && endPoints[1] != NULL) {
        const Coordinate &x1 = endPoints[0]->getCoordinate(OldTimeLevel);
        const Coordinate &x2 = endPoints[1]->getCoordinate(OldTimeLevel);
        Coordinate xr, xo;
        Sphere::rotate(x1, x2, xr);
        double dlat = (PI05-xr.getLat())*0.5;
        xr.set(xr.getLon(), PI05-dlat);
        Sphere::inverseRotate(x1, xo, xr);
        testPoint->setCoordinate(xo, NewTimeLevel);
    }
}

void Edge::linkPolygon(EdgeOrient orient, Polygon *polygon)
{
#ifdef DEBUG
    assert(polygons[orient] == NULL);
    assert(edgePointers[orient] == NULL);
#endif
    polygons[orient] = polygon;
    EdgePointer *edgePointer;
    polygon->edgePointers.append(&edgePointer);
    edgePointers[orient] = edgePointer;
    edgePointer->edge = this;
    edgePointer->orient = orient;
}

void Edge::setPolygon(EdgeOrient orient, Polygon *polygon)
{
#ifdef DEBUG
    assert(polygons[orient] == NULL);
    assert(edgePointers[orient] == NULL);
#endif
    polygons[orient] = polygon;
}

void Edge::setEdgePointer(EdgeOrient orient, EdgePointer *edgePointer)
{
#ifdef DEBUG
    assert(polygons[orient] != NULL);
#endif
    edgePointers[orient] = edgePointer;
    edgePointer->edge = this;
    edgePointer->orient = orient;
    edgePointer->resetAngle();
    edgePointer->next->resetAngle();
}

void Edge::calcNormVector()
{
    if (isNormVectorSet)
        normVector.save();
    const Coordinate &x1 = endPoints[0]->getCoordinate();
    const Coordinate &x2 = endPoints[1]->getCoordinate();
    Vector tmp = cross(x2.getCAR(), x1.getCAR());
    tmp /= norm(tmp);
    normVector.setNew(tmp);
    if (!isNormVectorSet) {
        const Coordinate &x1 = endPoints[0]->getCoordinate(OldTimeLevel);
        const Coordinate &x2 = endPoints[1]->getCoordinate(OldTimeLevel);
        Vector tmp = cross(x2.getCAR(), x1.getCAR());
        tmp /= norm(tmp);
        normVector.setOld(tmp);
        isNormVectorSet = true;
    }
}

const Vector &Edge::getNormVector(TimeLevel timeLevel) const
{
    if (timeLevel == OldTimeLevel)
        return normVector.getOld();
    else if (timeLevel == NewTimeLevel)
        return normVector.getNew();
    else
        REPORT_ERROR("Unknown time level.")
}

void Edge::calcLength()
{
    length = Sphere::calcDistance(endPoints[FirstPoint]->getCoordinate(),
                                  endPoints[SecondPoint]->getCoordinate());
}

Edge &Edge::operator=(const Edge &that)
{
    if (this != &that) {
        for (int i = 0; i < 2; ++i) {
            endPoints[i] = that.endPoints[i];
            polygons[i] = that.polygons[i];
            edgePointers[i] = that.edgePointers[i];
        }
        *testPoint = *(that.testPoint);
        normVector = that.normVector;
        isNormVectorSet = that.isNormVectorSet;
    }
    return *this;
}

void Edge::dump(int indentLevel) const
{
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "Edge " << getID() << ":" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  First Point ID:        " << endPoints[FirstPoint]->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Second Point ID:       " << endPoints[SecondPoint]->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Left Polygon ID:       ";
    if (polygons[EdgeLeft] != NULL)
        cout << polygons[EdgeLeft]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Right Polygon ID:      ";
    if (polygons[EdgeRight] != NULL)
        cout << polygons[EdgeRight]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Left Edge Pointer ID:  ";
    if (edgePointers[EdgeLeft] != NULL)
        cout << edgePointers[EdgeLeft]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Right Edge Pointer ID: ";
    if (edgePointers[EdgeRight] != NULL)
        cout << edgePointers[EdgeRight]->getID() << endl;
    else
        cout << "NONE" << endl;
}

// -----------------------------------------------------------------------------

EdgePointer::EdgePointer()
{
    angle.init();
    reinit();
}

EdgePointer::~EdgePointer()
{
}

void EdgePointer::reinit()
{
    edge = NULL;
    resetAngle();
}

void EdgePointer::changeEdge(Edge *newEdge, EdgeOrient newOrient)
{
    edge = newEdge;
    newEdge->setEdgePointer(newOrient, this);
    orient = newOrient;
    resetAngle();
    //prev->resetAngle();
    next->resetAngle();
    //calcAngle();
}

Vertex *EdgePointer::getEndPoint(PointOrder order) const
{
    if (orient == EdgeLeft) {
        return edge->getEndPoint(order);
    } else if (orient == EdgeRight) {
        order = order == FirstPoint ? SecondPoint : FirstPoint;
        return edge->getEndPoint(order);
    } else {
        REPORT_ERROR("Unknown orient.")
    }
}

double EdgePointer::calcAngle(const Vector &vector1, const Vector &vector2,
                              const Vertex &vertex)
{
    double angle;

    // -------------------------------------------------------------------------
    double tmp = dot(vector1, vector2);
    tmp = fmax(-1.0, fmin(1.0, tmp));
    angle = acos(-tmp);

    // -------------------------------------------------------------------------
    // handle obtuse angle
    Vector judge = cross(vector1, vector2);
    if (dot(vertex.getCoordinate().getCAR(), judge) < 0.0)
        angle = PI2-angle;

    return angle;
}

Vector EdgePointer::getNormVector(TimeLevel timeLevel) const
{
    if (orient == EdgeLeft) {
        if (timeLevel == OldTimeLevel)
            return edge->normVector.getOld();
        else if (timeLevel == NewTimeLevel)
            return edge->normVector.getNew();
        else
            REPORT_ERROR("Unknown time level.")
    } else if (orient == EdgeRight) {
        if (timeLevel == OldTimeLevel)
            return -edge->normVector.getOld();
        else if (timeLevel == NewTimeLevel)
            return -edge->normVector.getNew();
        else
            REPORT_ERROR("Unknown time level.")
    } else {
        REPORT_ERROR("Unknown orient.")
    }
}

void EdgePointer::calcAngle()
{
    if (isAngleSet)
        this->angle.save();
    Vertex *point = getEndPoint(FirstPoint);
    double angle = calcAngle(prev->getNormVector(), getNormVector(), *point);
    if (angle > 359/Rad2Deg)
        REPORT_DEBUG
    this->angle.setNew(angle);
    if (!isAngleSet) {
        angle = calcAngle(prev->getNormVector(OldTimeLevel),
                          getNormVector(OldTimeLevel), *point);
        if (angle > 359/Rad2Deg)
            REPORT_DEBUG
        this->angle.setOld(angle);
        isAngleSet = true;
    }
}

void EdgePointer::resetAngle()
{
    isAngleSet = false;
    angle.setOld(UNSET_ANGLE);
    angle.setNew(UNSET_ANGLE);
}

bool EdgePointer::isWrongAngle() const
{
    static const double dA = 180.0/Rad2Deg;
    double angleDiff = angle.getNew()-angle.getOld();
    if (fabs(angleDiff) > dA) {
        return true;
    } else {
        return false;
    }
}

bool EdgePointer::isTangly() const
{
    if (next->isWrongAngle())
        return true;
    if (isWrongAngle())
        return true;
    if (prev->isWrongAngle())
        return true;
    return false;
}

void EdgePointer::dump(int indentLevel) const
{
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "Edge Pointer " << getID() << ":" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Linked Edge ID:     " << edge->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "    First Point ID:   " << edge->getEndPoint(FirstPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "    Second Point ID:  " << edge->getEndPoint(SecondPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    string orientStr = orient == EdgeLeft ? "Left" : "Right";
    cout << "  Orientation:        " << orientStr << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  First Point ID:  " << getEndPoint(FirstPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Second Point ID: " << getEndPoint(SecondPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Old Angle:       " << getAngle(OldTimeLevel) << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  New Angle:       " << getAngle(NewTimeLevel) << endl;
}