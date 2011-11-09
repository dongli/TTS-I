#include "Edge.h"
#include "Polygon.h"
#include "Constants.h"
#include "Sphere.h"
#ifdef TTS_ONLINE
#include "TTS.h"
#endif

Edge::Edge()
{
#ifdef TTS_ONLINE
    detectAgent.checkin(this);
#endif
    normVector.init();
    reinit();
}

Edge::~Edge()
{
    clean();
}

void Edge::reinit()
{
    for (int i = 0; i < 2; ++i) {
        endPoints[i] = NULL;
        polygons[i] = NULL;
        edgePointers[i] = NULL;
    }
    isNormVectorSet = false;
#ifdef TTS_ONLINE
    detectAgent.reinit();
#endif
}

void Edge::clean()
{
    for (int i = 0; i < 2; ++i)
        if (endPoints[i] != NULL)
            endPoints[i]->unlinkEdge(this);
#ifdef TTS_ONLINE
    detectAgent.clean();
#endif
}

void Edge::linkEndPoint(PointOrder order, Vertex *point)
{
    isNormVectorSet = false;
    endPoints[order] = point;
    if (point != NULL)
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
        testPoint.setCoordinate(xo, NewTimeLevel);
    }
}

#ifdef TTS_ONLINE
void Edge::changeEndPoint(PointOrder order, Vertex *point,
                          MeshManager &meshManager,
                          const FlowManager &flowManager)
{
    Vertex *testPoint = getTestPoint();
    testPoint->Point::reinit();
    if (endPoints[order] != NULL)
        endPoints[order]->unlinkEdge(this);
    linkEndPoint(order, point);
    calcNormVector();
    calcLength();
    TTS::recordTask(TTS::UpdateAngle, getEdgePointer(OrientLeft));
    TTS::recordTask(TTS::UpdateAngle, getEdgePointer(OrientLeft)->next);
    TTS::recordTask(TTS::UpdateAngle, getEdgePointer(OrientRight));
    TTS::recordTask(TTS::UpdateAngle, getEdgePointer(OrientRight)->next);
    Location loc;
    meshManager.checkLocation(testPoint->getCoordinate(), loc);
    testPoint->setLocation(loc);
    TTS::track(meshManager, flowManager, testPoint);
}
#endif

void Edge::linkPolygon(OrientStatus orient, Polygon *polygon)
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

void Edge::setPolygon(OrientStatus orient, Polygon *polygon)
{
    polygons[orient] = polygon;
}

void Edge::setEdgePointer(OrientStatus orient, EdgePointer *edgePointer)
{
#ifdef DEBUG
    assert(polygons[orient] != NULL);
#endif
    edgePointers[orient] = edgePointer;
    edgePointer->edge = this;
    edgePointer->orient = orient;
#ifdef TTS_ONLINE
    TTS::recordTask(TTS::UpdateAngle, edgePointer);
    TTS::recordTask(TTS::UpdateAngle, edgePointer->next);
#else
    edgePointer->resetAngle();
    edgePointer->next->resetAngle();
#endif
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
        testPoint = that.testPoint;
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
    if (polygons[OrientLeft] != NULL)
        cout << polygons[OrientLeft]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Right Polygon ID:      ";
    if (polygons[OrientRight] != NULL)
        cout << polygons[OrientRight]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Left Edge Pointer ID:  ";
    if (edgePointers[OrientLeft] != NULL)
        cout << edgePointers[OrientLeft]->getID() << endl;
    else
        cout << "NONE" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Right Edge Pointer ID: ";
    if (edgePointers[OrientRight] != NULL)
        cout << edgePointers[OrientRight]->getID() << endl;
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

Vertex *EdgePointer::getEndPoint(PointOrder order) const
{
    if (orient == OrientLeft) {
        return edge->getEndPoint(order);
    } else if (orient == OrientRight) {
        order = order == FirstPoint ? SecondPoint : FirstPoint;
        return edge->getEndPoint(order);
    } else {
        REPORT_ERROR("Unknown order.")
    }
}

Polygon *EdgePointer::getPolygon(OrientStatus orient) const
{
    if (this->orient == OrientLeft) {
        return edge->getPolygon(orient);
    } else if (this->orient == OrientRight) {
        orient = orient == OrientLeft ? OrientRight : OrientLeft;
        return edge->getPolygon(orient);
    } else {
        REPORT_ERROR("Unknown orient.");
    }
}

EdgePointer *EdgePointer::getNeighborEdgePointer() const
{
    switch (orient) {
        case OrientLeft:
            return edge->getEdgePointer(OrientRight);
            break;
        case OrientRight:
            return edge->getEdgePointer(OrientLeft);
            break;
        default:
            REPORT_ERROR("Unknown orient.");
    }
}

double EdgePointer::calcAngle(const Vector &vector1, const Vector &vector2,
                              const Coordinate &x)
{
    double angle;
    
    // -------------------------------------------------------------------------
    double tmp = dot(vector1, vector2);
    tmp = fmax(-1.0, fmin(1.0, tmp));
    angle = acos(-tmp);
    
    // -------------------------------------------------------------------------
    // handle obtuse angle
    Vector judge = cross(vector1, vector2);
    if (dot(x.getCAR(), judge) < 0.0)
        angle = PI2-angle;
    
    return angle;
}

double EdgePointer::calcAngle(const Vector &vector1, const Vector &vector2,
                              const Vertex &vertex)
{
    return calcAngle(vector1, vector2, vertex.getCoordinate());
}

Vector EdgePointer::getNormVector(TimeLevel timeLevel) const
{
    if (orient == OrientLeft) {
        if (timeLevel == OldTimeLevel)
            return edge->normVector.getOld();
        else if (timeLevel == NewTimeLevel)
            return edge->normVector.getNew();
        else
            REPORT_ERROR("Unknown time level.")
    } else if (orient == OrientRight) {
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
    if (angle > 359/Rad2Deg) {
        cout << "Left polygon ID: " << edge->polygons[0]->getID() << endl;
        cout << "Right polygon ID: " << edge->polygons[1]->getID() << endl;
        edge->polygons[0]->dump("left_polygon");
        edge->polygons[1]->dump("right_polygon");
        REPORT_DEBUG
    }
    this->angle.setNew(angle);
    if (!isAngleSet) {
        angle = calcAngle(prev->getNormVector(OldTimeLevel),
                          getNormVector(OldTimeLevel), *point);
        if (angle > 359/Rad2Deg) {
            cout << "Left polygon ID: " << edge->polygons[0]->getID() << endl;
            cout << "Right polygon ID: " << edge->polygons[1]->getID() << endl;
            edge->polygons[0]->dump("left_polygon");
            edge->polygons[1]->dump("right_polygon");
            REPORT_DEBUG
        }
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
#ifdef DEBUG_BACKUP
        if (angle.getOld() > 20.0/Rad2Deg && angle.getOld() < 340.0/Rad2Deg) {
            REPORT_WARNING("This may be not a wrong angle situation!")
            cout << "Old angle: " << angle.getOld()*Rad2Deg << endl;
            cout << "New angle: " << angle.getNew()*Rad2Deg << endl;
            cout << "Edge ID: " << edge->getID() << endl;
            cout << "First point ID: " << edge->getEndPoint(FirstPoint)->getID() << endl;
            cout << "Second point ID: " << edge->getEndPoint(SecondPoint)->getID() << endl;
            cout << "Left polygon ID: " << edge->getPolygon(OrientLeft)->getID() << endl;
            cout << "Right polygon ID: " << edge->getPolygon(OrientRight)->getID() << endl;
            cout << "Dump of the two polygons:" << endl;
            cout << "Left:" << endl;
            edge->getPolygon(OrientLeft)->dump();
            cout << "Right:" << endl;
            edge->getPolygon(OrientRight)->dump();
        }
#endif
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

EdgePointer &EdgePointer::operator=(const EdgePointer &that)
{
    if (this != &that) {
        edge = that.edge;
        orient = that.orient;
        angle = that.angle;
        isAngleSet = that.isAngleSet;
    }
    return *this;
}

void EdgePointer::replace(EdgePointer *oldEdgePointer)
{
    *this = *oldEdgePointer;
    edge->edgePointers[oldEdgePointer->orient] = this;
}

void EdgePointer::dump(int indentLevel) const
{
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "Edge Pointer " << getID() << ":" << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Polygon ID: ";
    if (orient == OrientLeft) {
        cout << edge->getPolygon(OrientLeft)->getID() << endl;
    } else {
        cout << edge->getPolygon(OrientRight)->getID() << endl;
    }
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
    string orientStr = orient == OrientLeft ? "Left" : "Right";
    cout << "  Orientation:        " << orientStr << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  First Point ID:  " << getEndPoint(FirstPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Second Point ID: " << getEndPoint(SecondPoint)->getID() << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  Old Angle:       " << getAngle(OldTimeLevel)*Rad2Deg << endl;
    for (int i = 0; i < indentLevel; ++i)
        cout << "  ";
    cout << "  New Angle:       " << getAngle(NewTimeLevel)*Rad2Deg << endl;
}