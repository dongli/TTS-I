#ifndef _Edge_h_
#define _Edge_h_

#include "List.h"
#include "Vertex.h"
#include "Vector.h"
#include "Sphere.h"

class Polygon;
class EdgePointer;
#ifdef TTS_ONLINE
class MeshManager;
class FlowManager;
#include "ApproachDetector.h"
#include "EdgeTags.h"
#endif
#if defined TTS_ONLINE || PREPROCESS
#include "TestPoint.h"
#endif

// -----------------------------------------------------------------------------
enum PointOrder {
    FirstPoint = 0, SecondPoint = 1
};

// -----------------------------------------------------------------------------
class Edge : public ListElement<Edge>
{
public:
	Edge();
	virtual ~Edge();

    void reinit();
    void clean();

    void linkEndPoint(PointOrder, Vertex *, bool isSetTestPoint = true);
    Vertex *getEndPoint(PointOrder order) const { return endPoints[order]; }
#ifdef TTS_ONLINE
    void changeEndPoint(PointOrder, Vertex *point,
                        MeshManager &, const FlowManager &);
    void changeEndPoint(PointOrder, Vertex *point, Vertex *testPoint,
                        MeshManager &, const FlowManager &);
#endif
#if defined TTS_ONLINE || PREPROCESS
    TestPoint *getTestPoint() { return &testPoint; }
#endif

    void linkPolygon(OrientStatus, Polygon *);
    void setPolygon(OrientStatus, Polygon *);
    Polygon *getPolygon(OrientStatus orient) const { return polygons[orient]; }

    void setEdgePointer(OrientStatus, EdgePointer *);
    EdgePointer *getEdgePointer(OrientStatus orient) const { return edgePointers[orient]; }

    void calcNormVector();
    const Vector &getNormVector(TimeLevel timeLevel = NewTimeLevel) const;

    void calcLength();
    double getLength() const { return length; }

    Edge &operator=(const Edge &);

    void dump(int indentLevel = 0) const;

#ifdef TTS_ONLINE
    ApproachDetector::EdgeAgent detectAgent;
    EdgeTags tags;
#endif

private:
    friend class EdgePointer;

	Vertex *endPoints[2];
#if defined TTS_ONLINE || PREPROCESS
    TestPoint testPoint;
#endif
    Polygon *polygons[2];
	EdgePointer *edgePointers[2];
    MultiTimeLevel<Vector, 2> normVector;
    bool isNormVectorSet;
    double length;
};

// -----------------------------------------------------------------------------
#define UNSET_ANGLE -999.0

class EdgePointer : public ListElement<EdgePointer>
{
public:
	EdgePointer();
	virtual ~EdgePointer();

    void reinit();

    Vertex *getEndPoint(PointOrder order) const;
    Polygon *getPolygon(OrientStatus orient) const;

    EdgePointer *getNeighborEdgePointer() const;

    Vector getNormVector(TimeLevel timeLevel = NewTimeLevel) const;

    static double calcAngle(const Vector &, const Vector &);
    static double calcAngle(const Vector &, const Vector &, const Coordinate &);
    static double calcAngle(const Vector &, const Vector &, const Vertex &);

    void calcAngle();
    void resetAngle();
    double getAngle(TimeLevel timeLevel = NewTimeLevel) const {
        return angle.get(timeLevel);
    }
    bool isWrongAngle() const;
    bool isTangly() const;

    EdgePointer &operator=(const EdgePointer &);
    void replace(EdgePointer *);

    void dump(int indentLevel = 0) const;

	Edge *edge;
    OrientStatus orient;
    MultiTimeLevel<double, 2> angle;
    bool isAngleSet;
};

#endif
