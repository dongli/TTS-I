#ifndef _Edge_h_
#define _Edge_h_

#include "List.h"
#include "Vertex.h"
#include "Vector.h"
#include "Sphere.h"
#ifdef TTS_ONLINE
#include "MeshManager.h"
#include "FlowManager.h"
#include "ApproachDetector.h"
#endif

class Polygon;
class EdgePointer;

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

    void linkEndPoint(PointOrder, Vertex *);
    Vertex *getEndPoint(PointOrder order) const { return endPoints[order]; }
#ifdef TTS_ONLINE
    void changeEndPoint(PointOrder, Vertex *, MeshManager &, const FlowManager &);
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

    Vertex *getTestPoint() { return &testPoint; }

    Edge &operator=(const Edge &);

    void dump(int indentLevel = 0) const;

#ifdef TTS_ONLINE
    ApproachDetector::EdgeAgent detectAgent;
#endif

private:
    friend class EdgePointer;

	Vertex *endPoints[2];
    Vertex testPoint;
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

    Vector getNormVector(TimeLevel timeLevel = NewTimeLevel) const;

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
