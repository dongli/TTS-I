#ifndef _Edge_h_
#define _Edge_h_

#include "List.h"
#include "Vertex.h"
#include "Vector.h"

class Polygon;
class EdgePointer;

// -----------------------------------------------------------------------------
enum PointOrder {
    FirstPoint = 0, SecondPoint = 1
};

enum EdgeOrient {
    EdgeLeft = 0, EdgeRight = 1
};

// -----------------------------------------------------------------------------
class Edge : public ListElement<Edge>
{
public:
	Edge();
	virtual ~Edge();

    void reinit();

    void linkEndPoint(PointOrder, Vertex *);
    Vertex *getEndPoint(PointOrder order) const { return endPoints[order]; }

    void linkPolygon(EdgeOrient, Polygon *);
    void setPolygon(EdgeOrient, Polygon *);
    Polygon *getPolygon(EdgeOrient orient) const { return polygons[orient]; }

    void setEdgePointer(EdgeOrient, EdgePointer *);
    EdgePointer *getEdgePointer(EdgeOrient orient) const { return edgePointers[orient]; }

    void calcNormVector();
    const Vector &getNormVector(TimeLevel timeLevel = NewTimeLevel) const;

    void calcLength();
    double getLength() const { return length; }

    Vertex *getTestPoint() { return testPoint; }

    Edge &operator=(const Edge &);

    void dump(int indentLevel = 0) const;

private:
    friend class EdgePointer;

	Vertex *endPoints[2];
    Vertex *testPoint;
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

    void changeEdge(Edge *, EdgeOrient);

    Vertex *getEndPoint(PointOrder order) const;

    Vector getNormVector(TimeLevel timeLevel = NewTimeLevel) const;

    static double calcAngle(const Vector &, const Vector &, const Vertex &);

    void calcAngle();
    void resetAngle();
    double getAngle(TimeLevel timeLevel = NewTimeLevel) const {
        return angle.get(timeLevel);
    }
    bool isWrongAngle() const;
    bool isTangly() const;

    void dump(int indentLevel = 0) const;

	Edge *edge;
    EdgeOrient orient;
    MultiTimeLevel<double, 2> angle;
    bool isAngleSet;
};

#endif
