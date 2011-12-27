#ifndef Projection_h
#define Projection_h

#include "MultiTimeLevel.h"
#include "Sphere.h"

class Vertex;
class Edge;

namespace ApproachDetector
{
    enum ProjectionStatus {
        HasProjection, HasNoProjection, CrossEdge
    };

    class Projection {
    public:
        Projection();
        Projection(Vertex *, Edge *);
        Projection(const Projection &);
        ~Projection();

        void reinit();

        ProjectionStatus project(TimeLevel timeLevel);

        bool project(Vertex *vertex, Edge *edge, TimeLevel timeLevel);

        Vertex *getVertex() const { return vertex; }
        void setVertex(Vertex *vertex) { this->vertex = vertex; }

        Edge *getEdge() const { return edge; }
        void setEdge(Edge *edge) { this->edge = edge; }

        const Coordinate &getCoordinate(TimeLevel timeLevel = NewTimeLevel) const;

        double getDistance(TimeLevel timeLevel = NewTimeLevel) const;

        void setOrient(OrientStatus orient) { this->orient = orient; }
        OrientStatus getOrient() const { return orient; }

        void calcChangeRate();
        double getChangeRate() const { return changeRate; }

        bool isApproaching() const { return approach; }
        void checkApproaching();
        void setApproach(bool approach) { this->approach = approach; }

        bool isCalculated() const { return calculated; }
        void setCalculated() { calculated = true; }
        void expire() { calculated = false; }

        Projection &operator=(const Projection &);

    private:
        Vertex *vertex;
        Edge *edge;
        MultiTimeLevel<Coordinate, 2> x;
        MultiTimeLevel<double, 2> distance;
        OrientStatus orient;
        double changeRate;
        bool approach;
        bool calculated;
    };
}

#endif
