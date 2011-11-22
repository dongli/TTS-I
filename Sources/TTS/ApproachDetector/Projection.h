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

        //! \brief Calculate the projection of the vertex onto its paired edge.
        //! \param timeLevel The time level.
        //! \return The boolean status of the projection
        //!         - true (inside edge)
        //!         - false (outside edge, should be unpaired with the edge)
        ProjectionStatus project(TimeLevel timeLevel);

        //! \brief Calculate the projection of any vertex onto any edge.
        //! \param vertex The vertex.
        //! \param edge The edge.
        //! \param timeLevel The time level.
        //! \return The boolean status of the projection
        //!         - true (inside edge)
        //!         - false (outside edge, should be unpaired with the edge)
        bool project(Vertex *vertex, Edge *edge, TimeLevel timeLevel);

        Vertex *getVertex() const { return vertex; }
        void setVertex(Vertex *vertex) { this->vertex = vertex; }

        Edge *getEdge() const { return edge; }
        void setEdge(Edge *edge) { this->edge = edge; }

        const Coordinate &getCoordinate(TimeLevel timeLevel) const;

        double getDistance(TimeLevel timeLevel);

        OrientStatus getOrient() { return orient; }

        void calcChangeRate();
        double getChangeRate() const { return changeRate; }

        bool isApproaching() const { return approach; }
        void checkApproaching();
        void setApproach(bool approach) { this->approach = approach; }

        //! \brief Check the validation of data to avoid repeated calculation.
        //! \param none.
        //! \return The boolean status.
        bool isCalculated() const { return calculated; }

        //! \brief Validate the data.
        //! \param none.
        //! \return none.
        void setCalculated() { calculated = true; }
        
        //! \brief Expire the data.
        //! \param none.
        //! \return none.
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
