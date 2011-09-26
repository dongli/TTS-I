#ifndef Projection_h
#define Projection_h

#include "MultiTimeLevel.h"
#include "Sphere.h"

class Vertex;
class Edge;

namespace ApproachDetector
{
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
        bool project(TimeLevel timeLevel);

        //! \brief Calculate the projection of any vertex onto any edge.
        //! \param vertex The vertex.
        //! \param edge The edge.
        //! \param timeLevel The time level.
        //! \return The boolean status of the projection
        //!         - true (inside edge)
        //!         - false (outside edge, should be unpaired with the edge)
        bool project(Vertex *vertex, Edge *edge, TimeLevel timeLevel);

        Vertex *getVertex() { return vertex; }
        void setVertex(Vertex *vertex) { this->vertex = vertex; }

        Edge *getEdge() { return edge; }
        void setEdge(Edge *edge) { this->edge = edge; }

        const Coordinate &getCoordinate(TimeLevel timeLevel);

        double getDistance(TimeLevel timeLevel);

        OrientStatus getOrient() { return orient; }

        bool isApproaching() { return approach; }
        void checkApproaching();

        //! \brief Check the validation of data to avoid repeated calculation.
        //! \param none.
        //! \return The boolean status.
        bool isCalculated() { return calculated; }

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
        bool approach;
        bool calculated;
    };
}

#endif
