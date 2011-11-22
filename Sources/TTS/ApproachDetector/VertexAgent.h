#ifndef VertexAgent_h
#define VertexAgent_h

#include "Projection.h"
#include <list>

class Vertex;
class Edge;

namespace ApproachDetector
{
    class VertexAgent
    {
    public:
        VertexAgent();
        ~VertexAgent();

        void checkin(Vertex *);

        void reinit();
        void clean();

        //! \brief Record the edge that the host vertex is projected onto.
        //! \param edge The edge.
        //! \param projection Projection on the edge of the vertex.
        //! \return none.
        void recordProjection(Edge *edge, Projection *projection);

        //! \brief Remove the projection of one paired edge from the list.
        //! \param projection Projection on the edge of the vertex.
        //! \return none.
        void removeProjection(Projection *projection);

        //! \brief Expire the calculation status of projections
        //! \param none.
        //! \return none.
        void expireProjection();

        //! \brief Get the projection of one paired edge.
        //! \param edge The edge.
        //! \return The corresponding projection.
        Projection *getProjection(Edge *);

        //! \brief Select the paired edge that host vertex is approaching to.
        //! \param none.
        //! \return The corresponding projection.
        Projection *getActiveProjection();

        double getShortestDistance();

        VertexAgent &operator=(const VertexAgent &);

        void dump();

#ifdef DEBUG
        const std::list<Projection> &getProjections() const { return projections; }
#endif

    private:
        friend class EdgeAgent;

        Vertex *host;
        std::list<Projection> projections;
    };
}

#endif
