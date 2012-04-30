#ifndef VertexAgent_h
#define VertexAgent_h

#include "Projection.hpp"
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

        void recordProjection(Edge *edge, Projection *projection);

        void removeProjection(Projection *projection);
        void removeProjection(std::list<Projection>::iterator &it);

        void expireProjection();

        Projection *getProjection(Edge *);
        Projection *getActiveProjection();

        bool isCrossing();

        double getShortestDistance();

        VertexAgent &operator=(const VertexAgent &);

        void dump();

        std::list<Projection> &getProjections() { return projections; }

    private:
        friend class EdgeAgent;

        Vertex *host;
        std::list<Projection> projections;
    };
}

#endif
