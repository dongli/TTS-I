#ifndef EdgeAgent_h
#define EdgeAgent_h

#include <list>

class MeshManager;
class FlowManager;
class PolygonManager;
class Vertex;
class Edge;

namespace ApproachDetector
{
    class EdgeAgent
    {
    public:
        EdgeAgent();
        virtual ~EdgeAgent();

        void checkin(Edge *);
    
        void reinit();
        void clean();

        void recordVertex(Vertex *vertex);

        void removeVertex(Vertex *vertex);
        void removeVertex(std::list<Vertex *>::iterator &it);

        void updateVertexProjections(MeshManager &meshManager);

        void handoverVertices(Edge *edge);

        void dump();

        std::list<Vertex *> vertices;

    private:
        friend class VertexAgent;

        Edge *host;
    };
}

#endif
