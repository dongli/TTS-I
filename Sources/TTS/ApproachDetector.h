#ifndef _EdgeApproachProbe_h_
#define _EdgeApproachProbe_h_

#include "Sphere.h"

class Edge;
class EdgePointer;
class Vertex;
class VertexPointer;
class Polygon;
class MeshManager;
class FlowManager;
class PolygonManager;

namespace ApproachDetector
{
    // -------------------------------------------------------------------------
    class VertexAgent
    {
    public:
        VertexAgent();
        virtual ~VertexAgent();

        void checkin(Vertex *);

        void reinit();
        void clean();

        Edge *edge;
        Coordinate projection;
        double distance;
        OrientStatus orient;
        bool isApproaching;
        EdgePointer *edgePointer;

    private:
        friend class EdgeAgent;

        Vertex *host;
    };

    // -------------------------------------------------------------------------
    class EdgeAgent
    {
    public:
        EdgeAgent();
        virtual ~EdgeAgent();

        void checkin(Edge *);
    
        void reinit();
        void clean();

        void record(Vertex *);
        void remove(Vertex *);
        void handover(Edge *);
        void update();
        void changeEdgePointer();

        List<VertexPointer> vertexPointers;

    private:
        friend class VertexAgent;

        Edge *host;
    };

    // -------------------------------------------------------------------------
    namespace AgentPairs
    {
        void pair(Vertex *, Edge *, const Coordinate &, double, OrientStatus);
        void unpair(Vertex *, Edge *);
    }
    
    // -------------------------------------------------------------------------
    class ApproachingVertices
    {
    public:
        ApproachingVertices();
        virtual ~ApproachingVertices();

        static void jumpFirst(Vertex *);
        static void record(Vertex *);
        static void record(Vertex *, Vertex *);
        static void remove(Vertex *);
        static bool isEmpty();

        static void dump();

        static List<VertexPointer> vertexPointers;
    };

    // -------------------------------------------------------------------------
    bool isActive(double distance);
    bool isApproaching(double oldDistance, double newDistance);
    double approachTrendThreshold(double distance);
    void detect(MeshManager &, const FlowManager &, Polygon *);
    void detect(MeshManager &, const FlowManager &, PolygonManager &);
}

#endif
