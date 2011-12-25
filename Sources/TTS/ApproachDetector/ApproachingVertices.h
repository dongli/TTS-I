#ifndef ApproachingVertices_h
#define ApproachingVertices_h

#include <list>

class Vertex;

namespace ApproachDetector
{
    class ApproachingVertices
    {
    public:
        ApproachingVertices();
        ~ApproachingVertices();

        static void recordVertex(Vertex *vertex);

        static void removeVertex(Vertex *vertex);

        static void jumpVertex(Vertex *vertex1, Vertex *vertex2);

        static bool isEmpty() { return vertices.size() == 0 ? true : false; }

        static void dump();

        static std::list<Vertex *> vertices;
    };
}

#endif
