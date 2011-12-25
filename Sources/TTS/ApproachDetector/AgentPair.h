#ifndef AgentPair_h
#define AgentPair_h

#include "Projection.h"
#include <list>

class Vertex;
class Edge;

namespace ApproachDetector
{
    namespace AgentPair
    {
        void pair(Vertex *vertex, Edge *edge, Projection *projection);

        void unpair(Vertex *vertex, Edge *edge);

        void unpair(std::list<Vertex *>::iterator &it, Edge *edge);

        void unpair(std::list<Vertex *>::iterator &it, Projection *projection);

        void unpair(std::list<Projection>::iterator &it);
    }
}

#endif
