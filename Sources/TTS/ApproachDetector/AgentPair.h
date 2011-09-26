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
        //! \brief Pair the vertex and edge with given projection.
        //! \param vertex The vertex.
        //! \param edge The edge.
        //! \param projection The calculated projection.
        //! \return none.
        void pair(Vertex *vertex, Edge *edge, Projection *projection);

        //! \brief Unpair the vertex and edge.
        //! \param vertex The vertex.
        //! \param edge The edge.
        //! \return none.
        void unpair(Vertex *vertex, Edge *edge);

        //! \brief Unpair the vertex and edge, but use more information.
        //! \param it Vertex iterator in the approaching vertices list of the edge.
        //! \param edge The edge.
        //! \return none.
        void unpair(std::list<Vertex *>::iterator &it, Edge *edge);

        //! \brief Unpair the vertex and edge, but use more information.
        //! \param it Vertex iterator in the approaching vertices list of the edge.
        //! \param projection Projection on the edge of the vertex.
        //! \return none.
        void unpair(std::list<Vertex *>::iterator &it, Projection *projection);
    }
}

#endif
