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

        //! \brief Record the vertex into the list that will be processed
        //!        in TTS::splitPolygon method.
        //! \param vertex The vertex.
        //! \return none.
        static void recordVertex(Vertex *vertex);

        //! \brief Remove the vertex from the list that will be processed
        //!        in TTS::splitPolygon method.
        //! \param vertex The vertex.
        //! \return none.
        static void removeVertex(Vertex *vertex);

        //! \brief Move one vertex ahead the other one.
        //! \param vertex1 The first vertex to be jumped over.
        //! \param vertex2 The vertex that will be moved.
        //! \return none.
        static void jumpVertex(Vertex *vertex1, Vertex *vertex2);

        static bool isEmpty() { return vertices.size() == 0 ? true : false; }

        static void dump();

        static std::list<Vertex *> vertices;
    };
}

#endif
