#ifndef EdgeAgent_h
#define EdgeAgent_h

#include <list>

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

        //! \brief Record the approaching vertex into list.
        //! \param vertex a vertex that is appraoching the host edge.
        //! \return none.
        void recordVertex(Vertex *vertex);

        //! \brief Remove the recorded vertex from the list.
        //! \param vertex a vertex that will be removed.
        //! \return none.
        void removeVertex(Vertex *vertex);
        void removeVertex(std::list<Vertex *>::iterator &it);

        //! \brief Update the projection of each recorded vertex.
        //! \param none.
        //! \param none.
        //! \return none.
        void updateVertexProjections();

        //! \brief Hand over the approaching vertices to another edge.
        //! \param edge Successive edge.
        //! return none.
        void handoverVertices(Edge *edge);

        void dump();

        std::list<Vertex *> vertices;

    private:
        friend class VertexAgent;

        Edge *host;
    };
}

#endif
