#ifndef PotentialCrossDetector_h
#define PotentialCrossDetector_h

class Vertex;
class Edge;
class Polygon;

namespace PotentialCrossDetector
{
    enum Status {
        NoCross, Cross
    };

    //! \brief Detect the potential edge-crossing event (CASE 1).
    //! \param vertex3 The approaching vertex.
    //! \param testVertex The vertex that will replace vertex3.
    //! \param edge1 The edge that is being approached by vertex3.
    //! \return Detection result.
    Status detect1(Vertex *vertex3, Vertex *testVertex);

    Status detect2(Vertex *testVertex, Edge *edge1, Polygon *polygon2);

    Status detect3(Edge *edge1, Edge *edge2);
}

#endif
