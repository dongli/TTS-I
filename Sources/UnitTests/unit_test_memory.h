#include "Edge.h"
#include "Vertex.h"
#include "Coordinate.h"
#include "ApproachDetector.h"

#define PRINT_SIZE(TYPE) \
{ \
    ostringstream title; \
    title << "Size of " << #TYPE << ":"; \
    cout << setw(50) << left << title.str(); \
    cout << setw(10) << right << sizeof(TYPE) << " bytes" << endl; \
}

BOOST_AUTO_TEST_SUITE(Memory_Usage_Tests)

BOOST_AUTO_TEST_CASE(Report_Memory_Usage)
{
    PRINT_SIZE(Coordinate)
    PRINT_SIZE(Vertex)
    PRINT_SIZE(VertexPointer)
    // old VertexAgent size is 104 bytes
    PRINT_SIZE(ApproachDetector::VertexAgent)
    PRINT_SIZE(Edge)
    PRINT_SIZE(EdgePointer)
    PRINT_SIZE(ApproachDetector::EdgeAgent)

    // old value is 93 bytes
    cout << sizeof(Edge *)+sizeof(Coordinate)+
    sizeof(double)+sizeof(OrientStatus)+sizeof(bool)+
    sizeof(EdgePointer *)+sizeof(Vertex *) << endl;
}

BOOST_AUTO_TEST_SUITE_END()