#include "PolygonManager.h"
#include "Constants.h"

BOOST_AUTO_TEST_SUITE(Polygon_Tests)

BOOST_AUTO_TEST_CASE(NormVector_Test)
{
    Edge edge;
    Vertex v1, v2;
    v1.setCoordinate(0.0, PI05);
    v2.setCoordinate(PI05, 0.0);
    edge.linkEndPoint(FirstPoint, &v1);
    edge.linkEndPoint(SecondPoint, &v2);
    edge.calcNormVector();
    BOOST_REQUIRE(fabs(edge.getNormVector()(0)-1.0) < EPS);
    BOOST_REQUIRE(fabs(edge.getNormVector()(1)-0.0) < EPS);
    BOOST_REQUIRE(fabs(edge.getNormVector()(2)-0.0) < EPS);
}

BOOST_AUTO_TEST_CASE(Angle_Test)
{
    Edge edge1, edge2;
    Vertex v1, v2;
    Polygon polygon;

    // -------------------------------------------------------------------------
    v1.setCoordinate(0.0, 0.0);
    v2.setCoordinate(PI05, 0.0);
    edge1.linkEndPoint(FirstPoint, &v1);
    edge1.linkEndPoint(SecondPoint, &v2);
    edge1.calcNormVector();

    // -------------------------------------------------------------------------
    v1.setCoordinate(0.0, 0.0);
    v2.setCoordinate(0.0, PI05);
    edge2.linkEndPoint(FirstPoint, &v1);
    edge2.linkEndPoint(SecondPoint, &v2);
    edge2.calcNormVector();

    // -------------------------------------------------------------------------
    edge1.linkPolygon(EdgeRight, &polygon);
    edge2.linkPolygon(EdgeLeft, &polygon);

    // -------------------------------------------------------------------------
    EdgePointer *edgePointer = polygon.edgePointers.at(1);
    edgePointer->calcAngle();
    BOOST_REQUIRE(fabs(edgePointer->getAngle()-PI*1.5) < EPS);
}

BOOST_AUTO_TEST_CASE(TestPoint_Test)
{
    Edge edge;
    Vertex v1, v2;
    v1.setCoordinate(0.0, PI05);
    v2.setCoordinate(0.0, 0.0);
    edge.linkEndPoint(FirstPoint, &v1);
    edge.linkEndPoint(SecondPoint, &v2);
    Vertex *testPoint = edge.getTestPoint();
    BOOST_REQUIRE(fabs(testPoint->getCoordinate(OldTimeLevel).getLon()-0.0) < EPS);
    BOOST_REQUIRE(fabs(testPoint->getCoordinate(OldTimeLevel).getLat()-PI*0.25) < EPS);
}

BOOST_AUTO_TEST_SUITE_END()
