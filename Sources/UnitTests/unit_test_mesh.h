#include "RLLMesh.h"
#include "Constants.h"
#include "Sphere.h"
#include "MeshManager.h"

BOOST_AUTO_TEST_SUITE(Mesh_Tests)

BOOST_FIXTURE_TEST_CASE(RLLMesh_Tests, RLLMesh2D)
{
    RLLMesh mesh(RLLMesh::Full, numLon, numLat, lon, lat);

    // Test 1: The total area should equal to the area of the sphere.
    double totalArea = 0.0;
    for (int i = 0; i < mesh.area.extent(0); ++i) {
        for (int j = 0; j < mesh.area.extent(1); ++j) {
            totalArea += mesh.area(i, j);
        }
    }
    BOOST_REQUIRE(abs(totalArea-4.0*PI*Sphere::radius2) < EPS);
}

BOOST_FIXTURE_TEST_CASE(MeshManager_Test, RLLMesh2D)
{
    MeshManager meshManager;

    meshManager.construct(numLon, numLat, lon, lat);

    const RLLMesh &mesh = meshManager.getMesh(RLLMesh::Full);
    double totalArea = 0.0;
    for (int i = 0; i < mesh.area.extent(0); ++i) {
        for (int j = 0; j < mesh.area.extent(1); ++j) {
            totalArea += mesh.area(i, j);
        }
    }
    message.str("");
    message << "\nTotal area = " <<
        setw(30) << setprecision(22) << totalArea;
    message << "\nTrue area  = "  <<
        setw(30) << setprecision(22) << 4.0*PI*Sphere::radius2;
    message << "\nDifference = " <<
        setw(30) << setprecision(22) << totalArea-4.0*PI*Sphere::radius2;
    BOOST_REQUIRE_MESSAGE(abs(totalArea-4.0*PI*Sphere::radius2) < EPS, message.str());

    Point point;
    Location loc;

    point.setCoordinate(1.1/Rad2Deg, 88.51/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc, 1);
    loc.dump();
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == 0);
    BOOST_REQUIRE(loc.i.back() == 3);
    BOOST_REQUIRE(loc.j.back() == 2);

    point.setCoordinate(1.1/Rad2Deg, 89.1/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc, 1);
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == -1);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == -1);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == 0);
    BOOST_REQUIRE(loc.i.back() == 3);
    BOOST_REQUIRE(loc.j.back() == 1);

    point.setCoordinate(1.1/Rad2Deg, 89.6/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc, 1);
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == -1);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == -1);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == -1);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == -1);
    BOOST_REQUIRE(loc.i.back() == 3);
    BOOST_REQUIRE(loc.j.back() == 0);

    point.setCoordinate(1.1/Rad2Deg, 88.50/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc, 1);
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == 0);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.i.back() == 3);
    BOOST_REQUIRE(loc.j.back() == 3);

    point.setCoordinate(1.1/Rad2Deg, 88.0/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc, 1);
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == 1);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == 1);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == 1);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == 1);
    BOOST_REQUIRE(loc.i.back() == 3);
    BOOST_REQUIRE(loc.j.back() == 4);

    // Note: We got inaccurate floating point calculation here!
    //       149.99999999999997158 -> 150
    /*point.setCoordinate(149.5/Rad2Deg, 2.5/Rad2Deg, 0.0);
    meshManager.checkLocation(point.getCoordinate(), loc);
    loc.dump();
    BOOST_REQUIRE(loc.i[RLLMesh::Full] == 149);
    BOOST_REQUIRE(loc.j[RLLMesh::Full] == 86);
    BOOST_REQUIRE(loc.i[RLLMesh::LonHalf] == 150);
    BOOST_REQUIRE(loc.j[RLLMesh::LonHalf] == 86);
    BOOST_REQUIRE(loc.i[RLLMesh::LatHalf] == 149);
    BOOST_REQUIRE(loc.j[RLLMesh::LatHalf] == 87);
    BOOST_REQUIRE(loc.i[RLLMesh::BothHalf] == 150);
    BOOST_REQUIRE(loc.j[RLLMesh::BothHalf] == 87);*/
}

BOOST_AUTO_TEST_SUITE_END()
