#include "RLLMesh.h"
#include "Field.h"

BOOST_AUTO_TEST_SUITE(Field_Tests)

BOOST_FIXTURE_TEST_CASE(Field_Creation_Test, RLLMesh2D)
{
    RLLMesh mesh1(RLLMesh::LonHalf, numLon, numLat, lon, lat);
    Field field1;
    RLLMesh mesh2(RLLMesh::LatHalf, numLon, numLat, lon, lat);
    Field field2;

    field1.construct(mesh1);
    field2.construct(mesh2);
    BOOST_REQUIRE(field1.mesh->getNumLon() == numLon+2);
    BOOST_REQUIRE(field1.mesh->getNumLat() == numLat);
    BOOST_REQUIRE(field1.values.size() == (numLon+2)*numLat);
    BOOST_REQUIRE(field2.mesh->getNumLon() == numLon+1);
    BOOST_REQUIRE(field2.mesh->getNumLat() == numLat+1);
    BOOST_REQUIRE(field2.values.size() == (numLon+1)*(numLat+1));

    // check the periodic boundary conditions
    for (int i = 1; i < mesh1.getNumLon()-1; ++i)
        for (int j = 0; j < mesh1.getNumLat(); ++j) {
            double value = j*mesh1.getNumLon()+i+1.0;
            field1.values(i, j, 0).setNew(value);
        }
    for (int j = 0; j < mesh1.getNumLat(); ++j) {
        BOOST_REQUIRE(field1.values(0, j, 0) ==
                      field1.values(mesh1.getNumLon()-2, j, 0));
        BOOST_REQUIRE(field1.values(mesh1.getNumLon()-1, j, 0) ==
                      field1.values(1, j, 0));
    }
    for (int i = 0; i < mesh2.getNumLon()-1; ++i)
        for (int j = 0; j < mesh2.getNumLat(); ++j) {
            double value = j*mesh2.getNumLon()+i+1.0;
            field2.values(i, j, 0).setNew(value);
        }
    for (int j = 0; j < mesh2.getNumLat(); ++j) {
        BOOST_REQUIRE(field2.values(mesh2.getNumLon()-1, j, 0) ==
                      field2.values(0, j, 0));
    }

    BOOST_CHECK(field1.values(0, 0, 0).getOld() !=
                field1.values(0, 0, 0).getNew());
    field1.values(0, 0, 0).save();
    BOOST_REQUIRE(field1.values(0, 0, 0).getOld() ==
                  field1.values(0, 0, 0).getNew());
}

BOOST_AUTO_TEST_SUITE_END()
