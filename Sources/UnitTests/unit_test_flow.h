#include "FlowManager.h"
#include "Velocity.h"

BOOST_AUTO_TEST_SUITE(Flow_Tests)

BOOST_FIXTURE_TEST_CASE(Flow_Creation_Test, RLLMesh2D)
{
    MeshManager meshManager;
    FlowManager flowManager;

    meshManager.construct(numLon, numLat, lon, lat);
    flowManager.construct(meshManager);
}

BOOST_AUTO_TEST_CASE(Test_Velocity)
{
    Velocity velocity;
}

BOOST_AUTO_TEST_SUITE_END()
