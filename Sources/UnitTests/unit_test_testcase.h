#include "MovingVortices.h"
#include "SolidRotation.h"
#include "TimeManager.h"
#include <netcdfcpp.h>

BOOST_AUTO_TEST_SUITE(TestCases_Tests)

BOOST_FIXTURE_TEST_CASE(MovingVortices_Test, VelocityField2D)
{
    TimeManager timeManager;
    MovingVortices testCase;

    timeManager.setClock(1800.0, 1800.0*100, 100);
    testCase.calcVelocityField(flowManager);
    flowManager.output("test_moving_vortices_flow.nc");
    
    // -------------------------------------------------------------------------
    // test interpolation
    int numLon = 480, numLat = 360;
    double lon[numLon], lat[numLat];
    double dlon = PI2/numLon, dlat = PI/(numLat-1);
    Array<double, 2> u(numLat, numLon), v(numLat, numLon);
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-j*dlat;
    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            Location loc;
            Coordinate x;
            Velocity velocity;
            x.set(lon[i], lat[j]);
            meshManager.checkLocation(x, loc);
            flowManager.getVelocity(x, loc, NewTimeLevel,
                                    velocity, Velocity::LonLatSpace);
            u(j, i) = velocity.u;
            v(j, i) = velocity.v;
        }
    for (int i = 0; i < numLon; ++i)
        lon[i] *= Rad2Deg;
    for (int j = 0; j < numLat; ++j)
        lat[j] *= Rad2Deg;
    NcFile file("test_moving_vortices_flow_interp.nc", NcFile::Replace);
    NcDim *lonDim = file.add_dim("lon", numLon);
    NcDim *latDim = file.add_dim("lat", numLat);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    lonVar->add_att("long_name", "longitude");
    lonVar->add_att("units", "degree_east");
    latVar->add_att("long_name", "latitude");
    latVar->add_att("units", "degree_north");
    lonVar->put(lon, numLon);
    latVar->put(lat, numLat);
    NcVar *uVar = file.add_var("u", ncDouble, latDim, lonDim);
    NcVar *vVar = file.add_var("v", ncDouble, latDim, lonDim);
    uVar->put(u.data(), numLat, numLon);
    vVar->put(v.data(), numLat, numLon);
    file.close();
}

BOOST_FIXTURE_TEST_CASE(SolidRotation_Test, VelocityField2D)
{
    TimeManager timeManager;
    SolidRotation testCase;

    timeManager.setClock(1800.0);
    testCase.calcVelocityField(flowManager);
    flowManager.output("test_solid_rotation_flow.nc");

    // -------------------------------------------------------------------------
    // test interpolation
    int numLon = 480, numLat = 360;
    double lon[numLon], lat[numLat];
    double dlon = PI2/numLon, dlat = PI/(numLat-1);
    Array<double, 2> u(numLat, numLon), v(numLat, numLon);
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-j*dlat;
    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            Location loc;
            Coordinate x;
            Velocity velocity;
            x.set(lon[i], lat[j]);
            meshManager.checkLocation(x, loc);
            flowManager.getVelocity(x, loc, NewTimeLevel,
                                    velocity, Velocity::LonLatSpace);
            u(j, i) = velocity.u;
            v(j, i) = velocity.v;
        }
    for (int i = 0; i < numLon; ++i)
        lon[i] *= Rad2Deg;
    for (int j = 0; j < numLat; ++j)
        lat[j] *= Rad2Deg;
    NcFile file("test_solid_rotation_flow_interp.nc", NcFile::Replace);
    NcDim *lonDim = file.add_dim("lon", numLon);
    NcDim *latDim = file.add_dim("lat", numLat);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    lonVar->add_att("long_name", "longitude");
    lonVar->add_att("units", "degree_east");
    latVar->add_att("long_name", "latitude");
    latVar->add_att("units", "degree_north");
    lonVar->put(lon, numLon);
    latVar->put(lat, numLat);
    NcVar *uVar = file.add_var("u", ncDouble, latDim, lonDim);
    NcVar *vVar = file.add_var("v", ncDouble, latDim, lonDim);
    uVar->put(u.data(), numLat, numLon);
    vVar->put(v.data(), numLat, numLon);
    file.close();
}

BOOST_FIXTURE_TEST_CASE(StaticVortices_Test, VelocityField2D)
{
    TimeManager timeManager;
    SolidRotation testCase;

    timeManager.setClock(1800.0);
    testCase.calcVelocityField(flowManager);
    flowManager.output("test_static_vortices_flow.nc");

    int numLon = 480, numLat = 360;
    double lon[numLon], lat[numLat];
    double dlon = PI2/numLon, dlat = PI/(numLat-1);
    Array<double, 2> u(numLat, numLon), v(numLat, numLon);
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-j*dlat;
    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            Location loc;
            Coordinate x;
            Velocity velocity;
            x.set(lon[i], lat[j]);
            meshManager.checkLocation(x, loc);
            flowManager.getVelocity(x, loc, NewTimeLevel,
                                    velocity, Velocity::LonLatSpace);
            u(j, i) = velocity.u;
            v(j, i) = velocity.v;
        }
    for (int i = 0; i < numLon; ++i)
        lon[i] *= Rad2Deg;
    for (int j = 0; j < numLat; ++j)
        lat[j] *= Rad2Deg;
    NcFile file("test_static_vortices_flow_interp.nc", NcFile::Replace);
    NcDim *lonDim = file.add_dim("lon", numLon);
    NcDim *latDim = file.add_dim("lat", numLat);
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    lonVar->add_att("long_name", "longitude");
    lonVar->add_att("units", "degree_east");
    latVar->add_att("long_name", "latitude");
    latVar->add_att("units", "degree_north");
    lonVar->put(lon, numLon);
    latVar->put(lat, numLat);
    NcVar *uVar = file.add_var("u", ncDouble, latDim, lonDim);
    NcVar *vVar = file.add_var("v", ncDouble, latDim, lonDim);
    uVar->put(u.data(), numLat, numLon);
    vVar->put(v.data(), numLat, numLon);
    file.close();
}

BOOST_AUTO_TEST_SUITE_END()
