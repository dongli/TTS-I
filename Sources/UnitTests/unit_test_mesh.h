#ifndef unit_test_mesh_h
#define unit_test_mesh_h

#include "MeshManager.h"

void test_checkLocation()
{
    int numLon = 128, numLat = 4;
    double dlon, dlat;
    double lon[numLon], lat[numLat];

    dlon = PI2/numLon;
    dlat = PI/(numLat+1);
    for (int i = 0; i < numLon; ++i)
        lon[i] = i*dlon;
    for (int j = 0; j < numLat; ++j)
        lat[j] = PI05-(j+1)*dlat;

    MeshManager meshManager;

    meshManager.init(numLon, numLat, lon, lat);
    meshManager.getMesh(Full).dump();

    Point point;
    Coordinate x;
    Location loc;

    x.setSPH(-1.4/Rad2Deg, -73.0/Rad2Deg);
    point.setCoordinate(x);

    meshManager.checkLocation(point.getCoordinate(), loc, &point);
    loc.dump();
}

#endif