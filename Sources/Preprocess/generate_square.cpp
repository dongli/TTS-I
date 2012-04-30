#include "Constants.hpp"
#include "PolygonManager.hpp"
#include <netcdfcpp.h>

int main(void)
{
    int i, j, numVertex = 4;
    double lon[numVertex], lat[numVertex];
    double lon1 = 10.0/Rad2Deg, lon2 = 20.0/Rad2Deg;
    double lat1 = -5.0/Rad2Deg, lat2 = 5.0/Rad2Deg;

    lon[0] = lon2;
    lat[0] = lat1;
    lon[1] = lon2;
    lat[1] = lat2;
    lon[2] = lon1;
    lat[2] = lat2;
    lon[3] = lon1;
    lat[3] = lat1;

    // create a parcel with four edges
    PolygonManager polygonManager;

    polygonManager.vertices.create(numVertex);
    polygonManager.edges.create(numVertex);
    polygonManager.polygons.create(1);

    Vertex *vertices[numVertex];
    for (i = 0; i < numVertex; ++i) {
        vertices[i] = polygonManager.vertices.at(i);
        vertices[i]->setCoordinate(lon[i], lat[i]);
    }

    Edge *edges[numVertex];
    for (i = 0; i < numVertex; ++i) {
        j = i == numVertex-1 ? 0 : i+1;
        edges[i] = polygonManager.edges.at(i);
        edges[i]->linkEndPoint(FirstPoint, vertices[i]);
        edges[i]->linkEndPoint(SecondPoint, vertices[j]);
        edges[i]->linkPolygon(OrientLeft, polygonManager.polygons.front());
    }

    polygonManager.output("square.nc");

    return 0;
}
