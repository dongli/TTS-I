#include <netcdfcpp.h>
#include <cmath>
#include <sstream>

using namespace std;

static const int numLon = 128;
static const int numLat = 60;
static const bool hasPoles = false;
static const double PI = 4.0*atan(1.0);
static const double PI05 = PI*0.5;
static const double PI2 = PI*2.0;
static const double radius = 6371.229e3;
static const double radius2 = radius*radius;

int main(void)
{
    int numGrid;
    if (hasPoles) {
        numGrid = numLon*numLat+2;
    } else {
        numGrid = numLon*numLat;
    }
    double dlon, dlat, dlon05, dlat05;
    double lon[numGrid], lat[numGrid], volume[numGrid];

    dlon = PI2/numLon;
    dlat = PI/numLat;
    dlon05 = dlon*0.5;
    dlat05 = dlat*0.5;

    if (hasPoles) {
        lon[0] = 0.0;
        lat[0] = PI05;
    }
    for (int j = 0; j < numLat; ++j)
        for (int i = 0; i < numLon; ++i) {
            int k;
            if (hasPoles) {
                k = j*numLon+i+1;
            } else {
                k = j*numLon+i;
            }
            lon[k] = dlon05+i*dlon;
            lat[k] = PI05-dlat05-j*dlat;
            volume[k] = radius2*dlon*(sin(lat[k]+dlat05)-sin(lat[k]-dlat05));
        }
    if (hasPoles) {
        lon[numGrid-1] = 0.0;
        lat[numGrid-1] = -PI05;
    }

    // Verify the total volume
    double totalVolume = 0.0;
    for (int i = 0; i < numGrid; ++i)
        totalVolume += volume[i];
    cout << "Difference from PI: " << totalVolume/radius2/4.0-PI << endl;

    ostringstream fileName;
    fileName << "RLLMesh_" << numLon << "x" << numLat << ".nc";
    NcFile file(fileName.str().c_str(), NcFile::Replace);
    NcDim *numDim = file.add_dim("grid_size", numGrid);
    NcVar *lonVar = file.add_var("grid_center_lon", ncDouble, numDim);
    NcVar *latVar = file.add_var("grid_center_lat", ncDouble, numDim);
    NcVar *volVar = file.add_var("grid_volume", ncDouble, numDim);

    lonVar->put(lon, numGrid);
    latVar->put(lat, numGrid);
    volVar->put(volume, numGrid);

    file.close();

    return 0;
}
