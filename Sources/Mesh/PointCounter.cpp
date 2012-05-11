#include "PointCounter.hpp"
#include "ReportMacros.hpp"
#include "Constants.hpp"
#include "Sphere.hpp"
#include <netcdfcpp.h>

PointCounter::PointCounter()
{
    REPORT_ONLINE("PointCounter")
}

PointCounter::~PointCounter()
{
    REPORT_OFFLINE("PointCounter")
}

void PointCounter::init(const Array<double, 1> &lon, const Array<double, 1> &lat,
                        int numSubLon, int numSubLat)
{
    this->numSubLon = numSubLon; this->numSubLat = numSubLat;
    // -------------------------------------------------------------------------
    // bounds of cells for counting points
    int numLon = (lon.size()-2)*numSubLon;
    int numLat = lat.size()*numSubLat+1;
    double lonBnds[numLon], latBnds[numLat];
    int I = 0;
    for (int i = 1; i < lon.size(); ++i) {
        double dlon = (lon(i)-lon(i-1))/numSubLon;
        int k1, k2;
        if (i == 1) {
            k1 = ceil(numSubLon/2.0);
            k2 = numSubLon;
        } else if (i == lon.size()-1) {
            k1 = 0;
            k2 = ceil(numSubLon/2.0);
        } else {
            k1 = 0;
            k2 = numSubLon;
        }
        for (int k = k1; k < k2; ++k)
            lonBnds[I++] = lon(i-1)+k*dlon;
    }
    // Note: Point counter mesh includes poles.
    latBnds[0] = PI05;
    I = 1;
    for (int j = 0; j < lat.size()-1; ++j) {
        double dlat = (lat(j)-lat(j+1))/numSubLat;
        for (int k = 0; k < numSubLat; ++k)
            latBnds[I++] = lat(j)-k*dlat;
    }
    latBnds[I] = lat(lat.size()-1);
    latBnds[numLat-1] = -PI05;
    MeshSpec meshSpec;
    meshSpec.type = Full;
    meshSpec.isWithPoles = true;
    meshSpec.isAreaFit = false;
    mesh[0].init(meshSpec, numLon, numLat, lonBnds, latBnds);
#ifndef OUTPUT_POINT_COUNTER_MESH
    mesh[0].output("point_counter_bound_mesh.nc");
#endif
    // -------------------------------------------------------------------------
    // centers of cells
    numLon = mesh[0].lon.size()-2, numLat = mesh[0].lat.size()-1;
    double lonCnts[numLon], latCnts[numLat];
    for (int i = 0; i < numLon; ++i)
        lonCnts[i] = (mesh[0].lon(i)+mesh[0].lon(i+1))*0.5;
    for (int j = 0; j < numLat; ++j)
        latCnts[j] = (mesh[0].lat(j)+mesh[0].lat(j+1))*0.5;
    meshSpec.type = Full;
    meshSpec.isWithPoles = false;
    meshSpec.isAreaFit = true;
    mesh[1].init(meshSpec, numLon, numLat, lonCnts, latCnts);
#ifndef OUTPUT_POINT_COUNTER_MESH
    mesh[1].output("point_counter_center_mesh.nc");
#endif
    // -------------------------------------------------------------------------
    // TODO: Add the vertical codes.
    counters.resize(mesh[1].getNumLon()-2, mesh[1].getNumLat(), 1);
    points.resize(counters.shape());
    for (int i = 0; i < points.extent(0); ++i)
        for (int j = 0; j < points.extent(1); ++j)
            points(i, j, 0).resize(10);
    reset();
}

void PointCounter::reset()
{
    counters = 0;
}

void PointCounter::count(const Location &loc, Point *point)
{
    int l = counters(loc.i.back(), loc.j.back(), loc.k)++;
    if (points(loc.i.back(), loc.j.back(), loc.k).size() <= l)
        points(loc.i.back(), loc.j.back(), loc.k).resizeAndPreserve(l+10);
    points(loc.i.back(), loc.j.back(), loc.k)(l) = point;
}

void PointCounter::output(const string &fileName) const
{
    NcFile file(fileName.c_str(), NcFile::Replace);
    if (!file.is_valid()) {
        string message = "Failed to open file "+fileName+".";
        REPORT_ERROR(message.c_str());
    }

    NcDim *numLonDim = file.add_dim("lon", counters.extent(0));
    NcDim *numLatDim = file.add_dim("lat", counters.extent(1));
    NcDim *numBndsDim = file.add_dim("bnds", 2);
    //NcDim *numLevDim = file.add_dim("lev", counters.extent(2));

    NcVar *lonBndsVar = file.add_var("lon_bnds", ncDouble, numLonDim, numBndsDim);
    NcVar *latBndsVar = file.add_var("lat_bnds", ncDouble, numLatDim, numBndsDim);
    //NcVar *levVar = file.add_var("lev", ncDouble, numLevDim);

    lonBndsVar->add_att("long_name", "longitude bounds");
    lonBndsVar->add_att("units", "degree_east");
    latBndsVar->add_att("long_name", "latitude bounds");
    latBndsVar->add_att("units", "degree_north");

    int numLonBnds = mesh[0].getNumLon()-2, numLatBnds = mesh[0].getNumLat()-1;
    double lonBnds[numLonBnds][2], latBnds[numLatBnds][2];
    for (int i = 0; i < numLonBnds; ++i) {
        lonBnds[i][0] = mesh[0].lon(i)*Rad2Deg;
        lonBnds[i][1] = mesh[0].lon(i+1)*Rad2Deg;
    }
    for (int j = 0; j < numLatBnds; ++j) {
        latBnds[j][0] = mesh[0].lat(j)*Rad2Deg;
        latBnds[j][1] = mesh[0].lat(j+1)*Rad2Deg;
    }

    lonBndsVar->put(&lonBnds[0][0], numLonBnds, 2);
    latBndsVar->put(&latBnds[0][0], numLatBnds, 2);

    NcVar *countersVar = file.add_var("counters", ncInt, numLatDim, numLonDim);

    countersVar->add_att("long_name", "contained point numbers in grid boxes");

    int counters[this->counters.extent(1)][this->counters.extent(0)];
    for (int j = 0; j < this->counters.extent(1); ++j)
        for (int i = 0; i < this->counters.extent(0); ++i)
            counters[j][i] = this->counters(i, j, 0);
    countersVar->put(&counters[0][0], this->counters.extent(1), this->counters.extent(0));
}
