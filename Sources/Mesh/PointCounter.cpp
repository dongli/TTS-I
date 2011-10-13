#include "PointCounter.h"
#include "ReportMacros.h"
#include "Constants.h"
#include "Location.h"
#include "Point.h"
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
    // -------------------------------------------------------------------------
    // bounds of grid boxes for counting points
    this->numSubLon = numSubLon;
    this->numSubLat = numSubLat;
    lonBnds.resize((lon.size()-2)*numSubLon+1);
    for (int i = 0; i < lon.size()-2; ++i) {
        double dlon = (lon(i+1)-lon(i))/numSubLon;
        for (int k = 0; k < numSubLon; ++k)
            lonBnds(i*numSubLon+k) = lon(i)+k*dlon;
    }
    lonBnds(lonBnds.size()-1) = lon(lon.size()-2);
    latBnds.resize((lat.size()-1)*numSubLat+1+2);
    latBnds(0) = PI05;
    for (int j = 0; j < lat.size()-1; ++j) {
        double dlat = (lat(j)-lat(j+1))/numSubLat;
        for (int k = 0; k < numSubLat; ++k)
            latBnds(1+j*numSubLat+k) = lat(j)-k*dlat;
    }
    latBnds(latBnds.size()-2) = lat(lat.size()-1);
    latBnds(latBnds.size()-1) = -PI05;
    // -------------------------------------------------------------------------
    //! \todo Add the vertical codes.
    counters.resize(lonBnds.size()-1, latBnds.size()-1, 1);
    points.resize(lonBnds.size()-1, latBnds.size()-1, 1);
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

    double lonBnds[this->lonBnds.size()-1][2], latBnds[this->latBnds.size()-1][2];
    for (int i = 0; i < this->lonBnds.size()-1; ++i) {
        lonBnds[i][0] = this->lonBnds(i)*Rad2Deg;
        lonBnds[i][1] = this->lonBnds(i+1)*Rad2Deg;
    }
    for (int j = 0; j < this->latBnds.size()-1; ++j) {
        latBnds[j][0] = this->latBnds(j)*Rad2Deg;
        latBnds[j][1] = this->latBnds(j+1)*Rad2Deg;
    }

    lonBndsVar->put(&lonBnds[0][0], this->lonBnds.size()-1, 2);
    latBndsVar->put(&latBnds[0][0], this->latBnds.size()-1, 2);

    NcVar *countersVar = file.add_var("counters", ncInt, numLatDim, numLonDim);

    countersVar->add_att("long_name", "contained point numbers in grid boxes");

    int counters[this->counters.extent(1)][this->counters.extent(0)];
    for (int j = 0; j < this->counters.extent(1); ++j)
        for (int i = 0; i < this->counters.extent(0); ++i)
            counters[j][i] = this->counters(i, j, 0);
    countersVar->put(&counters[0][0], this->counters.extent(1), this->counters.extent(0));
}
