#include "RLLMesh.h"
#include "Constants.h"
#include "Sphere.h"
#include "ReportMacros.h"
#include <iostream>
#include <iomanip>

using std::cout;
using std::endl;
using std::setw;
using std::setprecision;

RLLMesh::RLLMesh()
{
    isConstructed = false;
}

RLLMesh::RLLMesh(MeshType type, int numLon, int numLat,
                 double *lon, double *lat)
{
    isConstructed = false;
    init(type, numLon, numLat, lon, lat);
}

RLLMesh::~RLLMesh()
{
}

void RLLMesh::init(MeshType type, int numLon, int numLat,
                   double *lon, double *lat)
{
    if (isConstructed) {
        REPORT_ERROR("RLL mesh has already been created.")
    }
	// Note: Input "lon" and "lat" are of FULL mesh
    this->type = type;
	// -------------------------------------------------------------------------
	// longitude grids:
	// There are several assumptions as following:
	//   1) The longitude grids are equidistance;
	//   2) Input "lon" is not periodic;
	dlon = lon[1]-lon[0];
    if (type == Full || type == LatHalf) {
		this->lon.resize(numLon+1);
		for (int i = 0; i < this->lon.size()-1; ++i)
			this->lon(i) = lon[i];
        this->lon(this->lon.size()-1) = lon[numLon-1]+dlon;
    } else if (type == LonHalf || type == BothHalf) {
		this->lon.resize(numLon+2);
		for (int i = 1; i < this->lon.size()-1; ++i)
			this->lon(i) = lon[i-1]+dlon*0.5;
        this->lon(0) = this->lon(1)-dlon;;
		this->lon(this->lon.size()-1) = this->lon(this->lon.size()-2)+dlon;
    }
	// -------------------------------------------------------------------------
	// latitude grids:
	// There are several assumptions as following:
	//   1) There is no grids on both north and south poles;
	// half meridinal grid intervals:
	if (type == Full || type == LonHalf) {
		this->lat.resize(numLat);
		cosLat.resize(numLat);
	    for (int j = 0; j < this->lat.size(); ++j) {
	        this->lat(j) = lat[j];
	        cosLat(j) = cos(lat[j]);
		}
        dlat.resize(numLat+1);
        dlat(0) = PI05-lat[0];
        for (int j = 1; j < dlat.size()-1; ++j) {
            dlat(j) = lat[j-1]-lat[j];
        }
        dlat(dlat.size()-1) = lat[numLat-1]+PI05;
    } else if (type == LatHalf || type == BothHalf) {
		this->lat.resize(numLat+1);
		cosLat.resize(numLat+1);
		dlat.resize(numLat);
        this->lat(0) = (PI05+lat[0])*0.5;
        cosLat(0) = cos(this->lat(0));
        for (int j = 1; j < this->lat.size()-1; ++j) {
            this->lat(j) = (lat[j]+lat[j-1])*0.5;
            cosLat(j) = cos(this->lat(j));
            dlat(j-1) = this->lat(j-1)-this->lat(j);
        }
        this->lat(this->lat.size()-1) = (-PI05+lat[numLat-1])*0.5;
        cosLat(cosLat.size()-1) = cos(this->lat(this->lat.size()-1));
        dlat(dlat.size()-1) = this->lat(this->lat.size()-2)-
                              this->lat(this->lat.size()-1);
	}
	// -------------------------------------------------------------------------
    // area of full mesh cells:
	if (type == Full) {
        area.resize(numLon,numLat+2);
		for (int i = 0; i < area.extent(0); ++i) {
			// normal regions:
			for (int j = 1; j < area.extent(1)-1; ++j) {
				area(i, j) = Sphere::radius2*dlon*(
					sin(lat[j-1]+dlat(j-1)*0.5)-
					sin(lat[j-1]-dlat(j)*0.5));
			}
			// pole caps:
			// Note: sum(area[:][0]) is the area of the north polar cap.
			area(i, 0) = Sphere::radius2*dlon*
				(1.0-sin(PI*0.5-*(dlat.begin())*0.5));
			area(i, area.extent(1)-1) = area(i, 0);
		}
	}
    isConstructed = true;
}

void RLLMesh::dump() const
{
    cout << "Longitude grids:" << endl;
    for (int i = 0; i < lon.size(); ++i) {
        cout << setw(10) << setprecision(4) << lon(i)*Rad2Deg;
        if (fmod(i, 8) == 7) {
            cout << endl;
        }
    }
    cout << endl;
    cout << "Latitude grids:" << endl;
    for (int j = 0; j < lat.size(); ++j) {
        cout << setw(10) << setprecision(4) << lat(j)*Rad2Deg;
        if (fmod(j, 8) == 7) {
            cout << endl;
        }
    }
    cout << endl;
}
