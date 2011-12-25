#include "RLLMesh.h"
#include "Constants.h"
#include "Sphere.h"
#include "ReportMacros.h"
#include <iostream>
#include <iomanip>
#include <netcdfcpp.h>

using std::cout;
using std::endl;
using std::setw;
using std::setprecision;

RLLMesh::RLLMesh()
{
    isConstructed = false;
}

RLLMesh::RLLMesh(MeshSpec spec, int numLon, int numLat,
                 double *lon, double *lat)
{
    isConstructed = false;
    init(spec, numLon, numLat, lon, lat);
}

RLLMesh::~RLLMesh()
{
}

void RLLMesh::init(MeshSpec spec, int numLon, int numLat,
                   double *lon, double *lat)
{
    if (isConstructed)
        REPORT_ERROR("RLL mesh has already been created.");
	// Note: Input "lon" and "lat" are of FULL mesh
    this->spec = spec;
	// -------------------------------------------------------------------------
	// longitude grids:
	// There are several assumptions as following:
	//   1) The longitude grids are equidistance;
	//   2) Input "lon" is not periodic;
	dlon = lon[1]-lon[0];
    if (spec.type == Full || spec.type == LatHalf) {
		this->lon.resize(numLon+1);
		for (int i = 0; i < this->lon.size()-1; ++i)
			this->lon(i) = lon[i];
        this->lon(this->lon.size()-1) = lon[numLon-1]+dlon;
    } else if (spec.type == LonHalf || spec.type == BothHalf) {
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
	if (spec.type == Full || spec.type == LonHalf) {
		this->lat.resize(numLat);
		cosLat.resize(numLat);
	    for (int j = 0; j < this->lat.size(); ++j) {
	        this->lat(j) = lat[j];
	        cosLat(j) = cos(lat[j]);
		}
        if (spec.isWithPoles) {
            dlat.resize(numLat-1);
            for (int j = 0; j < dlat.size(); ++j)
                dlat(j) = lat[j]-lat[j+1];
        } else {
            dlat.resize(numLat+1);
            dlat(0) = PI05-lat[0];
            for (int j = 1; j < dlat.size()-1; ++j)
                dlat(j) = lat[j-1]-lat[j];
            dlat(dlat.size()-1) = lat[numLat-1]+PI05;
        }
    } else if (spec.type == LatHalf || spec.type == BothHalf) {
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
	if (spec.type == Full) {
        if (spec.isWithPoles) {
            area.resize(numLon, numLat-1);
            for (int i = 0; i < area.extent(0); ++i)
                for (int j = 0; j < area.extent(1); ++j)
                    area(i, j) = Sphere::radius2*dlon*(sin(lat[j])-sin(lat[j+1]));
        } else {
            if (spec.isAreaFit) {
                area.resize(numLon, numLat);
                for (int i = 0; i < area.extent(0); ++i) {
                    for (int j = 1; j < area.extent(1)-1; ++j)
                        area(i, j) = Sphere::radius2*dlon*
                        (sin(lat[j]+dlat(j)*0.5)-sin(lat[j]-dlat(j+1)*0.5));
                    area(i, 0) = Sphere::radius2*dlon*
                    (1.0-sin(lat[0]-dlat(1)*0.5));
                    area(i, area.extent(1)-1) = Sphere::radius2*dlon*
                    (sin(lat[numLat-1]+dlat(numLat-1)*0.5)+1.0);
                }
            } else {
                area.resize(numLon, numLat+2);
                for (int i = 0; i < area.extent(0); ++i) {
                    // normal regions:
                    for (int j = 1; j < area.extent(1)-1; ++j)
                        area(i, j) = Sphere::radius2*dlon*
                        (sin(lat[j-1]+dlat(j-1)*0.5)-sin(lat[j-1]-dlat(j)*0.5));
                    // pole caps:
                    // Note: sum(area[:][0]) is the area of the north polar cap.
                    area(i, 0) = Sphere::radius2*dlon*
                    (1.0-sin(PI05-dlat(0)*0.5));
                    area(i, area.extent(1)-1) = Sphere::radius2*dlon*
                    (sin(-PI05+dlat(numLat)*0.5)+1.0);
                }
            }
        }
#ifdef DEBUG
        double totalArea = sum(area);
        assert(fabs(totalArea/Sphere::radius2/4.0-PI) < 1.0e-10);
#endif
    }
    isConstructed = true;
}

void RLLMesh::getBoundBox(int i1, int i2, int j1, int j2, int ii, int jj,
                          int &I1, int &I2, int &J1, int &J2) const
{
    // TODO: We will encounter problem in near-pole area.
    // Note: The mesh should be the center mesh!
    int di = fabs(i1-i2);
    if (i1 <= i2) {
        if (di < lon.size()-di) {
            I1 = i1-ii;
            I2 = i2+ii;
        } else {
            I1 = i2-ii;
            I2 = i1+ii;
        }
    } else {
        if (di < lon.size()-di) {
            I1 = i2-ii;
            I2 = i1+ii;
        } else {
            I1 = i1-ii;
            I2 = i2+ii;
        }
    }
    if (I1 < 0)
        I1 += lon.size()-1;
    if (I2 >= lon.size()-1)
        I2 -= lon.size()-1;
    if (j1 <= j2) {
        J1 = j1-jj;
        J2 = j2+jj;
    } else {
        J1 = j2-jj;
        J2 = j1+jj;
    }
}

bool RLLMesh::isInBoundBox(int I1, int I2, int J1, int J2, int i, int j) const
{
    if (j < J1 || j > J2)
        return false;
    int dI = fabs(I1-I2);
    if (I1 < I2) {
        if (dI < lon.size()-dI) {
            if (i < I1 || i > I2)
                return false;
        } else {
            if (i > I1 && i < I2)
                return false;
        } 
    } else {
        if (dI < lon.size()-dI) {
            if (i < I2 || i > I1)
                return false;
        } else {
            if (i > I2 && i < I1)
                return false;
        }
    }
    return true;
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

void RLLMesh::output(const string &fileName) const
{
    NcFile file(fileName.c_str(), NcFile::Replace);
    if (!file.is_valid()) {
        Message message;
        message << "Failed to create point counter mesh file \"";
        message << fileName << "\"!";
        REPORT_ERROR(message.str());
    }
    NcDim *lonDim = file.add_dim("lon", lon.size());
    NcDim *latDim = file.add_dim("lat", lat.size());
    NcVar *lonVar = file.add_var("lon", ncDouble, lonDim);
    NcVar *latVar = file.add_var("lat", ncDouble, latDim);
    double lonTmp[lon.size()], latTmp[lat.size()];
    for (int i = 0; i < lon.size(); ++i)
        lonTmp[i] = lon(i)*Rad2Deg;
    for (int j = 0; j < lat.size(); ++j)
        latTmp[j] = lat(j)*Rad2Deg;
    lonVar->put(lonTmp, lon.size());
    latVar->put(latTmp, lat.size());
    file.close();
}
