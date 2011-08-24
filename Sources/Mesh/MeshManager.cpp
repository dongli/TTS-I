#include "MeshManager.h"
#include "ReportMacros.h"
#include "Constants.h"
#include "Sphere.h"
#include <iostream>

using std::cout;
using std::endl;
using std::min;

MeshManager::MeshManager()
{
#ifndef UNIT_TEST
    REPORT_ONLINE("MeshManager")
#endif
    this->PoleR = 5.0/Rad2Deg;
}

MeshManager::~MeshManager()
{
#ifndef UNIT_TEST
    REPORT_OFFLINE("MeshManager")
#endif
}

void MeshManager::setPoleR(double PoleR)
{
    this->PoleR = PoleR;
}

void MeshManager::construct(int numLon, int numLat, 
                            double *lon, double *lat)
{
    // Note: Assume the input "lon", "lat" and "lev" are the axis
    //       coordinate of full mesh and full layers.
    // -------------------------------------------------------------------------
    mesh[RLLMesh::Full].construct(RLLMesh::Full, numLon, numLat, lon, lat);
    mesh[RLLMesh::LonHalf].construct(RLLMesh::LonHalf, numLon, numLat, lon, lat);
    mesh[RLLMesh::LatHalf].construct(RLLMesh::LatHalf, numLon, numLat, lon, lat);
    mesh[RLLMesh::BothHalf].construct(RLLMesh::BothHalf, numLon, numLat, lon, lat);

    // -------------------------------------------------------------------------
    pointCounter.construct(mesh[RLLMesh::BothHalf].lon, mesh[RLLMesh::BothHalf].lat, 2, 2);
}

void MeshManager::construct(int numLon, int numLat, int numLev,
                            double *lon, double *lat, double *lev)
{
    // Note: Assume the input "lon", "lat" and "lev" are the axis
    //       coordinate of full mesh and full layers.
    // -------------------------------------------------------------------------
    mesh[RLLMesh::Full].construct(RLLMesh::Full, numLon, numLat, lon, lat);
    mesh[RLLMesh::LonHalf].construct(RLLMesh::LonHalf, numLon, numLat, lon, lat);
    mesh[RLLMesh::LatHalf].construct(RLLMesh::LatHalf, numLon, numLat, lon, lat);
    mesh[RLLMesh::BothHalf].construct(RLLMesh::BothHalf, numLon, numLat, lon, lat);

    // -------------------------------------------------------------------------
    layers[Layers::Full].construct(Layers::Full, numLev, lev);
    layers[Layers::Half].construct(Layers::Half, numLev, lev);

    // -------------------------------------------------------------------------
}

bool MeshManager::hasLayers() const
{
    return layers[0].isConstructed;
}

void MeshManager::checkLocation(const Coordinate &x, Location &loc,
                                Point *point)
{
    double rlat = PI05-x.getLat();
    if (fabs(rlat) < EPS) rlat = 0.0;

    double ratio;
    int j;

    // -------------------------------------------------------------------------
    // full mesh location index
    // Note: The zonal grids are equidistant.
    ratio = x.getLon()/mesh[RLLMesh::Full].dlon;
    // Note: This is really buggy! Add the up limit on the zonal index of full
    //       Mesh to handle the occasion that the longitude is 360 degree.
    loc.i[RLLMesh::Full] = min(int(floor(ratio)),
                               mesh[RLLMesh::Full].getNumLon()-2);
#ifdef DEBUG
    if (loc.i[RLLMesh::Full] == -1) {
        REPORT_ERROR("Location longitude index is -1!")
    }
#endif
    // Note: The meridinal grids may not be equidistant.
    // The range of j is (-1,n-1)
    if (loc.j[RLLMesh::Full] == LOCATION_UNSET_INDEX) {
        // first check
        if (x.getLat() >= *(mesh[RLLMesh::Full].lat.begin()) &&
            x.getLat() <= PI05) {
            // north polar cap
            loc.j[RLLMesh::Full] = -1;
        } else if (x.getLat() >= -PI05 &&
                   x.getLat() < mesh[RLLMesh::Full].lat(mesh[RLLMesh::Full].lat.size()-1)) {
            // south polar cap
            loc.j[RLLMesh::Full] = mesh[RLLMesh::Full].getNumLat()-1;
        } else {
            // normal region
            for (j = 0; j < mesh[RLLMesh::Full].getNumLat()-1; ++j) {
                if (x.getLat() >= mesh[RLLMesh::Full].lat(j+1) &&
                    x.getLat() < mesh[RLLMesh::Full].lat(j)) {
                    loc.j[RLLMesh::Full] = j;
                    break;
                }
            }
#ifdef DEBUG
            if (j == mesh[RLLMesh::Full].getNumLat()-1) {
                REPORT_ERROR("Latitude is out of range [-90,90].")
            }
#endif
        }
    } else {
        // take the hint
        double lat1, lat2;
        if (loc.j[RLLMesh::Full] == -1) {
            lat1 = PI05;
        } else {
            lat1 = mesh[RLLMesh::Full].lat(loc.j[RLLMesh::Full]);
        }
        if (loc.j[RLLMesh::Full] == mesh[RLLMesh::Full].getNumLat()-1) {
            lat2 = -PI05;
        } else {
            lat2 = mesh[RLLMesh::Full].lat(loc.j[RLLMesh::Full]+1);
        }
        if (x.getLat() > lat1) {
            // move northward
            if (loc.j[RLLMesh::Full] <= 0) {
                if (x.getLat() <= PI05) {
                    loc.j[RLLMesh::Full] = -1;
                } else {
                    cout << "[Error]: MeshManager::checkLocation: " <<
                    "Latitude > 90 degree!" << endl;
                }
            } else {
                for (j = loc.j[RLLMesh::Full]-1; j >= 0; j--) {
                    if (x.getLat() >= mesh[RLLMesh::Full].lat(j+1) &&
                        x.getLat() < mesh[RLLMesh::Full].lat(j)) {
                        loc.j[RLLMesh::Full] = j;
                        break;
                    }
                }
            }
        } else if (x.getLat() <= lat2) {
            // move southward
            if (loc.j[RLLMesh::Full] >= int(mesh[RLLMesh::Full].lat.size()-2)) {
                if (x.getLat() >= -PI05) {
                    loc.j[RLLMesh::Full] = mesh[RLLMesh::Full].getNumLat()-1;
                } else {
                    cout << "[Error]: MeshManager::checkLocation: " <<
                    "Latitude < -90 degree!" << endl;
                }
            } else {
                for (j = loc.j[RLLMesh::Full]+1; 
                    j < mesh[RLLMesh::Full].getNumLat()-1; ++j) {
                    if (x.getLat() >= mesh[RLLMesh::Full].lat(j+1) &&
                        x.getLat() < mesh[RLLMesh::Full].lat(j)) {
                        loc.j[RLLMesh::Full] = j;
                        break;
                    }
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // longitude half mesh location index
    ratio = x.getLon()/mesh[RLLMesh::LonHalf].dlon+0.5;
    loc.i[RLLMesh::LonHalf] = min(int(floor(ratio)),
                                  mesh[RLLMesh::LonHalf].getNumLon()-2);
    loc.j[RLLMesh::LonHalf] = loc.j[RLLMesh::Full];

    // -------------------------------------------------------------------------
    // latitude half mesh location index
    loc.i[RLLMesh::LatHalf] = loc.i[RLLMesh::Full];
    if (loc.j[RLLMesh::LatHalf] == LOCATION_UNSET_INDEX) {
        // first check
        if (x.getLat() >= *(mesh[RLLMesh::LatHalf].lat.begin()) &&
            x.getLat() <= PI05) {
            loc.j[RLLMesh::LatHalf] = -1;
        } else if (x.getLat() >= -PI05 &&
                   x.getLat() < mesh[RLLMesh::LatHalf].lat(mesh[RLLMesh::LatHalf].lat.size()-1)) {
            loc.j[RLLMesh::LatHalf] = mesh[RLLMesh::LatHalf].getNumLat()-1;
        } else {
            for (j = 0; j < mesh[RLLMesh::LatHalf].getNumLat()-1; ++j) {
                if (x.getLat() >= mesh[RLLMesh::LatHalf].lat(j+1) &&
                    x.getLat() < mesh[RLLMesh::LatHalf].lat(j)) {
                    loc.j[RLLMesh::LatHalf] = j;
                    break;
                }
            }
        }
    } else {
        // take the hint
        double lat1, lat2;
        if (loc.j[RLLMesh::LatHalf] == -1) {
            lat1 = PI05;
        } else {
            lat1 = mesh[RLLMesh::LatHalf].lat(loc.j[RLLMesh::LatHalf]);
        }
        if (loc.j[RLLMesh::LatHalf] == mesh[RLLMesh::LatHalf].getNumLat()-1) {
            lat2 = -PI05;
        } else {
            lat2 = mesh[RLLMesh::LatHalf].lat(loc.j[RLLMesh::LatHalf]+1);
        }
        if (x.getLat() > lat1) {
            // move northward
            if (loc.j[RLLMesh::LatHalf] <= 0) {
                if (x.getLat() <= PI05) {
                    loc.j[RLLMesh::LatHalf] = -1;
                } else {
                    cout << "[Error]: MeshManager::checkLocation: " <<
                    "Latitude > 90 degree!" << endl;
                }
            } else {
                for (j = loc.j[RLLMesh::LatHalf]-1; j >= 0; j--) {
                    if (x.getLat() >= mesh[RLLMesh::LatHalf].lat(j+1) &&
                        x.getLat() < mesh[RLLMesh::LatHalf].lat(j)) {
                        loc.j[RLLMesh::LatHalf] = j;
                        break;
                    }
                }
            }
        } else if (x.getLat() <= lat2) {
            // move southward
            if (loc.j[RLLMesh::LatHalf] >= mesh[RLLMesh::LatHalf].getNumLat()-2) {
                if (x.getLat() >= -PI05) {
                    loc.j[RLLMesh::LatHalf] = mesh[RLLMesh::LatHalf].getNumLat()-1;
                } else {
                    cout << "[Error]: MeshManager::checkLocation: "
                    "Latitude < -90 degree!" << endl;
                }
            } else {
                for (j = loc.j[RLLMesh::LatHalf]+1; 
                    j < mesh[RLLMesh::LatHalf].getNumLat()-1; ++j) {
                    if (x.getLat() >= mesh[RLLMesh::LatHalf].lat(j+1) &&
                        x.getLat() < mesh[RLLMesh::LatHalf].lat(j)) {
                        loc.j[RLLMesh::LatHalf] = j;
                        break;
                    }
                }
            }
        }
    }
    
    // -------------------------------------------------------------------------
    // both half mesh location index
    loc.i[RLLMesh::BothHalf] = loc.i[RLLMesh::LonHalf];
    loc.j[RLLMesh::BothHalf] = loc.j[RLLMesh::LatHalf];

    // -------------------------------------------------------------------------
    // check whether in the polar cap
    if (loc.j[RLLMesh::Full] == -1) {
        loc.inPolarCap = true;
        loc.pole = Location::NorthPole;
    } else if (loc.j[RLLMesh::Full] == mesh[RLLMesh::Full].getNumLat()-1) {
        loc.inPolarCap = true;
        loc.pole = Location::SouthPole;
    } else {
        loc.inPolarCap = false;
        loc.pole = Location::Null;
    }
    
    // -------------------------------------------------------------------------
    // check whether on the pole
    double R = PI05-fabs(x.getLat());
    if (R < PoleR) {
        loc.onPole = true;
        loc.pole = x.getLat() > 0.0 ? Location::NorthPole : Location::SouthPole;
    } else {
        loc.onPole = false;
    }

    // -------------------------------------------------------------------------
    // vertical layer index
    if (!hasLayers()) {
        loc.k = 0;
    }
    
    // -------------------------------------------------------------------------
    // count
    if (point != NULL) {
        static double dlon = mesh[RLLMesh::BothHalf].dlon/pointCounter.numSubLon;
        ratio = (x.getLon()-mesh[RLLMesh::BothHalf].lon(loc.i[RLLMesh::BothHalf]))/dlon;
        loc.i.back() = int(floor(ratio))+loc.i[RLLMesh::BothHalf]*pointCounter.numSubLon;
        if (loc.j[RLLMesh::BothHalf] == -1) {
            loc.j.back() = 0;
        } else if (loc.j[RLLMesh::BothHalf] == mesh[RLLMesh::BothHalf].getNumLat()-1) {
            loc.j.back() = pointCounter.counters.extent(1)-1;
        } else {
            double dlat = mesh[RLLMesh::BothHalf].dlat(loc.j[RLLMesh::BothHalf])/pointCounter.numSubLat;
            ratio = (mesh[RLLMesh::BothHalf].lat(loc.j[RLLMesh::BothHalf])-x.getLat())/dlat;
            loc.j.back() = int(floor(ratio))+loc.j[RLLMesh::BothHalf]*pointCounter.numSubLat+1;
        }
        pointCounter.count(loc, point);
    }
}

void MeshManager::move(const Coordinate &x0, Coordinate &x1, const Velocity &v,
                       Second dt, const Location &loc) const
{
    double lon, lat;
    if (!loc.onPole) {
        double dlon, dlat;
        dlon = v.u*dt/Sphere::radius/cos(x0.getLat());
        dlat = v.v*dt/Sphere::radius;
        lon = x0.getLon()+dlon;
        lat = x0.getLat()+dlat;
        // Polar boundary check
        if (lat > PI05) {
            lon = PI+x0.getLon()-dlon;
            lat = PI-x0.getLat()-dlat;
        }
        if (lat < -PI05) {
            lon = PI+x0.getLon()-dlon;
            lat = -PI-x0.getLon()-dlat;
        }
        // Zonal boundary check
        if (lon < 0.0) {
            lon = PI2+fmod(lon, PI2);
        } else if (lon > PI2) {
            lon = fmod(lon, PI2);
        }
#ifdef DEBUG
        if (lon != lon) {
            REPORT_ERROR("Longtiude is NaN!")
        }
        if (lat != lat) {
            REPORT_ERROR("Latitude is NaN!")
        }
#endif
    } else {
        double xt[2];
        double tanLat = tan(x0.getLat());
        double sign;
        if (loc.pole == Location::NorthPole) {
            sign = 1.0;
        } else {
            sign = -1.0;
        }
        xt[0] = sign*Sphere::radius*cos(x0.getLon())/tanLat;
        xt[1] = sign*Sphere::radius*sin(x0.getLon())/tanLat;
        // Move in local coordinate system
        xt[0] += v.ut*dt;
        xt[1] += v.vt*dt;
        // Transform back into spherical coordinate system
        lon = atan2(xt[1], xt[0]);
        if (lon < 0.0) lon += PI2;
        lat = sign*atan(Sphere::radius/sqrt(xt[0]*xt[0]+xt[1]*xt[1]));
#ifdef DEBUG
        if (lon != lon) {
            REPORT_ERROR("Longtiude is NaN!")
        }
        if (lat != lat) {
            REPORT_ERROR("Latitude is NaN!")
        }
#endif
    }
    x1.set(lon, lat);
}
