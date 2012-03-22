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
    REPORT_ONLINE("MeshManager")
    // TODO: Determine the value of PoleR.
    this->PoleR = 18.0/Rad2Deg;
}

MeshManager::~MeshManager()
{
    REPORT_OFFLINE("MeshManager")
}

void MeshManager::setPoleR(double PoleR)
{
    this->PoleR = PoleR;
}

void MeshManager::init(int numLon, int numLat, double *lon, double *lat)
{
    // Note: Assume the input "lon", "lat" and "lev" are the axis
    //       coordinate of full mesh and full layers.
    // -------------------------------------------------------------------------
    MeshSpec meshSpec;
    meshSpec.isWithPoles = false;
    meshSpec.isAreaFit = false;
    meshSpec.type = Full;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = LonHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = LatHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = BothHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    // -------------------------------------------------------------------------
    pointCounter.init(mesh[BothHalf].lon, mesh[BothHalf].lat, 1, 1);
}

void MeshManager::init(int numLon, int numLat, int numLev,
                       double *lon, double *lat, double *lev)
{
    // Note: Assume the input "lon", "lat" and "lev" are the axis
    //       coordinate of full mesh and full layers.
    // -------------------------------------------------------------------------
    MeshSpec meshSpec;
    meshSpec.isWithPoles = false;
    meshSpec.isAreaFit = false;
    meshSpec.type = Full;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = LonHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = LatHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    meshSpec.type = BothHalf;
    mesh[meshSpec.type].init(meshSpec, numLon, numLat, lon, lat);
    // -------------------------------------------------------------------------
    layers[Layers::Full].init(Layers::Full, numLev, lev);
    layers[Layers::Half].init(Layers::Half, numLev, lev);
    // -------------------------------------------------------------------------
}

bool MeshManager::hasLayers() const
{
    return layers[0].isConstructed;
}

void MeshManager::checkLocation(const Coordinate &x, Location &loc,
                                Point *point)
{
#ifdef DEBUG
    if (x.getLat() > PI05 || x.getLat() < -PI05) {
        Message message;
        message << "Point latitude (" << x.getLat()*Rad2Deg;
        message << ") is greater than 90 degree!";
        REPORT_ERROR(message.str());
    }
#endif
    double ratio;
    int j;
    // -------------------------------------------------------------------------
    // full mesh location index
    // Note: The zonal grids are equidistant.
    ratio = x.getLon()/mesh[Full].dlon;
    // Note: This is really buggy! Add the up limit on the zonal index of full
    //       Mesh to handle the occasion that the longitude is 360 degree.
    loc.i[Full] = min(int(floor(ratio)), mesh[Full].getNumLon()-2);
#ifdef DEBUG
    if (loc.i[Full] == -1) {
        REPORT_ERROR("Location longitude index is -1!")
    }
#endif
    // Note: The meridinal grids may not be equidistant.
    //       The range of j is (-1,mesh[Full].getNumLat()-1)
    if (loc.j[Full] == LOCATION_UNSET_INDEX) {
        // first check
        if (x.getLat() >= *(mesh[Full].lat.begin()) && x.getLat() <= PI05) {
            // north polar cap
            loc.j[Full] = -1;
        } else if (x.getLat() >= -PI05 &&
                   x.getLat() < mesh[Full].lat(mesh[Full].lat.size()-1)) {
            // south polar cap
            loc.j[Full] = mesh[Full].getNumLat()-1;
        } else {
            // normal region
            for (j = 0; j < mesh[Full].getNumLat()-1; ++j)
                if (x.getLat() >= mesh[Full].lat(j+1) &&
                    x.getLat() < mesh[Full].lat(j)) {
                    loc.j[Full] = j;
                    break;
                }
        }
    } else {
        // take the hint
        double lat1, lat2;
        if (loc.j[Full] == -1) {
            lat1 = PI05;
        } else {
            lat1 = mesh[Full].lat(loc.j[Full]);
        }
        if (loc.j[Full] == mesh[Full].getNumLat()-1) {
            lat2 = -PI05;
        } else {
            lat2 = mesh[Full].lat(loc.j[Full]+1);
        }
        if (x.getLat() > lat1) {
            // move northward
            for (j = loc.j[Full]-1; j >= 0; j--)
                if (x.getLat() >= mesh[Full].lat(j+1) &&
                    x.getLat() < mesh[Full].lat(j)) {
                    loc.j[Full] = j;
                    break;
                }
            if (j == -1)
                loc.j[Full] = -1;
        } else if (x.getLat() <= lat2) {
            // move southward
            for (j = loc.j[Full]+1; j < mesh[Full].getNumLat()-1; ++j)
                if (x.getLat() >= mesh[Full].lat(j+1) &&
                    x.getLat() < mesh[Full].lat(j)) {
                    loc.j[Full] = j;
                    break;
                }
            if (j == mesh[Full].getNumLat()-1)
                loc.j[Full] = mesh[Full].getNumLat()-1;
        }
    }
    // -------------------------------------------------------------------------
    // longitude half mesh location index
    ratio = x.getLon()/mesh[LonHalf].dlon+0.5;
    loc.i[LonHalf] = min(int(floor(ratio)), mesh[LonHalf].getNumLon()-2);
    loc.j[LonHalf] = loc.j[Full];
    // -------------------------------------------------------------------------
    // latitude half mesh location index
    loc.i[LatHalf] = loc.i[Full];
    if (loc.j[LatHalf] == LOCATION_UNSET_INDEX) {
        // first check
        if (x.getLat() >= *(mesh[LatHalf].lat.begin()) && x.getLat() <= PI05) {
            loc.j[LatHalf] = -1;
        } else if (x.getLat() >= -PI05 &&
                   x.getLat() < mesh[LatHalf].lat(mesh[LatHalf].lat.size()-1)) {
            loc.j[LatHalf] = mesh[LatHalf].getNumLat()-1;
        } else {
            for (j = 0; j < mesh[LatHalf].getNumLat()-1; ++j)
                if (x.getLat() >= mesh[LatHalf].lat(j+1) &&
                    x.getLat() < mesh[LatHalf].lat(j)) {
                    loc.j[LatHalf] = j;
                    break;
                }
        }
    } else {
        // take the hint
        double lat1, lat2;
        if (loc.j[LatHalf] == -1) {
            lat1 = PI05;
        } else {
            lat1 = mesh[LatHalf].lat(loc.j[LatHalf]);
        }
        if (loc.j[LatHalf] == mesh[LatHalf].getNumLat()-1) {
            lat2 = -PI05;
        } else {
            lat2 = mesh[LatHalf].lat(loc.j[LatHalf]+1);
        }
        if (x.getLat() > lat1) {
            // move northward
            for (j = loc.j[LatHalf]-1; j >= 0; j--)
                if (x.getLat() >= mesh[LatHalf].lat(j+1) &&
                    x.getLat() < mesh[LatHalf].lat(j)) {
                    loc.j[LatHalf] = j;
                    break;
                }
            if (j == -1)
                loc.j[LatHalf] = -1;
        } else if (x.getLat() <= lat2) {
            // move southward
            for (j = loc.j[LatHalf]+1; j < mesh[LatHalf].getNumLat()-1; ++j)
                if (x.getLat() >= mesh[LatHalf].lat(j+1) &&
                    x.getLat() < mesh[LatHalf].lat(j)) {
                    loc.j[LatHalf] = j;
                    break;
                }
            if (j == mesh[LatHalf].getNumLat()-1)
                loc.j[LatHalf] = mesh[LatHalf].getNumLat()-1;
        }
    }
    // -------------------------------------------------------------------------
    // both half mesh location index
    loc.i[BothHalf] = loc.i[LonHalf];
    loc.j[BothHalf] = loc.j[LatHalf];

#ifdef DEBUG
    for (int i = 0; i < 4; ++i) {
        assert(loc.i[i] > -1 && loc.i[i] < mesh[i].getNumLon()-1);
        assert(loc.j[i] > -2 && loc.j[i] < mesh[i].getNumLat());
    }
#endif
    // -------------------------------------------------------------------------
    // check whether in the polar cap
    if (loc.j[Full] == -1) {
        loc.inPolarCap = true;
        loc.pole = Location::NorthPole;
    } else if (loc.j[Full] == mesh[Full].getNumLat()-1) {
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
    // TODO: This part is duplicate with "countPoint" functionally, try to
    //       merge them.
    if (point != NULL) {
        double dlon = mesh[BothHalf].dlon/pointCounter.numSubLon;
        ratio = (x.getLon()-mesh[BothHalf].lon(loc.i[BothHalf]))/dlon;
        if (loc.i[3] == mesh[BothHalf].getNumLon()-2)
            loc.i.back() = int(floor(ratio));
        else
            loc.i.back() = int(floor(ratio))+loc.i[BothHalf]*pointCounter.numSubLon;
        if (loc.j[BothHalf] == -1) {
            loc.j.back() = 0;
        } else if (loc.j[BothHalf] == mesh[BothHalf].getNumLat()-1) {
            loc.j.back() = pointCounter.counters.extent(1)-1;
        } else {
            double dlat = mesh[BothHalf].dlat(loc.j[BothHalf])/pointCounter.numSubLat;
            ratio = (mesh[BothHalf].lat(loc.j[BothHalf])-x.getLat())/dlat;
            loc.j.back() = int(floor(ratio))+loc.j[BothHalf]*pointCounter.numSubLat+1;
        }
        pointCounter.count(loc, point);
    }
}

void MeshManager::countPoint(Point *point)
{
    double dlon = mesh[BothHalf].dlon/pointCounter.numSubLon;
    const Coordinate &x = point->getCoordinate();
    Location loc = point->getLocation();
    double ratio = (x.getLon()-mesh[BothHalf].lon(loc.i[BothHalf]))/dlon;
    if (loc.i[3] == mesh[BothHalf].getNumLon()-2)
        loc.i.back() = int(floor(ratio));
    else
        loc.i.back() = int(floor(ratio))+loc.i[BothHalf]*pointCounter.numSubLon;
    if (loc.j[BothHalf] == -1) {
        loc.j.back() = 0;
    } else if (loc.j[BothHalf] == mesh[BothHalf].getNumLat()-1) {
        loc.j.back() = pointCounter.counters.extent(1)-1;
    } else {
        double dlat = mesh[BothHalf].dlat(loc.j[BothHalf])/pointCounter.numSubLat;
        ratio = (mesh[BothHalf].lat(loc.j[BothHalf])-x.getLat())/dlat;
        loc.j.back() = int(floor(ratio))+loc.j[BothHalf]*pointCounter.numSubLat+1;
    }
    pointCounter.count(loc, point);
    point->setLocation(loc);
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
        // TODO: Check if the polar boundary will be reached or not.
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
    x1.setSPH(lon, lat);
}
