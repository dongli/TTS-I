#include "Field.h"
#include "ReportMacros.h"
#include "Constants.h"

Field::Field()
{
    mesh = NULL;
    layers = NULL;
}

Field::~Field()
{
}

void Field::init(const RLLMesh &mesh)
{
    this->mesh = &mesh;
    values.resize(mesh.getNumLon(), mesh.getNumLat(), 1);
    if (mesh.spec.type == Full || mesh.spec.type == LatHalf) {
        for (int i = 0; i < mesh.getNumLon()-1; ++i)
            for (int j = 0; j < mesh.getNumLat(); ++j)
                values(i, j, 0).init();
        // zonal periodic boundary condition
        for (int j = 0; j < mesh.getNumLat(); ++j)
            values(mesh.getNumLon()-1, j, 0).mirror(values(0, j, 0));
    } else if (mesh.spec.type == LonHalf) {
        for (int i = 1; i < mesh.getNumLon()-1; ++i)
            for (int j = 0; j < mesh.getNumLat(); ++j)
                values(i, j, 0).init();
        // zonal periodic boundary conditions
        for (int j = 0; j < mesh.getNumLat(); ++j) {
            values(0, j, 0).mirror(values(mesh.getNumLon()-2, j, 0));
            values(mesh.getNumLon()-1, j, 0).mirror(values(1, j, 0));
        }
    }
}

void Field::init(const RLLMesh &mesh, const Layers &layers)
{
    this->mesh = &mesh;
    this->layers = &layers;
    values.resize(mesh.getNumLon(), mesh.getNumLat(), layers.getNumLev());
    if (mesh.spec.type == Full || mesh.spec.type == LatHalf) {
        for (int k = 0; k < layers.getNumLev(); ++k) {
            for (int i = 0; i < mesh.getNumLon()-1; ++i)
                for (int j = 0; j < mesh.getNumLat(); ++j)
                    values(i, j, k).init();
            // zonal periodic boundary condition
            for (int j = 0; j < mesh.getNumLat(); ++j)
                values(mesh.getNumLon()-1, j, k).mirror(values(0, j, k));
        }
    } else if (mesh.spec.type == LonHalf) {
        for (int k = 0; k < layers.getNumLev(); ++k) {
            for (int i = 1; i < mesh.getNumLon()-1; ++i)
                for (int j = 0; j < mesh.getNumLat(); ++j)
                    values(i, j, k).init();
            // zonal periodic boundary conditions
            for (int j = 0; j < mesh.getNumLat(); ++j) {
                values(0, j, k).mirror(values(mesh.getNumLon()-2, j, k));
                values(mesh.getNumLon()-1, j, k).mirror(values(1, j, k));
            }
        }
    }
}

double Field::interp(const Coordinate &x, const Location &loc,
                     TimeLevel timeLevel) const
{
#ifdef DEBUG
    assert(loc.i[mesh->spec.type] != LOCATION_UNSET_INDEX);
#endif
    // bilinear interpolation
    int i1, i2, i3, i4, j1, j2, j3, j4;
    i1 = loc.i[mesh->spec.type];
    i2 = i1+1; i3 = i1; i4 = i2;
    j1 = loc.j[mesh->spec.type];
    j2 = j1; j3 = j1+1; j4 = j3;
    double lon, lat;
    double f1, f2, f3, f4;
    double a, b, c, d;
    lon = (x.getLon()-mesh->lon(i1))/(mesh->lon(i2)-mesh->lon(i1));
    lat = (x.getLat()-mesh->lat(j1))/(mesh->lat(j3)-mesh->lat(j1));
    f1 = values(i1, j1, loc.k).get(timeLevel);
    f2 = values(i2, j2, loc.k).get(timeLevel);
    f3 = values(i3, j3, loc.k).get(timeLevel);
    f4 = values(i4, j4, loc.k).get(timeLevel);
    a = f1;
    b = f2-f1;
    c = f3-f1;
    d = f1-f2-f3+f4;
    double res = a+b*lon+c*lat+d*lon*lat;
#ifdef DEBUG
    if (res != res) {
        REPORT_ERROR("Result is NaN!")
    }
#endif
    return res;
}
