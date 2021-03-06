#include "Field.hpp"
#include "ReportMacros.hpp"
#include "Constants.hpp"

Field::Field()
{
    mesh[Center] = NULL;
    mesh[Bound] = NULL;
    layers = NULL;
}

Field::~Field()
{
}

void Field::init(const RLLMesh &meshCnt)
{
    mesh[Center] = &meshCnt;
    values.resize(meshCnt.getNumLon(), meshCnt.getNumLat(), 1);
    for (int i = 1; i < meshCnt.getNumLon()-1; ++i)
        for (int j = 0; j < meshCnt.getNumLat(); ++j)
            values(i, j, 0).init();
    // zonal periodic boundary conditions
    for (int j = 0; j < meshCnt.getNumLat(); ++j) {
        values(0, j, 0).mirror(values(meshCnt.getNumLon()-2, j, 0));
        values(meshCnt.getNumLon()-1, j, 0).mirror(values(1, j, 0));
    }
}

void Field::init(const RLLMesh &meshCnt, const Layers &layers)
{
    mesh[Center] = &meshCnt;
    this->layers = &layers;
    values.resize(meshCnt.getNumLon(), meshCnt.getNumLat(), layers.getNumLev());
    for (int k = 0; k < layers.getNumLev(); ++k) {
        for (int i = 1; i < meshCnt.getNumLon()-1; ++i)
            for (int j = 0; j < meshCnt.getNumLat(); ++j)
                values(i, j, k).init();
        // zonal periodic boundary conditions
        for (int j = 0; j < meshCnt.getNumLat(); ++j) {
            values(0, j, k).mirror(values(meshCnt.getNumLon()-2, j, k));
            values(meshCnt.getNumLon()-1, j, k).mirror(values(1, j, k));
        }
    }
}

void Field::init(const RLLMesh &meshCnt, const RLLMesh &meshBnd)
{
    mesh[Bound] = &meshBnd;
    init(meshCnt);
}

void Field::init(const RLLMesh &meshCnt, const RLLMesh &meshBnd,
                 const Layers &layers)
{
    mesh[Bound] = &meshBnd;
    init(meshCnt, layers);
}

double Field::interp(const Coordinate &x, const Location &loc,
                     TimeLevel timeLevel) const
{
#ifdef DEBUG
    assert(loc.i[mesh[Center]->spec.type] != LOCATION_UNSET_INDEX);
#endif
    // bilinear interpolation
    int i1, i2, i3, i4, j1, j2, j3, j4;
    i1 = loc.i[mesh[Center]->spec.type];
    i2 = i1+1; i3 = i1; i4 = i2;
    j1 = loc.j[mesh[Center]->spec.type];
    j2 = j1; j3 = j1+1; j4 = j3;
    double lon, lat;
    double f1, f2, f3, f4;
    double a, b, c, d;
    lon = (x.getLon()-mesh[Center]->lon(i1))/
          (mesh[Center]->lon(i2)-mesh[Center]->lon(i1));
    lat = (x.getLat()-mesh[Center]->lat(j1))/
          (mesh[Center]->lat(j3)-mesh[Center]->lat(j1));
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

MultiTimeLevel<double, 2> &Field::operator()(int i, int j, int k)
{
#ifdef DEBUG
    assert(i >= 0 && i < mesh[Center]->getNumLon()-2);
#endif
    return values(i+1, j, k);
}

const MultiTimeLevel<double, 2> &Field::operator()(int i, int j, int k) const
{
#ifdef DEBUG
    assert(i >= 0 && i < mesh[Center]->getNumLon()-2);
#endif
    return values(i+1, j, k);
}
