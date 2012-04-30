#ifndef PolarRing_h
#define PolarRing_h

#include "Field.hpp"
#include "Velocity.hpp"
#include <vector>
#include <blitz/array.h>

using std::vector;
using blitz::Array;

class PolarRingVelocity
{
public:
    PolarRingVelocity();
    virtual ~PolarRingVelocity();

    void linkVelocityField(const Field &u, const Field &v);

    int getNumLon() const { return u[0].extent(0); }
    int getNumLev() const { return u[0].extent(1); }

    double getLon(int i) const { return lon[i]; }
    double getLat(int j) const { return lat[j]; }

    void update();

    Velocity interp(const Coordinate &, const Location &, TimeLevel) const;

    vector<double> lon;
    vector<double> sinLon, cosLon;
    double lat[2], sinLat[2], sinLat2[2];
    Array<MultiTimeLevel<double, 2>, 2> u[2], v[2];
    Array<MultiTimeLevel<double, 2>, 2> ut[2], vt[2]; // transformed velocity

private:
    bool isInitialized;
    const Field *uField, *vField;
};

#endif
