#include "Velocity.h"
#include <cmath>
#include <iomanip>

using std::setw;
using std::setprecision;

Velocity::Velocity()
{
}

Velocity::~Velocity()
{
}

const Velocity Velocity::operator+(const Velocity &that) const
{
    Velocity res;
    res.u = this->u+that.u;
    res.v = this->v+that.v;
    res.ut = this->ut+that.ut;
    res.vt = this->vt+that.vt;
    return res;
}

const Velocity Velocity::operator*(double scale) const
{
    Velocity res;
    res.u = this->u*scale;
    res.v = this->v*scale;
    res.ut = this->ut*scale;
    res.vt = this->vt*scale;
    return res;
}

const Velocity Velocity::operator/(double scale) const
{
    Velocity res;
    res.u = this->u/scale;
    res.v = this->v/scale;
    res.ut = this->ut/scale;
    res.vt = this->vt/scale;
    return res;
}

Velocity &Velocity::operator-=(const Velocity &that)
{
    this->u -= that.u;
    this->v -= that.v;
    this->ut -= that.ut;
    this->vt -= that.vt;
    return *this;
}

void Velocity::transform(const Coordinate &x, Location::Pole pole, Velocity &velocity)
{
    double sign;
    if (pole == Location::NorthPole) {
        sign = 1.0;
    } else if (pole == Location::SouthPole) {
        sign = -1.0;
    }
    double sinLon = sin(x.getLon());
    double cosLon = cos(x.getLon());
    double sinLat = sin(x.getLat());
    double sinLat2 = sinLat*sinLat;
    velocity.ut = sign*(-sinLon/sinLat*velocity.u
                        -cosLon/sinLat2*velocity.v);
    velocity.vt = sign*( cosLon/sinLat*velocity.u
                        -sinLon/sinLat2*velocity.v);
}

void Velocity::dump(int indentLevel) const
{
    cout << setw(30) << setprecision(20) << u;
    cout << setw(30) << setprecision(20) << v << endl;
    cout << setw(30) << setprecision(20) << ut;
    cout << setw(30) << setprecision(20) << vt << endl;
}
