#ifndef _Velocity_h_
#define _Velocity_h_

#include "Coordinate.h"
#include "Location.h"

class Velocity
{
public:
    enum Type {
        StereoPlane, LonLatSpace
    };

    Velocity();
    ~Velocity();

    const Velocity operator+(const Velocity &) const;

    const Velocity operator*(double) const;

    const Velocity operator/(double) const;

    Velocity &operator-=(const Velocity &);

    static void transform(const Coordinate &, Location::Pole, Velocity &);

    void dump(int indentLevel = 0) const;

    double u, v;
    double ut, vt;
};

#endif
