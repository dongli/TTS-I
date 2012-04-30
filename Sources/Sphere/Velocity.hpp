#ifndef Velocity_h
#define Velocity_h

#include "Coordinate.hpp"
#include "Location.hpp"

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
