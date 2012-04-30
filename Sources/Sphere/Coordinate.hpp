#ifndef Coordinate_h
#define Coordinate_h

#include "ReportMacros.hpp"
#include "Vector.hpp"
#include "Constants.hpp"
#include <cmath>
#include <iomanip>

using std::setw;
using std::setprecision;

class Coordinate
{
public:
	Coordinate() { reinit(); }
    Coordinate(double lon, double lat, double lev = 0.0) {
        setSPH(lon, lat, lev);
    }
	virtual ~Coordinate() {}

    void reinit() { isSet_ = false; }

	void setSPH(double lon, double lat, double lev = 0.0) {
        sph(0) = lon;
        sph(1) = lat;
        sph(2) = lev;
        // convert spherical coordinate to Cartesian coordinate
        double cosLat = cos(lat);
        // TODO: Do we need to multiply sphere radius?
        car(0) = cosLat*cos(lon);
        car(1) = cosLat*sin(lon);
        car(2) = sin(lat);
        isSet_ = true;
    }
    void setCAR(double x, double y, double z) {
        sph(0) = x;
        sph(1) = y;
        sph(2) = z;
        // TODO: Consider the vertical coordinate!
        sph(0) = atan2(y, x);
        sph(1) = asin(z);
        if (sph(0) < 0.0) sph(0) += PI2;
        if (sph(0) > PI2) sph(0) -= PI2;
        isSet_ = true;
    }
    bool isSet() const { return isSet_; }

    const Vector &getSPH() const { return sph; }
	double getLon() const { return sph(0); }
	double getLat() const { return sph(1); }
	double getLev() const { return sph(2); }

    double getX() const { return car(0); }
    const Vector &getCAR() const { return car; }
    double getY() const { return car(1); }
    double getZ() const { return car(2); }

    friend bool operator==(const Coordinate &a, const Coordinate &b) {
        if (a.getLon() == b.getLon() &&
            a.getLat() == b.getLat() &&
            a.getLev() == b.getLev()) {
            return true;
        } else {
            return false;
        }
    }

    Coordinate &operator=(const Coordinate &a) {
        if (this != &a) {
            this->sph = a.sph;
            this->car = a.car;
        }
        return *this;
    }

    void dump(int indentLevel = 0) const {
        for (int i = 0; i < indentLevel; ++i)
            cout << "  ";
        cout << setw(30) << setprecision(20) << sph(0);
        cout << setw(30) << setprecision(20) << sph(1) << endl;
    }

private:
    Vector sph; // spherical coordinate
    Vector car; // Cartesian coordinate
    bool isSet_;
};

#endif
