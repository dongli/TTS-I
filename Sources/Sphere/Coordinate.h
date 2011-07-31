#ifndef _Coordinate_h_
#define _Coordinate_h_

#include "ReportMacros.h"
#include "Vector.h"
#include "Constants.h"
#include <cmath>
#include <iomanip>

using std::setw;
using std::setprecision;

class Coordinate
{
public:
	Coordinate() {}
	virtual ~Coordinate() {}

	void set(double lon, double lat, double lev = 0.0) {
        sph(0) = lon;
        sph(1) = lat;
        sph(2) = lev;
        // Convert spherical coordinate to Cartesian coordinate
        double cosLat = cos(lat);
        car(0) = cosLat*cos(lon);
        car(1) = cosLat*sin(lon);
        car(2) = sin(lat);
    }

    const TinyVector<double, 3> &getSPH() const { return sph; }
	double getLon() const { return sph(0); }
	double getLat() const { return sph(1); }
	double getLev() const { return sph(2); }

    double getX() const { return car(0); }
    const TinyVector<double, 3> &getCAR() const { return car; }
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
};

#endif
