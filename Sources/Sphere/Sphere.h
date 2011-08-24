#ifndef _Sphere_h_
#define _Sphere_h_

#include "Point.h"

enum OrientStatus {
    OrientLeft = 0, OrientRight = 1, OrientOn = 2
};

enum InCircleStatus {
    InsideCircle = 1, OutsideCircle = 2, OnCircle = 0
};

class Sphere
{
public:
	Sphere() {};
	virtual ~Sphere() {};

    static void setRadius(double radius);

    static double calcDistance(const Coordinate &x1, const Coordinate &x2);

    static bool project(const Coordinate &x1, const Coordinate &x2,
                        const Coordinate &x3, Coordinate &x4, double &distance);

    static void rotate(const Coordinate &xp, const Coordinate &xo,
                       Coordinate &xr);
    static void inverseRotate(const Coordinate &xp, Coordinate &xo,
                              const Coordinate &xr);

    static void calcIntersect(const Coordinate &x1, const Coordinate &x2,
                              const Coordinate &x3, const Coordinate &x4,
                              Coordinate &x5, Coordinate &x6);
    static void calcIntersectLat(const Coordinate &x1, const Coordinate &x2,
                                 double lon, double &lat1, double &lat2);
    static void calcIntersectLon(const Coordinate &x1, const Coordinate &x2,
                                 double lat, double &lon1, double &lon2);

    static OrientStatus orient(const Coordinate &, const Coordinate &,
                               const Coordinate &);
    static OrientStatus orient(Point *, Point *, Point *);

    static bool overlapTest(Point *, Point *);

    static InCircleStatus inCircle(Point *, Point *, Point *, Point *);

    static int inTriangle(Point *, Point *, Point *, Point *);

	static double radius;
	static double radius2;
};

#endif
