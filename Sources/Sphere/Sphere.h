#ifndef _Sphere_h_
#define _Sphere_h_

#include "Point.h"

enum OrientStatus {
    OrientLeft = 1, OrientRight = 2, OrientOn = 0
};

enum InCircleStatus {
    InsideCircle = 1, OutsideCircle = 2, OnCircle = 0
};

class Sphere
{
public:
	Sphere();
	virtual ~Sphere();

    static double calcDistance(const Coordinate &x1, const Coordinate &x2);

    static void rotate(const Coordinate &xp, const Coordinate &xo,
                       Coordinate &xr);
    static void inverseRotate(const Coordinate &xp, Coordinate &xo,
                              const Coordinate &xr);

    static OrientStatus orient(const Coordinate &, const Coordinate &,
                               const Coordinate &);
    static OrientStatus orient(Point *, Point *, Point *);

    static bool overlapTest(Point *, Point *);

    static InCircleStatus inCircle(Point *, Point *, Point *, Point *);

    static int inTriangle(Point *, Point *, Point *, Point *);

	static const double radius;
	static const double radius2;
};

#endif
