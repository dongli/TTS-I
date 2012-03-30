#ifndef _Sphere_h_
#define _Sphere_h_

#include "Point.h"
class Polygon;

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

    static void convertCAR(double x, double y, double z,
                           double &lon, double &lat);
    static void convertSPH(double lon, double lat,
                           double &x, double &y, double &z);

    static double calcDistance(const Coordinate &x1, const Coordinate &x2);

    static double calcAngle(const Vector &vector1, const Vector &vector2);
    static double calcAngle(const Vector &vector1, const Vector &vector2,
                            const Coordinate &x);

    static double calcArea(const Coordinate &x1, const Coordinate &x2,
                           const Coordinate &x3);

    static bool project(const Coordinate &x1, const Coordinate &x2,
                        const Coordinate &x3, Coordinate &x4, double &distance);
    static bool isProject(const Coordinate &x1, const Coordinate &x2,
                          const Coordinate &x3);
    static bool isProject(Point *point1, Point *point2, Point *point3);

    static void rotate(const Coordinate &xp, const Coordinate &xo,
                       Coordinate &xr);
    static void inverseRotate(const Coordinate &xp, Coordinate &xo,
                              const Coordinate &xr);

    static bool isIntersect(const Coordinate &x1, const Coordinate &x2,
                            const Coordinate &x3, const Coordinate &x4);
    static bool isIntersect(Point *point1, Point *point2,
                            Point *point3, Point *point4);
    static bool calcIntersect(const Coordinate &x1, const Coordinate &x2,
                              const Coordinate &x3, const Coordinate &x4,
                              Coordinate &x5, Coordinate &x6);
    static bool calcIntersectLat(const Coordinate &x1, const Coordinate &x2,
                                 double lon, double lat1, double lat2,
                                 Coordinate &x);
    static bool calcIntersectLon(const Coordinate &x1, const Coordinate &x2,
                                 double lon1, double lon2, double lat,
                                 Coordinate &x, bool useMPFR = false);

    static void calcMiddlePoint(const Coordinate &x1, const Coordinate &x2,
                                Coordinate &x);
    static void calcCentroid(Polygon const *polygon, Coordinate &x);

    static bool is_lon_between(double lon1, double lon2, double lon);
    static bool is_lon_gt(double lon1, double lon2);
    static bool is_lon_lt(double lon1, double lon2);
    static bool is_lon_eq(double lon1, double lon2);
    static double diff_lon(double lon1, double lon2);

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
