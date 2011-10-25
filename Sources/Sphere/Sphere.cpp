#include "Sphere.h"
#include "ReportMacros.h"
#include "Constants.h"
#include <cmath>

double Sphere::radius = 1.0;
double Sphere::radius2 = radius*radius;

void Sphere::setRadius(double r)
{
    radius = r;
    radius2 = r*r;
}

double Sphere::calcDistance(const Coordinate &x1, const Coordinate &x2)
{
    double dlon = x1.getLon()-x2.getLon();
    double tmp1 = sin(x1.getLat())*sin(x2.getLat());
    double tmp2 = cos(x1.getLat())*cos(x2.getLat())*cos(dlon);
    return radius*acos(tmp1+tmp2);
}

bool Sphere::project(const Coordinate &x1, const Coordinate &x2,
                     const Coordinate &x3, Coordinate &x4, double &distance)
{
    Coordinate x1r, x2r, x3r, x4r, xpr, xp;
    double lon;
    // find the rotating north pole that make arc x1->x2 equator
    rotate(x1, x2, x2r);
    xpr.set(x2r.getLon()-PI05, 0.0);
    inverseRotate(x1, xp, xpr);
    // rotate according to rotating north pole
    rotate(xp, x1, x1r);
    rotate(xp, x2, x2r);
    rotate(xp, x3, x3r);
    if (fabs(x3r.getLat()-PI05) < EPS) {
        REPORT_ERROR("Ambiguous point!")
    }
    if (x1r.getLon() < x2r.getLon()) {
        if (x3r.getLon() >= x2r.getLon() || x3r.getLon() <= x1r.getLon()) {
            distance = fabs(x3r.getLat())*Sphere::radius;
            x4r.set(x3r.getLon(), 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
        lon = x3r.getLon()-PI;
        if (lon < 0.0)
            lon += PI2;
        if (lon >= x2r.getLon() || lon <= x1r.getLon()) {
            distance = (PI-fabs(x3r.getLat()))*Sphere::radius;
            x4r.set(lon, 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
    } else {
        if (x3r.getLon() >= x2r.getLon() && x3r.getLon() <= x1r.getLon()) {
            distance = fabs(x3r.getLat())*Sphere::radius;
            x4r.set(x3r.getLon(), 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
        lon = x3r.getLon()-PI;
        if (lon < 0.0)
            lon += PI2;
        if (lon >= x2r.getLon() && lon <= x1r.getLon()) {
            distance = (PI-fabs(x3r.getLat()))*Sphere::radius;
            x4r.set(lon, 0.0);
            inverseRotate(xp, x4, x4r);
            return true;
        }
    }
    distance = UndefinedDistance;
    return false;
}

void Sphere::rotate(const Coordinate &xp, const Coordinate &xo, Coordinate &xr)
{
    double dlon = xo.getLon()-xp.getLon();
    double cosLatP = cos(xp.getLat());
    double sinLatP = sin(xp.getLat());
    double cosLatO = cos(xo.getLat());
    double sinLatO = sin(xo.getLat());
    double cosDlon = cos(dlon);
    double sinDlon = sin(dlon);

    double tmp1, tmp2, tmp3;

    tmp1 = cosLatO*sinDlon;
    tmp2 = cosLatO*sinLatP*cosDlon-cosLatP*sinLatO;
    double lon = atan2(tmp1, tmp2);
    if (lon < 0.0) lon = PI2+lon;

    tmp1 = sinLatO*sinLatP;
    tmp2 = cosLatO*cosLatP*cosDlon;
    tmp3 = tmp1+tmp2;
#ifdef DEBUG
    if (tmp3 < -1.0 || tmp3 > 1.0) {
        if (fabs(tmp3)-1.0 < EPS) {
            REPORT_WARNING("tmp3 is out of range [-1, 1]!")
            tmp3 = fmin(1.0, fmax(-1.0, tmp3));
        } else
            REPORT_ERROR("tmp3 is out of range [-1, 1]!")
    }
#endif
    double lat = asin(tmp3);

    xr.set(lon, lat, xo.getLev());
}

void Sphere::inverseRotate(const Coordinate &xp, Coordinate &xo,
                           const Coordinate &xr)
{
    double cosLatP = cos(xp.getLat());
    double sinLatP = sin(xp.getLat());
    double cosLonR = cos(xr.getLon());
    double sinLonR = sin(xr.getLon());
    double cosLatR = cos(xr.getLat());
    double sinLatR = sin(xr.getLat());

    double tmp1, tmp2, tmp3;

    tmp1 = cosLatR*sinLonR;
    tmp2 = sinLatR*cosLatP+cosLatR*cosLonR*sinLatP;
#ifdef DEBUG
    static const double eps = 1.0e-15;
    if (fabs(tmp2) < eps) {
        //REPORT_WARNING("tmp2 is near zero!")
        tmp2 = 0.0;
    }
#endif
    double lon = xp.getLon()+atan2(tmp1, tmp2);
    if (lon > PI2) lon -= PI2;
    if (lon < 0.0) lon += PI2;

    tmp1 = sinLatR*sinLatP;
    tmp2 = cosLatR*cosLatP*cosLonR;
    tmp3 = tmp1-tmp2;
#ifdef DEBUG
    if (tmp3 < -1.0 || tmp3 > 1.0) {
        REPORT_ERROR("tmp3 is out of range [-1,1]!")
    }
#endif
    double lat = asin(tmp3);
    xo.set(lon, lat, xr.getLev());
}

void Sphere::calcIntersect(const Coordinate &x1, const Coordinate &x2,
                           const Coordinate &x3, const Coordinate &x4,
                           Coordinate &x5, Coordinate &x6)
{
    double a =  x1.getY()*x2.getZ()-x1.getZ()*x2.getY();
    double b = -x1.getX()*x2.getZ()+x1.getZ()*x2.getX();
    double c =  x1.getX()*x2.getY()-x1.getY()*x2.getX();
    double d =  x3.getY()*x4.getZ()-x3.getZ()*x4.getY();
    double e = -x3.getX()*x4.getZ()+x3.getZ()*x4.getX();
    double f =  x3.getX()*x4.getY()-x3.getY()*x4.getX();

    double h = (d*c-f*a)/(e*a-d*b);
    double g = -(b*h+c)/a;
    double z = sqrt(radius2/(g*g+h*h+1.0));
    double x = g*z;
    double y = h*z;

    double lat1 = asin(z/radius);
    double lat2 = -lat1;
    double lon1 = atan2(y, x);
    double lon2 = lon1-PI;

    if (lon1 < 0.0) lon1 += PI2;
    if (lon1 > PI2) lon1 -= PI2;
    if (lon2 < 0.0) lon2 += PI2;
    if (lon2 > PI2) lon2 -= PI2;

    x5.set(lon1, lat1);
    x6.set(lon2, lat2);
}

void Sphere::calcIntersectLat(const Coordinate &x1, const Coordinate &x2,
                              double lon, Coordinate &x3, Coordinate &x4)
{
    Coordinate x5, x6;
    x5.set(lon, 0.0);
    x6.set(lon, PI05);
    calcIntersect(x1, x2, x5, x6, x3, x4);
}

inline void Sphere::calcIntersectLon(const Coordinate &x1, const Coordinate &x2,
                                     double lat, double &lon1, double &lon2)
{
    double a =  x1.getY()*x2.getZ()-x1.getZ()*x2.getY();
    double b = -x1.getX()*x2.getZ()+x1.getZ()*x2.getX();
    double c =  x1.getX()*x2.getY()-x1.getY()*x2.getX();

//    double z = radius*sin(lat);
    double z = sin(lat);
    double z2 = z*z;
    double a2 = a*a;
    double a2_plus_b2 = a2+b*b;
    double a2_plus_c2 = a2+c*c;
    double d = b*c*z/a2_plus_b2;
    double e = sqrt(d*d-(a2_plus_c2*z2-a2)/a2_plus_b2);

    double y1 = -d+e;
    double y2 = -d-e;

    lon1 = atan2(y1, (-b*y1-c*z)/a);
    lon2 = atan2(y2, (-b*y2-c*z)/a);
    if (lon1 < 0.0) lon1 += PI2;
    if (lon1 > PI2) lon1 -= PI2;
    if (lon2 < 0.0) lon2 += PI2;
    if (lon2 > PI2) lon2 -= PI2;
}

void Sphere::calcIntersectLon(const Coordinate &x1, const Coordinate &x2,
                              double lat, Coordinate &x3, Coordinate &x4)
{
    double lon1, lon2;
    calcIntersectLon(x1, x2, lat, lon1, lon2);
    x3.set(lon1, lat);
    x4.set(lon2, lat);
}

OrientStatus Sphere::orient(const Coordinate &x1, const Coordinate &x2,
                            const Coordinate &x3)
{
    static const double eps = 1.0e-16;

    double det = x3.getX()*(x1.getY()*x2.getZ()-x1.getZ()*x2.getY())-
                 x3.getY()*(x1.getX()*x2.getZ()-x1.getZ()*x2.getX())+
                 x3.getZ()*(x1.getX()*x2.getY()-x1.getY()*x2.getX());

    if (det > eps) {
        return OrientLeft;
    } else if (-det > eps) {
        return OrientRight;
    } else {
        return OrientOn;
    }
}

bool Sphere::is_lon_between(double lon1, double lon2, double lon)
{
    if (lon1 < 0) lon1 += PI2;
    if (lon1 > PI2) lon1 -= PI2;
    if (lon2 < 0) lon2 += PI2;
    if (lon2 > PI2) lon2 -= PI2;
    if (lon1 < lon2) {
        if (lon >= lon1 && lon < lon2)
            return true;
        else
            return false;
    } else {
        if ((lon >= lon1 && lon < PI2) || (lon >= 0.0 && lon < lon2))
            return true;
        else
            return false;
    }
}

bool Sphere::is_lon_gt(double lon1, double lon2)
{
    double dlon = lon1-lon2;
    if (dlon > 0.0) {
        if (PI2-dlon > dlon)
            return true;
        else
            return false;
    } else
        if (PI2+dlon < -dlon)
            return true;
        else
            return false;
}

bool Sphere::is_lon_lt(double lon1, double lon2)
{
    double dlon = lon1-lon2;
    if (dlon > 0.0) {
        if (PI2-dlon > dlon)
            return false;
        else
            return true;
    } else
        if (PI2+dlon < -dlon)
            return false;
        else
            return true;
}

bool Sphere::is_lon_eq(double lon1, double lon2)
{
    if (fabs(lon1-lon2) < EPS)
        return true;
    else if (fabs(PI2-fabs(lon1-lon2)) < EPS)
        return true;
    return false;
}

double Sphere::diff_lon(double lon1, double lon2)
{
    double dlon;
    if (lon1 > lon2) lon1 -= PI2;
    dlon = lon2-lon1;
    return dlon;
}

OrientStatus Sphere::orient(Point *endPoint1, Point *endPoint2, Point *point)
{
    double det;
    static double eps = 1.0e-16;

    double x1 = endPoint1->getCoordinate().getX();
    double x2 = endPoint2->getCoordinate().getX();
    double x3 = point->getCoordinate().getX();
    double y1 = endPoint1->getCoordinate().getY();
    double y2 = endPoint2->getCoordinate().getY();
    double y3 = point->getCoordinate().getY();
    double z1 = endPoint1->getCoordinate().getZ();
    double z2 = endPoint2->getCoordinate().getZ();
    double z3 = point->getCoordinate().getZ();

    det = x3*(y1*z2-z1*y2)-y3*(x1*z2-z1*x2)+z3*(x1*y2-y1*x2);

    if (det > eps) {
        return OrientLeft;
    } else if (-det > eps) {
        return OrientRight;
    } else {
        return OrientOn;
    }
}

bool Sphere::overlapTest(Point *point1, Point *point2)
{
    static double eps = 1.0e-4;
    double dlon, dlat;
    bool res = false;

    dlon = fabs(point1->getCoordinate().getLon()-
                point2->getCoordinate().getLon());
    dlat = fabs(point1->getCoordinate().getLat()-
                point2->getCoordinate().getLat());
    if (dlat < eps) {
        if (PI05-fabs(point1->getCoordinate().getLat()) < eps) {
            res = true;
        } else {
            if (dlon < eps || PI2-dlon < eps)
                res = true;
        }
    }
    return res;
}

InCircleStatus Sphere::inCircle(Point *point1, Point *point2,
                                Point *point3, Point *point)
{
    Point *points[3];
    points[0] = point1;
    points[1] = point2;
    points[2] = point3;

    double dx[3], dy[3], dz[3];

    for (int i = 0; i < 3; ++i) {
        dx[i] = points[i]->getCoordinate().getX()-
                point->getCoordinate().getX();
        dy[i] = points[i]->getCoordinate().getY()-
                point->getCoordinate().getY();
        dz[i] = points[i]->getCoordinate().getZ()-
                point->getCoordinate().getZ();
    }

    double det = dx[2]*(dy[1]*dz[0]-dy[0]*dz[1])-
                 dy[2]*(dx[1]*dz[0]-dx[0]*dz[1])+
                 dz[2]*(dx[1]*dy[0]-dx[0]*dy[1]);

    static double eps = 1.0e-16; // Tune it!

    if (det > eps) {
        return InsideCircle;
    } else if (-det > eps) {
        return OutsideCircle;
    } else {
        return OnCircle;
    }
}

int Sphere::inTriangle(Point *vertex1, Point *vertex2,
                       Point *vertex3, Point *point)
{
    Point *vertices[3];
    vertices[0] = vertex1;
    vertices[1] = vertex2;
    vertices[2] = vertex3;

    for (int i = 0; i < 3; ++i)
        if (Sphere::overlapTest(vertices[i], point))
            return -(i+1);

    int ip1, im1, k = -1, onPlane[2], res;
    OrientStatus ret;

    for (int i = 0; i < 3; ++i) {
        ip1 = i != 2 ? i+1 : 0;
        ret = Sphere::orient(vertices[i], vertices[ip1], point);
        if (ret == OrientRight) { // outside
            return -4;
        } else if (ret == OrientOn) {
            im1 = i != 0 ? i-1 : 2;
            onPlane[++k] = im1;
        }
    }
    if (k == -1) { // inside
        res = 4;
    } else if (k == 0) { // on the edge
        res = onPlane[k]+1;
    } else if (k == 1) { // on the vertex
        if (onPlane[0] == 0 && onPlane[1] == 1) {
            res = -3;
        } else if (onPlane[0] == 1 && onPlane[1] == 0) {
            res = -3;
        } else if (onPlane[0] == 1 && onPlane[1] == 2) {
            res = -1;
        } else if (onPlane[0] == 2 && onPlane[1] == 1) {
            res = -1;
        } else if (onPlane[0] == 0 && onPlane[1] == 2) {
            res = -2;
        } else if (onPlane[0] == 2 && onPlane[1] == 0) {
            res = -2;
        }
    }
    return res;

}
